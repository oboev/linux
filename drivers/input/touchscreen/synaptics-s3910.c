// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Synaptics TouchComm (TCM) touchscreens over SPI, written for
 * the S3910.
 *
 * Derived from the TCM oncell I2C driver:
 *   Copyright (c) 2024 Frieder Hannenheim <frieder.hannenheim@proton.me>
 *   Copyright (c) 2024 Caleb Connolly <caleb@postmarketos.org>
 * Copyright (c) 2026 Aleksey Solovyev <aleksey.solovyev@protonmail.com>
 *
 * The controller boots its application firmware from its own flash; no
 * firmware download is needed. Rather than parsing whatever touch report
 * format the firmware defaults to, this driver programs its own byte-aligned
 * format (CMD_SET_TOUCH_REPORT_CONFIG) and parses exactly that.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define TCM_MARKER				0xa5

/* commands */
#define TCM_CMD_IDENTIFY			0x02
#define TCM_CMD_ENABLE_REPORT			0x05
#define TCM_CMD_DISABLE_REPORT			0x06
#define TCM_CMD_RUN_APPLICATION_FIRMWARE	0x14
#define TCM_CMD_GET_APPLICATION_INFO		0x20
#define TCM_CMD_SET_TOUCH_REPORT_CONFIG		0x26

/* status codes (message code field, below 0x10) */
#define TCM_STATUS_IDLE				0x00
#define TCM_STATUS_OK				0x01
#define TCM_STATUS_BUSY				0x02
#define TCM_STATUS_CONTINUED_READ		0x03
#define TCM_STATUS_RECEIVE_BUFFER_OVERFLOW	0x0c
#define TCM_STATUS_PREVIOUS_COMMAND_PENDING	0x0d
#define TCM_STATUS_NOT_IMPLEMENTED		0x0e
#define TCM_STATUS_ERROR			0x0f

/* report types (message code field, 0x10 and up) */
#define TCM_REPORT_IDENTIFY			0x10
#define TCM_REPORT_TOUCH			0x11

#define TCM_MODE_APPLICATION			0x01

#define TCM_APP_STATUS_OK			0x00
#define TCM_APP_STATUS_BOOTING			0x01
#define TCM_APP_STATUS_UPDATING			0x02

/* touch report config entities */
#define TCM_TOUCH_END				0x00
#define TCM_TOUCH_FOREACH_ACTIVE_OBJECT		0x01
#define TCM_TOUCH_FOREACH_END			0x03
#define TCM_TOUCH_OBJECT_N_INDEX		0x06
#define TCM_TOUCH_OBJECT_N_CLASSIFICATION	0x07
#define TCM_TOUCH_OBJECT_N_X_POSITION		0x08
#define TCM_TOUCH_OBJECT_N_Y_POSITION		0x09
#define TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS		0x18

/*
 * Every message the chip sends fits well inside this: identification is 28
 * bytes, application info ~60, and a 10-finger touch report in the format
 * below is 65. Reading past the end of a message returns padding, and
 * reading a whole message at once means never dealing with continued reads.
 */
#define TCM_READ_SIZE				256

#define TCM_MAX_FINGERS				10

/* the power-on and reset timing the vendor tree uses for this part */
#define TCM_POWER_ON_DELAY_MS			200
#define TCM_RESET_ACTIVE_US			10000
#define TCM_RESET_SETTLE_MS			85

#define TCM_RESPONSE_TIMEOUT_MS			1000

struct tcm_message_header {
	u8 marker;
	u8 code;
	__le16 length;
} __packed;

struct tcm_identification {
	struct tcm_message_header header;
	u8 version;
	u8 mode;
	char part_number[16];
	u8 build_id[4];
	u8 max_write_size[2];
} __packed;

struct tcm_app_info {
	struct tcm_message_header header;
	u8 version[2];
	__le16 status;
	u8 static_config_size[2];
	u8 dynamic_config_size[2];
	u8 app_config_start_write_block[2];
	u8 app_config_size[2];
	u8 max_touch_report_config_size[2];
	u8 max_touch_report_payload_size[2];
	char customer_config_id[16];
	__le16 max_x;
	__le16 max_y;
	__le16 max_objects;
	u8 num_of_buttons[2];
	u8 num_of_image_rows[2];
	u8 num_of_image_cols[2];
	u8 has_hybrid_data[2];
} __packed;

/*
 * The touch report format this driver programs into the chip. Data entities
 * carry a bit-count byte; the foreach/end control codes do not. Everything
 * is byte-aligned on purpose so the report needs no bit-stream decoding:
 *
 *   [count u8] then per active object [index u8][class u8][x le16][y le16]
 */
static const u8 tcm_touch_config[] = {
	TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS,	8,
	TCM_TOUCH_FOREACH_ACTIVE_OBJECT,
	TCM_TOUCH_OBJECT_N_INDEX,		8,
	TCM_TOUCH_OBJECT_N_CLASSIFICATION,	8,
	TCM_TOUCH_OBJECT_N_X_POSITION,		16,
	TCM_TOUCH_OBJECT_N_Y_POSITION,		16,
	TCM_TOUCH_FOREACH_END,
	TCM_TOUCH_END,
};

struct tcm_touch_object {
	u8 index;
	u8 classification;
	__le16 x;
	__le16 y;
} __packed;

struct tcm_data {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[2];
	struct touchscreen_properties props;

	/* DMA-safe transfer buffers */
	u8 *tx_fill;
	u8 *rx_buf;
	u8 *cmd_buf;

	/* serializes command/response exchanges */
	struct mutex cmd_lock;
	struct completion response;
	/* last non-touch message, stashed by the IRQ handler */
	u8 msg[TCM_READ_SIZE];
	u16 msg_len;
};

static int tcm_send_cmd(struct tcm_data *tcm, u8 cmd,
			const void *payload, u16 length)
{
	tcm->cmd_buf[0] = cmd;
	put_unaligned_le16(length, &tcm->cmd_buf[1]);
	if (length)
		memcpy(&tcm->cmd_buf[3], payload, length);

	return spi_write(tcm->spi, tcm->cmd_buf, 3 + length);
}

static int tcm_read_message(struct tcm_data *tcm)
{
	struct spi_transfer xfer = {
		.tx_buf = tcm->tx_fill,
		.rx_buf = tcm->rx_buf,
		.len = TCM_READ_SIZE,
	};

	return spi_sync_transfer(tcm->spi, &xfer, 1);
}

static void tcm_report_touches(struct tcm_data *tcm, const u8 *payload,
			       u16 length)
{
	const struct tcm_touch_object *obj;
	unsigned int count, i;

	if (!tcm->input)
		return;

	if (length < 1)
		return;

	count = payload[0];
	if (length < 1 + count * sizeof(*obj)) {
		dev_err_ratelimited(&tcm->spi->dev,
				    "short touch report: %u objects in %u bytes\n",
				    count, length);
		return;
	}

	obj = (const struct tcm_touch_object *)&payload[1];
	for (i = 0; i < count; i++, obj++) {
		u16 x = get_unaligned_le16(&obj->x);
		u16 y = get_unaligned_le16(&obj->y);

		if (obj->index >= TCM_MAX_FINGERS)
			continue;

		input_mt_slot(tcm->input, obj->index);
		input_mt_report_slot_state(tcm->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(tcm->input, &tcm->props, x, y, true);
	}

	input_mt_sync_frame(tcm->input);
	input_sync(tcm->input);
}

static irqreturn_t tcm_irq(int irq, void *data)
{
	struct tcm_data *tcm = data;
	struct tcm_message_header *header;
	u16 len;
	int ret;

	ret = tcm_read_message(tcm);
	if (ret) {
		dev_err_ratelimited(&tcm->spi->dev,
				    "failed to read message: %d\n", ret);
		return IRQ_HANDLED;
	}

	header = (struct tcm_message_header *)tcm->rx_buf;
	if (header->marker != TCM_MARKER)
		return IRQ_HANDLED;

	if (header->code == TCM_STATUS_IDLE)
		return IRQ_HANDLED;

	len = le16_to_cpu(header->length);
	if (len > TCM_READ_SIZE - sizeof(*header)) {
		dev_err_ratelimited(&tcm->spi->dev,
				    "message %#x too long (%u bytes)\n",
				    header->code, len);
		return IRQ_HANDLED;
	}

	if (header->code == TCM_REPORT_TOUCH) {
		tcm_report_touches(tcm, tcm->rx_buf + sizeof(*header), len);
		return IRQ_HANDLED;
	}

	memcpy(tcm->msg, tcm->rx_buf, sizeof(*header) + len);
	tcm->msg_len = sizeof(*header) + len;
	complete(&tcm->response);

	return IRQ_HANDLED;
}

/*
 * Wait until the IRQ handler stashes a message with the wanted code,
 * skipping over unrelated ones. Must be called with cmd_lock held and the
 * completion reinitialized before the triggering action.
 */
static int tcm_wait_for(struct tcm_data *tcm, u8 code, void *buf, size_t length)
{
	unsigned long timeout = msecs_to_jiffies(TCM_RESPONSE_TIMEOUT_MS);
	u8 got;

	for (;;) {
		timeout = wait_for_completion_timeout(&tcm->response, timeout);
		if (timeout == 0)
			return -ETIMEDOUT;

		got = tcm->msg[1];
		if (got == code)
			break;

		switch (got) {
		case TCM_STATUS_BUSY:
		case TCM_STATUS_OK:
		case TCM_REPORT_IDENTIFY:
			/* not what we asked for - keep waiting */
			reinit_completion(&tcm->response);
			continue;
		default:
			dev_err(&tcm->spi->dev,
				"error response %#x waiting for %#x\n",
				got, code);
			return -EIO;
		}
	}

	if (buf)
		memcpy(buf, tcm->msg, min_t(size_t, length, tcm->msg_len));

	return 0;
}

static int tcm_request(struct tcm_data *tcm, u8 cmd,
		       const void *payload, u16 payload_len,
		       u8 resp_code, void *buf, size_t length)
{
	int ret;

	guard(mutex)(&tcm->cmd_lock);

	reinit_completion(&tcm->response);
	ret = tcm_send_cmd(tcm, cmd, payload, payload_len);
	if (ret)
		return ret;

	return tcm_wait_for(tcm, resp_code, buf, length);
}

static int tcm_power_on(struct tcm_data *tcm)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
	if (ret)
		return ret;

	msleep(TCM_POWER_ON_DELAY_MS);

	gpiod_set_value_cansleep(tcm->reset_gpio, 1);
	usleep_range(TCM_RESET_ACTIVE_US, TCM_RESET_ACTIVE_US + 1000);
	gpiod_set_value_cansleep(tcm->reset_gpio, 0);
	msleep(TCM_RESET_SETTLE_MS);

	return 0;
}

static void tcm_power_off(void *data)
{
	struct tcm_data *tcm = data;

	disable_irq(tcm->spi->irq);
	gpiod_set_value_cansleep(tcm->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
}

static int tcm_input_open(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);
	u8 report = TCM_REPORT_TOUCH;

	return tcm_request(tcm, TCM_CMD_ENABLE_REPORT, &report, 1,
			   TCM_STATUS_OK, NULL, 0);
}

static void tcm_input_close(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);
	u8 report = TCM_REPORT_TOUCH;
	int ret;

	ret = tcm_request(tcm, TCM_CMD_DISABLE_REPORT, &report, 1,
			  TCM_STATUS_OK, NULL, 0);
	if (ret)
		dev_err(&tcm->spi->dev, "failed to disable reporting: %d\n",
			ret);
}

static int tcm_hw_init(struct tcm_data *tcm, u16 *max_x, u16 *max_y,
		       u16 *max_objects)
{
	struct tcm_identification id = { 0 };
	struct tcm_app_info app_info = { 0 };
	unsigned int tries;
	u16 status;
	int ret;

	/*
	 * Coming out of reset the chip raises the interrupt with an
	 * unsolicited IDENTIFY report. Wait for it; ask explicitly only if
	 * it does not arrive.
	 */
	scoped_guard(mutex, &tcm->cmd_lock) {
		ret = tcm_wait_for(tcm, TCM_REPORT_IDENTIFY, &id, sizeof(id));
	}
	if (ret) {
		ret = tcm_request(tcm, TCM_CMD_IDENTIFY, NULL, 0,
				  TCM_STATUS_OK, &id, sizeof(id));
		if (ret) {
			dev_err(&tcm->spi->dev, "chip did not identify: %d\n",
				ret);
			return ret;
		}
	}

	dev_info(&tcm->spi->dev, "Synaptics TCM %.16s, protocol v%u, mode %#x\n",
		 id.part_number, id.version, id.mode);

	if (id.mode != TCM_MODE_APPLICATION) {
		ret = tcm_request(tcm, TCM_CMD_RUN_APPLICATION_FIRMWARE,
				  NULL, 0, TCM_REPORT_IDENTIFY,
				  &id, sizeof(id));
		if (ret) {
			dev_err(&tcm->spi->dev,
				"failed to start application firmware: %d\n",
				ret);
			return ret;
		}
		if (id.mode != TCM_MODE_APPLICATION) {
			dev_err(&tcm->spi->dev,
				"chip stuck in mode %#x\n", id.mode);
			return -ENODEV;
		}
	}

	for (tries = 0; ; tries++) {
		ret = tcm_request(tcm, TCM_CMD_GET_APPLICATION_INFO, NULL, 0,
				  TCM_STATUS_OK, &app_info, sizeof(app_info));
		if (ret) {
			dev_err(&tcm->spi->dev,
				"failed to get application info: %d\n", ret);
			return ret;
		}

		status = le16_to_cpu(app_info.status);
		if (status != TCM_APP_STATUS_BOOTING &&
		    status != TCM_APP_STATUS_UPDATING)
			break;
		if (tries > 50)
			return -ETIMEDOUT;
		msleep(20);
	}

	if (status != TCM_APP_STATUS_OK) {
		dev_err(&tcm->spi->dev, "application firmware status %#x\n",
			status);
		return -ENODEV;
	}

	dev_info(&tcm->spi->dev,
		 "application firmware v%u.%u, customer config '%.16s'\n",
		 app_info.version[0], app_info.version[1],
		 app_info.customer_config_id);

	ret = tcm_request(tcm, TCM_CMD_SET_TOUCH_REPORT_CONFIG,
			  tcm_touch_config, sizeof(tcm_touch_config),
			  TCM_STATUS_OK, NULL, 0);
	if (ret) {
		dev_err(&tcm->spi->dev,
			"failed to set touch report config: %d\n", ret);
		return ret;
	}

	*max_x = le16_to_cpu(app_info.max_x);
	*max_y = le16_to_cpu(app_info.max_y);
	*max_objects = le16_to_cpu(app_info.max_objects);

	return 0;
}

static int tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct tcm_data *tcm;
	u16 max_x, max_y, max_objects;
	int ret;

	tcm = devm_kzalloc(dev, sizeof(*tcm), GFP_KERNEL);
	if (!tcm)
		return -ENOMEM;

	tcm->spi = spi;
	spi_set_drvdata(spi, tcm);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	tcm->tx_fill = devm_kmalloc(dev, TCM_READ_SIZE, GFP_KERNEL);
	tcm->rx_buf = devm_kmalloc(dev, TCM_READ_SIZE, GFP_KERNEL);
	tcm->cmd_buf = devm_kmalloc(dev, 64, GFP_KERNEL);
	if (!tcm->tx_fill || !tcm->rx_buf || !tcm->cmd_buf)
		return -ENOMEM;
	memset(tcm->tx_fill, 0xff, TCM_READ_SIZE);

	init_completion(&tcm->response);
	ret = devm_mutex_init(dev, &tcm->cmd_lock);
	if (ret)
		return ret;

	tcm->supplies[0].supply = "vdd";
	tcm->supplies[1].supply = "avdd";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(tcm->supplies),
				      tcm->supplies);
	if (ret)
		return ret;

	/* hold the chip in reset while it powers up */
	tcm->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(tcm->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(tcm->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, tcm_irq,
					IRQF_ONESHOT, "synaptics_tcm", tcm);
	if (ret)
		return ret;

	ret = tcm_power_on(tcm);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, tcm_power_off, tcm);
	if (ret)
		return ret;

	ret = tcm_hw_init(tcm, &max_x, &max_y, &max_objects);
	if (ret)
		return ret;

	tcm->input = devm_input_allocate_device(dev);
	if (!tcm->input)
		return -ENOMEM;

	tcm->input->name = "Synaptics TCM Touchscreen";
	tcm->input->id.bustype = BUS_SPI;
	tcm->input->open = tcm_input_open;
	tcm->input->close = tcm_input_close;

	input_set_abs_params(tcm->input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);

	touchscreen_parse_properties(tcm->input, true, &tcm->props);

	ret = input_mt_init_slots(tcm->input,
				  clamp_t(u16, max_objects, 1, TCM_MAX_FINGERS),
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	input_set_drvdata(tcm->input, tcm);

	return input_register_device(tcm->input);
}

static const struct of_device_id tcm_of_match[] = {
	{ .compatible = "syna,s3910" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcm_of_match);

static const struct spi_device_id tcm_spi_ids[] = {
	{ "s3910" },
	{ }
};
MODULE_DEVICE_TABLE(spi, tcm_spi_ids);

static struct spi_driver tcm_spi_driver = {
	.probe = tcm_probe,
	.id_table = tcm_spi_ids,
	.driver = {
		.name = "synaptics-tcm-spi",
		.of_match_table = tcm_of_match,
	},
};
module_spi_driver(tcm_spi_driver);

MODULE_AUTHOR("Aleksey Solovyev <aleksey.solovyev@protonmail.com>");
MODULE_DESCRIPTION("Synaptics TouchComm SPI touchscreen driver");
MODULE_LICENSE("GPL");
