// SPDX-License-Identifier: GPL-2.0-only
//
// SI-IN SIA9177 smart power amplifier driver (I2S in, I2C control).
//
// The part has no public datasheet. The register model and sequences come
// from the vendor sipa/sia91xx driver and from the parameter blob it loads
// (odm/firmware/sipa.bin v1.0.5 on the OnePlus 13R): 8-bit register
// addresses, 16-bit big-endian values, chip ID in register 0x06, and three
// data-driven register lists - init, startup, shutdown - of which the
// playback-scene values are baked in below, one set per speaker position.
// The two amplifiers share one I2S bus as clock consumers; each takes one
// slot, selected by register 0x16, and the chip tracks BCLK/WS on its own,
// so nothing is rate-dependent here.
//
// The vendor stack additionally runs an excursion-protection algorithm in
// the ADSP, fed by IV sense on the second data line. Nothing here drives
// it; the baked-in gains are the vendor's own tuning, but sustained
// high-level playback has no excursion limiter guarding it.

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#define SIA9177_REG_STATE	0x01	/* low nibble: 0 standby, 5 running */
#define SIA9177_REG_INT_STAT	0x08
#define SIA9177_REG_INT_CLEAR	0x09
#define SIA9177_REG_CTRL	0x13	/* bit2 amp enable, bit0 standby */
#define SIA9177_REG_CHIP_ID	0x06

#define SIA9177_STATE_MASK	0x000f
#define SIA9177_STATE_STANDBY	0x0000
#define SIA9177_STATE_RUNNING	0x0005

/*
 * Reset pin polarity: driving it high holds the chip in reset/powerdown,
 * low lets it run (the vendor driver's SIA91XX_ENABLE_LEVEL is 0).
 */

struct sia9177_tuning {
	const struct reg_sequence *init;
	unsigned int init_len;
	const struct reg_sequence *start;
	unsigned int start_len;
};

struct sia9177_priv {
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct clk *bclk;
	unsigned long bclk_rate;
	const struct sia9177_tuning *tuning;
};

/*
 * Playback-scene register values from the OnePlus 13R parameter blob.
 * All are opaque tuning apart from what is named above; the left/right
 * differences are the I2S slot select in 0x16 and per-speaker gains.
 */
static const struct reg_sequence sia9177_init_left[] = {
	{ 0x17, 0x0474 }, { 0x18, 0x16b1 }, { 0x19, 0x2c28 },
	{ 0x1a, 0x6400 }, { 0x1b, 0x83bb }, { 0x1c, 0x8b40 },
	{ 0x1d, 0xae80 }, { 0x1e, 0x8000 }, { 0x20, 0x8322 },
	{ 0x23, 0x1aa8 }, { 0x24, 0x4800 }, { 0x25, 0x0800 },
	{ 0x29, 0x6000 }, { 0x2b, 0x90f0 }, { 0x2d, 0x12fd },
	{ 0x2e, 0x63df }, { 0x30, 0x0000 }, { 0x31, 0x800b },
};

static const struct reg_sequence sia9177_init_right[] = {
	{ 0x17, 0x0474 }, { 0x18, 0x16b1 }, { 0x19, 0x3428 },
	{ 0x1a, 0x6400 }, { 0x1b, 0x83cc }, { 0x1c, 0x8b40 },
	{ 0x1d, 0xae80 }, { 0x1e, 0x8000 }, { 0x20, 0x8322 },
	{ 0x23, 0x1aa8 }, { 0x24, 0x4800 }, { 0x25, 0x0800 },
	{ 0x29, 0x6000 }, { 0x2b, 0x90f0 }, { 0x2d, 0x12fd },
	{ 0x2e, 0x601f }, { 0x30, 0x0000 }, { 0x31, 0x800b },
};

static const struct reg_sequence sia9177_start_left[] = {
	{ 0x14, 0x93a8 }, { 0x15, 0x8808 }, { 0x16, 0x003b },
	{ 0x12, 0x9d60 }, { 0x13, 0x0384 }, { 0x17, 0x0474 },
};

static const struct reg_sequence sia9177_start_right[] = {
	{ 0x14, 0x93a8 }, { 0x15, 0x8848 }, { 0x16, 0x0b30 },
	{ 0x12, 0x9d60 }, { 0x13, 0x0384 }, { 0x17, 0x0474 },
};

static const struct sia9177_tuning sia9177_tuning_left = {
	.init = sia9177_init_left,
	.init_len = ARRAY_SIZE(sia9177_init_left),
	.start = sia9177_start_left,
	.start_len = ARRAY_SIZE(sia9177_start_left),
};

static const struct sia9177_tuning sia9177_tuning_right = {
	.init = sia9177_init_right,
	.init_len = ARRAY_SIZE(sia9177_init_right),
	.start = sia9177_start_right,
	.start_len = ARRAY_SIZE(sia9177_start_right),
};

/*
 * The chip latches its clock configuration when BCLK first appears, so
 * bring-up is split the way the vendor driver splits it: reset release and
 * the init writes happen at stream open, while the bus is still silent; the
 * startup sequence runs at unmute, after the DSP has started the clocks.
 * Running the whole of it on either side of the clock edge leaves the chip
 * parked in state 2 and silent, with no fault latched - proven on hardware
 * both ways.
 */
static int sia9177_configure(struct snd_soc_component *component)
{
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int val;
	int ret;

	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	usleep_range(5000, 6000);

	ret = regmap_multi_reg_write(priv->regmap, priv->tuning->init,
				     priv->tuning->init_len);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, SIA9177_REG_STATE, &val);
	if (ret)
		return ret;
	if ((val & SIA9177_STATE_MASK) != SIA9177_STATE_STANDBY)
		dev_warn(component->dev, "not in standby after init: %#x\n",
			 val);

	/* clear whatever interrupt state reset left behind */
	regmap_read(priv->regmap, SIA9177_REG_INT_STAT, &val);
	regmap_write(priv->regmap, SIA9177_REG_INT_CLEAR, 0xffff);

	return 0;
}

static int sia9177_power_up(struct snd_soc_component *component)
{
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int val;
	int ret;

	/*
	 * On AudioReach nothing else turns the MI2S bit clock on - the DSP
	 * paces the stream data without it and every SoC-side counter stays
	 * clean, so its absence is invisible from the host. The amp is the
	 * only thing that notices.
	 */
	if (priv->bclk) {
		clk_set_rate(priv->bclk, priv->bclk_rate);
		ret = clk_prepare_enable(priv->bclk);
		if (ret)
			return ret;
	}

	ret = regmap_multi_reg_write(priv->regmap, priv->tuning->start,
				     priv->tuning->start_len);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, SIA9177_REG_STATE, &val);
	if (ret)
		return ret;
	if ((val & SIA9177_STATE_MASK) != SIA9177_STATE_RUNNING)
		dev_warn(component->dev, "failed to start: state %#x\n", val);

	return 0;
}

static void sia9177_power_down(struct snd_soc_component *component)
{
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int val;

	regmap_update_bits(priv->regmap, SIA9177_REG_CTRL, BIT(2), 0);
	usleep_range(1000, 1500);
	regmap_update_bits(priv->regmap, SIA9177_REG_CTRL, BIT(0), BIT(0));
	usleep_range(1000, 1500);

	if (!regmap_read(priv->regmap, SIA9177_REG_STATE, &val) &&
	    (val & SIA9177_STATE_MASK) != SIA9177_STATE_STANDBY)
		dev_warn(component->dev, "failed to stop: state %#x\n", val);

	if (priv->bclk)
		clk_disable_unprepare(priv->bclk);
}

static int sia9177_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct sia9177_priv *priv =
		snd_soc_component_get_drvdata(dai->component);

	/* two slots at the stream's physical width */
	priv->bclk_rate = params_rate(params) * 2 *
		snd_pcm_format_physical_width(params_format(params));

	return 0;
}

static int sia9177_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	return sia9177_configure(dai->component);
}

static void sia9177_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct sia9177_priv *priv =
		snd_soc_component_get_drvdata(dai->component);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
}

static int sia9177_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	if (mute) {
		sia9177_power_down(component);
		return 0;
	}

	return sia9177_power_up(component);
}

static const struct snd_soc_dai_ops sia9177_dai_ops = {
	.startup = sia9177_startup,
	.shutdown = sia9177_shutdown,
	.hw_params = sia9177_hw_params,
	.mute_stream = sia9177_mute_stream,
	/*
	 * Unmute at trigger, right after the CPU DAI's trigger has started
	 * the DSP port - the closest the core gets to the vendor's ordering,
	 * where the startup sequence runs against live-or-imminent clocks
	 * while the init writes above happened on a silent bus.
	 */
	.mute_unmute_on_trigger = true,
};

static struct snd_soc_dai_driver sia9177_dai = {
	.name = "sia9177-aif",
	.playback = {
		.stream_name = "Playback",
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
		.channels_min = 1,
		.channels_max = 2,
	},
	.ops = &sia9177_dai_ops,
};

static const struct snd_soc_dapm_widget sia9177_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route sia9177_dapm_routes[] = {
	{ "OUT", NULL, "AIF IN" },
};

static const struct snd_soc_component_driver sia9177_component = {
	.dapm_widgets = sia9177_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sia9177_dapm_widgets),
	.dapm_routes = sia9177_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(sia9177_dapm_routes),
	.use_pmdown_time = 1,
	.endianness = 1,
};

static const struct regmap_config sia9177_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xff,
};

static int sia9177_detect(struct device *dev, struct sia9177_priv *priv)
{
	unsigned int id;
	int ret;

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	usleep_range(5000, 6000);

	ret = regmap_read(priv->regmap, SIA9177_REG_CHIP_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read chip ID\n");

	/* ID ranges from the vendor driver's SIA9177 match table */
	if (id != 0x5880 && (id < 0x5890 || id > 0x5891) &&
	    (id < 0x58a0 || id > 0x58a1))
		return dev_err_probe(dev, -ENODEV,
				     "unexpected chip ID %#06x\n", id);

	dev_dbg(dev, "found SIA9177, ID %#06x\n", id);

	/* park it in powerdown until playback starts */
	gpiod_set_value_cansleep(priv->reset_gpio, 1);

	return 0;
}

static int sia9177_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct sia9177_priv *priv;
	u32 channel = 0;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = devm_regmap_init_i2c(i2c, &sia9177_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "cannot init regmap\n");

	priv->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "cannot get reset gpio\n");

	priv->bclk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(priv->bclk))
		return dev_err_probe(dev, PTR_ERR(priv->bclk),
				     "cannot get bit clock\n");
	priv->bclk_rate = 3072000;

	device_property_read_u32(dev, "si,channel-num", &channel);
	priv->tuning = channel ? &sia9177_tuning_right : &sia9177_tuning_left;

	i2c_set_clientdata(i2c, priv);

	ret = sia9177_detect(dev, priv);
	if (ret)
		return ret;

	return devm_snd_soc_register_component(dev, &sia9177_component,
					       &sia9177_dai, 1);
}

static const struct of_device_id sia9177_of_match[] = {
	{ .compatible = "si,sia9177" },
	{ }
};
MODULE_DEVICE_TABLE(of, sia9177_of_match);

static const struct i2c_device_id sia9177_i2c_id[] = {
	{ "sia9177" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sia9177_i2c_id);

static struct i2c_driver sia9177_i2c_driver = {
	.driver = {
		.name = "sia9177",
		.of_match_table = sia9177_of_match,
	},
	.probe = sia9177_i2c_probe,
	.id_table = sia9177_i2c_id,
};
module_i2c_driver(sia9177_i2c_driver);

MODULE_DESCRIPTION("SI-IN SIA9177 amplifier driver");
MODULE_LICENSE("GPL");
