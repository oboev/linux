// SPDX-License-Identifier: GPL-2.0-only
//
// SI-IN SIA9177 smart power amplifier driver (I2S in, I2C control).
//
// The part has no public datasheet. The register model and sequences come
// from the vendor sipa/sia91xx driver and from the parameter blob it loads
// (odm/firmware/sipa.bin v1.0.5 on the OnePlus 13R): 8-bit register
// addresses, 16-bit big-endian values, chip ID in register 0x06, and three
// data-driven register lists - init, startup, shutdown - which the blob
// carries once per audio scene and per speaker position; the two scenes
// this board has a use for are baked in below. The two amplifiers share one
// I2S bus as clock consumers; each takes one slot, selected by register
// 0x16, and the chip tracks BCLK/WS on its own, so nothing is rate-dependent
// here.
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
#include <linux/mutex.h>
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
 * Fault bits in INT_STAT, from the vendor driver's irq_range[]. Sticky:
 * cleared by writing ones to INT_CLEAR, or by reset.
 */
#define SIA9177_INT_POR		BIT(0)
#define SIA9177_INT_NOCLK	BIT(2)
#define SIA9177_INT_OTP		BIT(3)
#define SIA9177_INT_UVP		BIT(5)
#define SIA9177_INT_OCP		BIT(6)
#define SIA9177_INT_TDMERR	BIT(7)

/*
 * The chip ramps 2 -> 4 -> 5 over some milliseconds once the startup
 * sequence has landed, so RUN has to be watched for rather than read once.
 */
#define SIA9177_RUN_POLL_US	1000
#define SIA9177_RUN_TIMEOUT_US	20000

/*
 * Reset pin polarity: driving it high holds the chip in reset/powerdown,
 * low lets it run (the vendor driver's SIA91XX_ENABLE_LEVEL is 0).
 */

/*
 * The blob defines six scenes. VOICE is byte-identical to PLAYBACK on this
 * board and the three MMI_* ones are factory test modes that drive a single
 * amplifier, so only these two are worth exposing.
 */
enum sia9177_scene {
	SIA9177_SCENE_SPEAKER,
	SIA9177_SCENE_RECEIVER,
	SIA9177_SCENE_COUNT,
};

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
	bool bclk_on;
	/* the fitted position's tuning, indexed by enum sia9177_scene */
	const struct sia9177_tuning *tuning;
	/* serialises a scene or switch change against the stream's own transitions */
	struct mutex lock;
	unsigned int scene;
	bool enabled;
	bool powered;
	bool running;
};

/*
 * Register values from the OnePlus 13R parameter blob, per scene and per
 * speaker position. All are opaque tuning apart from what is named above;
 * the left/right differences are the I2S slot select in 0x16 and per-speaker
 * gains. Receiver lowers the gains, for a part held against an ear rather
 * than played into a room.
 *
 * Both positions carry a receiver column, but only the top one is ever
 * asked for it: the vendor's handset path selects the scene on channel 0
 * and leaves channel 1 out of the stream entirely, so channel 1's receiver
 * values are carried here for completeness and nothing selects them.
 */
static const struct reg_sequence sia9177_init_left_speaker[] = {
	{ 0x17, 0x0474 }, { 0x18, 0x16b1 }, { 0x19, 0x2c28 },
	{ 0x1a, 0x6400 }, { 0x1b, 0x83bb }, { 0x1c, 0x8b40 },
	{ 0x1d, 0xae80 }, { 0x1e, 0x8000 }, { 0x20, 0x8322 },
	{ 0x23, 0x1aa8 }, { 0x24, 0x4800 }, { 0x25, 0x0800 },
	{ 0x29, 0x6000 }, { 0x2b, 0x90f0 }, { 0x2d, 0x12fd },
	{ 0x2e, 0x63df }, { 0x30, 0x0000 }, { 0x31, 0x800b },
};

static const struct reg_sequence sia9177_init_left_receiver[] = {
	{ 0x17, 0x0474 }, { 0x18, 0x1681 }, { 0x19, 0x2428 },
	{ 0x1a, 0x6400 }, { 0x1b, 0x8366 }, { 0x1c, 0x8ac0 },
	{ 0x1d, 0xae80 }, { 0x1e, 0x8000 }, { 0x20, 0x8321 },
	{ 0x23, 0x1aa8 }, { 0x24, 0x4800 }, { 0x25, 0x0800 },
	{ 0x29, 0x6000 }, { 0x2b, 0x90f0 }, { 0x2d, 0x12fc },
	{ 0x2e, 0x635f }, { 0x30, 0x0000 }, { 0x31, 0x800b },
};

static const struct reg_sequence sia9177_init_right_speaker[] = {
	{ 0x17, 0x0474 }, { 0x18, 0x16b1 }, { 0x19, 0x3428 },
	{ 0x1a, 0x6400 }, { 0x1b, 0x83cc }, { 0x1c, 0x8b40 },
	{ 0x1d, 0xae80 }, { 0x1e, 0x8000 }, { 0x20, 0x8322 },
	{ 0x23, 0x1aa8 }, { 0x24, 0x4800 }, { 0x25, 0x0800 },
	{ 0x29, 0x6000 }, { 0x2b, 0x90f0 }, { 0x2d, 0x12fd },
	{ 0x2e, 0x601f }, { 0x30, 0x0000 }, { 0x31, 0x800b },
};

static const struct reg_sequence sia9177_init_right_receiver[] = {
	{ 0x17, 0x0474 }, { 0x18, 0x1681 }, { 0x19, 0x2428 },
	{ 0x1a, 0x6400 }, { 0x1b, 0x8300 }, { 0x1c, 0x8ac0 },
	{ 0x1d, 0xae86 }, { 0x1e, 0x8000 }, { 0x20, 0x8321 },
	{ 0x23, 0x1aa8 }, { 0x24, 0x4800 }, { 0x25, 0x0800 },
	{ 0x29, 0x6000 }, { 0x2b, 0x90f0 }, { 0x2d, 0x12fc },
	{ 0x2e, 0x635f }, { 0x30, 0x0000 }, { 0x31, 0x800b },
};

static const struct reg_sequence sia9177_start_left_speaker[] = {
	{ 0x14, 0x93a8 }, { 0x15, 0x8808 }, { 0x16, 0x003b },
	{ 0x12, 0x9d60 }, { 0x13, 0x0384 }, { 0x17, 0x0474 },
};

static const struct reg_sequence sia9177_start_left_receiver[] = {
	{ 0x14, 0x9ba8 }, { 0x15, 0x8808 }, { 0x16, 0x003b },
	{ 0x12, 0x9d60 }, { 0x13, 0x0384 }, { 0x17, 0x0474 },
};

static const struct reg_sequence sia9177_start_right_speaker[] = {
	{ 0x14, 0x93a8 }, { 0x15, 0x8848 }, { 0x16, 0x0b30 },
	{ 0x12, 0x9d60 }, { 0x13, 0x0384 }, { 0x17, 0x0474 },
};

static const struct reg_sequence sia9177_start_right_receiver[] = {
	{ 0x14, 0x93a8 }, { 0x15, 0x8808 }, { 0x16, 0x0b30 },
	{ 0x12, 0x9d60 }, { 0x13, 0x0384 }, { 0x17, 0x0474 },
};

#define SIA9177_TUNING(_position, _scene) {				\
	.init = sia9177_init_##_position##_##_scene,			\
	.init_len = ARRAY_SIZE(sia9177_init_##_position##_##_scene),	\
	.start = sia9177_start_##_position##_##_scene,			\
	.start_len = ARRAY_SIZE(sia9177_start_##_position##_##_scene),	\
}

static const struct sia9177_tuning sia9177_tuning_left[SIA9177_SCENE_COUNT] = {
	[SIA9177_SCENE_SPEAKER]  = SIA9177_TUNING(left, speaker),
	[SIA9177_SCENE_RECEIVER] = SIA9177_TUNING(left, receiver),
};

static const struct sia9177_tuning sia9177_tuning_right[SIA9177_SCENE_COUNT] = {
	[SIA9177_SCENE_SPEAKER]  = SIA9177_TUNING(right, speaker),
	[SIA9177_SCENE_RECEIVER] = SIA9177_TUNING(right, receiver),
};

static void sia9177_report_faults(struct snd_soc_component *component,
				  const char *when, unsigned int stat)
{
	dev_warn(component->dev, "faults latched %s: %#x%s%s%s%s%s%s\n",
		 when, stat,
		 stat & SIA9177_INT_POR ? " POR" : "",
		 stat & SIA9177_INT_NOCLK ? " NOCLK" : "",
		 stat & SIA9177_INT_OTP ? " OTP" : "",
		 stat & SIA9177_INT_UVP ? " UVP" : "",
		 stat & SIA9177_INT_OCP ? " OCP" : "",
		 stat & SIA9177_INT_TDMERR ? " TDMERR" : "");
}

static void sia9177_bclk_disable(struct sia9177_priv *priv)
{
	if (!priv->bclk_on)
		return;

	clk_disable_unprepare(priv->bclk);
	priv->bclk_on = false;
}

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

	ret = regmap_multi_reg_write(priv->regmap, priv->tuning[priv->scene].init,
				     priv->tuning[priv->scene].init_len);
	if (ret)
		goto err_reset;

	ret = regmap_read(priv->regmap, SIA9177_REG_STATE, &val);
	if (ret)
		goto err_reset;
	if ((val & SIA9177_STATE_MASK) != SIA9177_STATE_STANDBY)
		dev_warn(component->dev, "not in standby after init: %#x\n",
			 val);

	/* clear whatever interrupt state reset left behind */
	regmap_write(priv->regmap, SIA9177_REG_INT_CLEAR, 0xffff);

	return 0;

	/*
	 * The core skips .shutdown for a DAI whose .startup failed, so this
	 * is the only thing that can put the chip back in powerdown.
	 */
err_reset:
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	return ret;
}

static int sia9177_start(struct snd_soc_component *component)
{
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int val;
	int ret;

	ret = regmap_multi_reg_write(priv->regmap, priv->tuning[priv->scene].start,
				     priv->tuning[priv->scene].start_len);
	if (ret)
		return ret;

	/*
	 * Watch the state nibble rather than reading it once: the chip climbs
	 * 2 -> 4 -> 5 over some milliseconds from here, and a single read
	 * catches it mid-ramp and calls a success a failure. If it really does
	 * not arrive, INT_STAT says why - TDMERR for a geometry the chip
	 * rejects, NOCLK for a bit clock that never came - and a nibble still
	 * at 2 means the startup writes did not take at all.
	 */
	ret = regmap_read_poll_timeout(priv->regmap, SIA9177_REG_STATE, val,
				       (val & SIA9177_STATE_MASK) ==
					       SIA9177_STATE_RUNNING,
				       SIA9177_RUN_POLL_US,
				       SIA9177_RUN_TIMEOUT_US);
	if (ret) {
		unsigned int stat;

		dev_warn(component->dev, "failed to start: state %#x (%d)\n",
			 val, ret);
		if (!regmap_read(priv->regmap, SIA9177_REG_INT_STAT, &stat) &&
		    stat)
			sia9177_report_faults(component, "at start", stat);
	}

	priv->running = true;

	return 0;
}

static int sia9177_power_up(struct snd_soc_component *component)
{
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	/*
	 * On AudioReach nothing else turns the MI2S bit clock on - the DSP
	 * paces the stream data without it and every SoC-side counter stays
	 * clean, so its absence is invisible from the host. The amp is the
	 * only thing that notices.
	 */
	if (priv->bclk) {
		ret = clk_set_rate(priv->bclk, priv->bclk_rate);
		if (ret) {
			dev_err(component->dev,
				"cannot set bit clock to %lu Hz: %d\n",
				priv->bclk_rate, ret);
			return ret;
		}

		ret = clk_prepare_enable(priv->bclk);
		if (ret) {
			dev_err(component->dev,
				"cannot enable bit clock: %d\n", ret);
			return ret;
		}
		priv->bclk_on = true;
	}

	priv->powered = true;

	/*
	 * An amplifier switched off is left in the standby the init writes put
	 * it in. It keeps its bit-clock vote: the interface is shared, and the
	 * part that is playing needs the clock either way.
	 */
	if (!priv->enabled)
		return 0;

	ret = sia9177_start(component);
	if (ret) {
		priv->powered = false;
		sia9177_bclk_disable(priv);
	}

	return ret;
}

static void sia9177_stop(struct snd_soc_component *component)
{
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int val;

	priv->running = false;

	regmap_update_bits(priv->regmap, SIA9177_REG_CTRL, BIT(2), 0);
	usleep_range(1000, 1500);
	regmap_update_bits(priv->regmap, SIA9177_REG_CTRL, BIT(0), BIT(0));
	usleep_range(1000, 1500);

	if (!regmap_read(priv->regmap, SIA9177_REG_STATE, &val) &&
	    (val & SIA9177_STATE_MASK) != SIA9177_STATE_STANDBY)
		dev_warn(component->dev, "failed to stop: state %#x\n", val);

	/*
	 * Last chance to see what the stream latched: .shutdown asserts reset
	 * straight after this, which clears the register. Nothing mainline
	 * consumes the IV sense the vendor's excursion protection runs on, so
	 * the sticky over-temperature/over-current/under-voltage bits are the
	 * only telemetry there is that playback was overdriving the parts.
	 */
	if (!regmap_read(priv->regmap, SIA9177_REG_INT_STAT, &val) && val) {
		sia9177_report_faults(component, "during playback", val);
		regmap_write(priv->regmap, SIA9177_REG_INT_CLEAR, 0xffff);
	}
}

static void sia9177_power_down(struct snd_soc_component *component)
{
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);

	sia9177_stop(component);
	priv->powered = false;
	sia9177_bclk_disable(priv);
}

/*
 * Re-apply the tuning when the scene changes under a running stream. The bit
 * clock is deliberately left running: the DSP is still feeding the interface,
 * the other amplifier is still voting for the clock, and the chip latched its
 * clock configuration when BCLK first appeared - taking it away here would
 * make the return a second first-appearance on a chip that is not in reset.
 *
 * That does put the init writes on a live bus, which is the ordering
 * sia9177_configure() exists to avoid. It is not the same case - the chip is
 * past the latch and only its tuning is being replaced - but it is not the
 * vendor's case either: the vendor driver has this path and compiles it out
 * on this platform, where scenes change by tearing the device down instead.
 * A chip that does not come back says so, with the state it stuck at.
 */
static int sia9177_apply(struct snd_soc_component *component)
{
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	ret = regmap_multi_reg_write(priv->regmap, priv->tuning[priv->scene].init,
				     priv->tuning[priv->scene].init_len);
	if (ret)
		return ret;

	return sia9177_start(component);
}

static int sia9177_rescene(struct snd_soc_component *component)
{
	sia9177_stop(component);

	return sia9177_apply(component);
}

static const char * const sia9177_scene_text[] = { "Speaker", "Receiver" };

static SOC_ENUM_SINGLE_EXT_DECL(sia9177_scene_enum, sia9177_scene_text);

static int sia9177_scene_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = priv->scene;

	return 0;
}

static int sia9177_scene_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int scene = ucontrol->value.enumerated.item[0];
	int ret = 0;

	if (scene >= SIA9177_SCENE_COUNT)
		return -EINVAL;

	mutex_lock(&priv->lock);

	if (scene != priv->scene) {
		priv->scene = scene;
		ret = priv->running ? sia9177_rescene(component) : 0;
		if (!ret)
			ret = 1;
	}

	mutex_unlock(&priv->lock);

	return ret;
}

static int sia9177_switch_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = priv->enabled;

	return 0;
}

/*
 * Whether this amplifier takes part in the stream at all. Both do for
 * ordinary playback; the earpiece is the one case that wants a single part,
 * and the vendor builds it the same way - its handset path enables channel 0
 * alone and leaves channel 1 at the muted default, so the top transducer is
 * the only one sounding.
 */
static int sia9177_switch_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	bool enable = ucontrol->value.integer.value[0];
	int ret = 0;

	mutex_lock(&priv->lock);

	if (enable != priv->enabled) {
		priv->enabled = enable;
		/*
		 * Off mid-stream is the shutdown sequence without the clock
		 * teardown; on is the whole tuning again, because the scene
		 * may have moved while this part sat out.
		 */
		if (priv->powered) {
			if (enable)
				ret = sia9177_apply(component);
			else
				sia9177_stop(component);
		}
		if (!ret)
			ret = 1;
	}

	mutex_unlock(&priv->lock);

	return ret;
}

static const struct snd_kcontrol_new sia9177_controls[] = {
	SOC_ENUM_EXT("Scene", sia9177_scene_enum,
		     sia9177_scene_get, sia9177_scene_put),
	SOC_SINGLE_BOOL_EXT("Switch", 0,
			    sia9177_switch_get, sia9177_switch_put),
};

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
	struct sia9177_priv *priv =
		snd_soc_component_get_drvdata(dai->component);
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	mutex_lock(&priv->lock);
	ret = sia9177_configure(dai->component);
	mutex_unlock(&priv->lock);

	return ret;
}

static void sia9177_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct sia9177_priv *priv =
		snd_soc_component_get_drvdata(dai->component);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	mutex_lock(&priv->lock);
	priv->running = false;
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	mutex_unlock(&priv->lock);
}

static int sia9177_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;
	struct sia9177_priv *priv = snd_soc_component_get_drvdata(component);
	int ret = 0;

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	mutex_lock(&priv->lock);
	if (mute)
		sia9177_power_down(component);
	else
		ret = sia9177_power_up(component);
	mutex_unlock(&priv->lock);

	return ret;
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
	.controls = sia9177_controls,
	.num_controls = ARRAY_SIZE(sia9177_controls),
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

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;

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
	priv->tuning = channel ? sia9177_tuning_right : sia9177_tuning_left;
	priv->scene = SIA9177_SCENE_SPEAKER;
	priv->enabled = true;

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
