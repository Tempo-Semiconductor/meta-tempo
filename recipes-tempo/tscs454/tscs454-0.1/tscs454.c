/*
 * This driver supports the three bi-directional serial interfaces on the
 * tscs454 CODEC.
 * The serial formats supported on each interface are as follows:
 *
 * 1. I2S / TDM
 * 2. I2S
 * 3. I2S
 *
 * There is also an asynchronous sample rate converter that is available to
 * one of the tree interfaces. This can support one interface as ansynchronous
 * slave when the others are configured as masters or it can support one
 * interface operating at a different sample rate than the other two.
 */
#include <linux/version.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/firmware.h>

#include "tscs454.h"

static const unsigned int PLL_48K_RATE = (48000 * 256);
static const unsigned int PLL_44_1K_RATE = (44100 * 256);
#define PLL_48K_FS_GCF 8000
#define PLL_44_1K_FS_GCF 11025
#define ISRC_FS_48K 48000
#define ISRC_FS_44_1K 44100

#define COEFF_SIZE 3
#define BIQUAD_COEFF_COUNT 5
#define BIQUAD_SIZE (COEFF_SIZE * BIQUAD_COEFF_COUNT)

#define COEFF_RAM_MAX_ADDR 0xcd
#define COEFF_RAM_COEFF_COUNT (COEFF_RAM_MAX_ADDR + 1)
#define COEFF_RAM_SIZE (COEFF_SIZE * COEFF_RAM_COEFF_COUNT)

enum {
	TSCS454_DAI1_ID,
	TSCS454_DAI2_ID,
	TSCS454_DAI3_ID,
	TSCS454_DAI_COUNT,
	TSCS454_DAI_NONE,
};

/*********
 * Types *
 *********/

struct pll {
	unsigned int users;
	struct mutex lock;
};

static inline void pll_init(struct pll *pll) {
	pll->users = 0;
	mutex_init(&pll->lock);
}

struct internal_rate {
	unsigned int fs;
	struct pll *pll;
};

struct asrc {
	unsigned int available:1;
	unsigned int in:1;
	unsigned int user_id;
	struct mutex lock;
};

static inline void asrc_init(struct asrc *asrc, bool in)
{
	asrc->available = true;
	asrc->in = in;
	asrc->user_id = TSCS454_DAI_NONE;
	mutex_init(&asrc->lock);
}

struct aif {
	unsigned int id;
	unsigned int master:1;
	struct asrc *asrc_in;
	struct asrc *asrc_out;
	struct pll *pll;
};

static inline void aif_init(struct aif *aif, unsigned int id)
{
	aif->id = id;
}

struct coeff_ram {
	u8 cache[COEFF_RAM_SIZE];
	bool synced;
	struct mutex lock;
};

static inline void init_coeff_ram_cache(u8 *cache)
{
	const u8 norm_addrs[] = { 0x00, 0x05, 0x0a, 0x0f, 0x14, 0x19, 0x1f,
		0x20, 0x25, 0x2a, 0x2f, 0x34, 0x39, 0x3f, 0x40, 0x45, 0x4a,
		0x4f, 0x54, 0x59, 0x5f, 0x60, 0x65, 0x6a, 0x6f, 0x74, 0x79,
		0x7f, 0x80, 0x85, 0x8c, 0x91, 0x96, 0x97, 0x9c, 0xa3, 0xa8,
		0xad, 0xaf, 0xb0, 0xb5, 0xba, 0xbf, 0xc4, 0xc9, };
	int i;

	for (i = 0; i < ARRAY_SIZE(norm_addrs); i++)
		cache[((norm_addrs[i] + 1) * COEFF_SIZE) - 1] = 0x40;
}

static inline void coeff_ram_init(struct coeff_ram *ram)
{
	init_coeff_ram_cache(ram->cache);
	ram->synced = false;
	mutex_init(&ram->lock);
}

struct tscs454 {
	struct regmap *regmap;
	unsigned int internal_fs;
	struct asrc asrc_in;
	struct asrc asrc_out;
	struct aif dais[TSCS454_DAI_COUNT];
	unsigned int active_aifs;
	struct mutex active_aifs_lock;
	struct pll pll1;
	struct pll pll2;
	struct internal_rate internal_rate;

	struct coeff_ram dac_ram;
	struct coeff_ram spk_ram;
	struct coeff_ram sub_ram;

	struct clk *sysclk;
	int sysclk_src_id;
	unsigned int bclk_freq;
};

struct coeff_ram_ctl {
	unsigned int addr;
	struct soc_bytes_ext bytes_ext;
};

/**********
 * regmap *
 **********/
static const struct reg_sequence tscs454_patch[] = {
	{ R_TDMCTL0, FV_TDMMD_256 },
	{ R_PCMP2CTL0, FV_PCMFLENP_256 },
	{ R_PCMP3CTL0, FV_PCMFLENP_256 },
	{ VIRT_ADDR(0x0A, 0x13), 1 << 3 },
};

static bool tscs454_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case R_PLLSTAT:

	case R_SPKCRRDL:
	case R_SPKCRRDM:
	case R_SPKCRRDH:
	case R_SPKCRS:

	case R_DACCRRDL:
	case R_DACCRRDM:
	case R_DACCRRDH:
	case R_DACCRS:

	case R_SUBCRRDL:
	case R_SUBCRRDM:
	case R_SUBCRRDH:
	case R_SUBCRS:
		return true;
	default:
		return false;
	};
}

static bool tscs454_writable(struct device *dev, unsigned int reg)
{	
	switch (reg) {
	case R_SPKCRRDL:
	case R_SPKCRRDM:
	case R_SPKCRRDH:

	case R_DACCRRDL:
	case R_DACCRRDM:
	case R_DACCRRDH:

	case R_SUBCRRDL:
	case R_SUBCRRDM:
	case R_SUBCRRDH:
		return false;
	default:
		return true;
	};
}

static bool tscs454_readable(struct device *dev, unsigned int reg)
{	
	switch (reg) {
	case R_SPKCRWDL:
	case R_SPKCRWDM:
	case R_SPKCRWDH:

	case R_DACCRWDL:
	case R_DACCRWDM:
	case R_DACCRWDH:

	case R_SUBCRWDL:
	case R_SUBCRWDM:
	case R_SUBCRWDH:
		return false;
	default:
		return true;
	};
}

static bool tscs454_precious(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case R_SPKCRWDL:
	case R_SPKCRWDM:
	case R_SPKCRWDH:
	case R_SPKCRRDL:
	case R_SPKCRRDM:
	case R_SPKCRRDH:

	case R_DACCRWDL:
	case R_DACCRWDM:
	case R_DACCRWDH:
	case R_DACCRRDL:
	case R_DACCRRDM:
	case R_DACCRRDH:

	case R_SUBCRWDL:
	case R_SUBCRWDM:
	case R_SUBCRWDH:
	case R_SUBCRRDL:
	case R_SUBCRRDM:
	case R_SUBCRRDH:
		return true;
	default:
		return false;
	};
}

static const struct regmap_range_cfg tscs454_regmap_range_cfg = {
	.name = "Pages",
	.range_min = VIRT_BASE,
	.range_max = VIRT_ADDR(0xFE, 0x02),
	.selector_reg = R_PAGESEL,
	.selector_mask = 0xff,
	.selector_shift = 0,
	.window_start = 0, 
	.window_len = 0x100,
	.selector_shift = 0,
};

static struct regmap_config const tscs454_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tscs454_writable,
	.readable_reg = tscs454_readable,
	.volatile_reg = tscs454_volatile,
	.precious_reg = tscs454_precious,
	.ranges = &tscs454_regmap_range_cfg,
	.num_ranges = 1,
	.max_register = VIRT_ADDR(0xFE, 0x02),
	.cache_type = REGCACHE_RBTREE,
};
static inline int tscs454_data_init(struct tscs454 *tscs454,
		struct i2c_client *i2c)
{
	int i;
	int ret;

	tscs454->regmap = devm_regmap_init_i2c(i2c, &tscs454_regmap_cfg);
	if (IS_ERR(tscs454->regmap)) {
		ret = PTR_ERR(tscs454->regmap);
		return ret;
	}

	for (i = 0; i < TSCS454_DAI_COUNT; i++)
		aif_init(&tscs454->dais[i], i);

	asrc_init(&tscs454->asrc_in, true);
	asrc_init(&tscs454->asrc_out, false);
	mutex_init(&tscs454->active_aifs_lock);
	pll_init(&tscs454->pll1);
	pll_init(&tscs454->pll2);

	coeff_ram_init(&tscs454->dac_ram);
	coeff_ram_init(&tscs454->spk_ram);
	coeff_ram_init(&tscs454->sub_ram);

	return 0;
}

struct reg_setting {
	unsigned int addr;
	unsigned int val;
};

/*************************
 * Coefficient RAM Stuff *
 *************************/

static int coeff_ram_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	struct coeff_ram_ctl *ctl =
		(struct coeff_ram_ctl *)kcontrol->private_value;
	struct soc_bytes_ext *params = &ctl->bytes_ext;
	u8 *coeff_ram;
	struct mutex *coeff_ram_lock;

	if (strstr(kcontrol->id.name, "DAC")) {
		coeff_ram = tscs454->dac_ram.cache;
		coeff_ram_lock = &tscs454->dac_ram.lock;
	} else if (strstr(kcontrol->id.name, "Speaker")) {
		coeff_ram = tscs454->spk_ram.cache;
		coeff_ram_lock = &tscs454->spk_ram.lock;
	} else if (strstr(kcontrol->id.name, "Sub")) {
		coeff_ram = tscs454->sub_ram.cache;
		coeff_ram_lock = &tscs454->sub_ram.lock;
	} else {
		return -EINVAL;
	}

	mutex_lock(coeff_ram_lock);

	memcpy(ucontrol->value.bytes.data,
		&coeff_ram[ctl->addr * COEFF_SIZE], params->max);

	mutex_unlock(coeff_ram_lock);

	return 0;
}

#define DACCRSTAT_MAX_TRYS 10
static int write_coeff_ram(struct snd_soc_codec *codec, u8 *coeff_ram,
		unsigned int r_stat, unsigned int r_addr, unsigned int r_wr,
		unsigned int coeff_addr, unsigned int coeff_cnt)
{
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	int cnt;
	int trys;
	int ret;

	for (cnt = 0; cnt < coeff_cnt; cnt++, coeff_addr++) {

		for (trys = 0; trys < DACCRSTAT_MAX_TRYS; trys++) {
			ret = snd_soc_read(codec, r_stat);
			if (ret < 0) {
				dev_err(codec->dev,
					"Failed to read stat (%d)\n", ret);
				return ret;
			}
			if (!ret)
				break;
		}

		if (trys == DACCRSTAT_MAX_TRYS) {
			ret = -EIO;
			dev_err(codec->dev,
				"Coefficient write error (%d)\n", ret);
			return ret;
		}

		ret = regmap_write(tscs454->regmap, r_addr, coeff_addr);
		if (ret < 0) {
			dev_err(codec->dev,
				"Failed to write dac ram address (%d)\n", ret);
			return ret;
		}

		ret = regmap_bulk_write(tscs454->regmap, r_wr,
			&coeff_ram[coeff_addr * COEFF_SIZE],
			COEFF_SIZE);
		if (ret < 0) {
			dev_err(codec->dev,
				"Failed to write dac ram (%d)\n", ret);
			return ret;
		}
	}

	return 0;
}

static int coeff_ram_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	struct coeff_ram_ctl *ctl =
		(struct coeff_ram_ctl *)kcontrol->private_value;
	struct soc_bytes_ext *params = &ctl->bytes_ext;
	unsigned int coeff_cnt = params->max / COEFF_SIZE;
	u8 *coeff_ram;
	struct mutex *coeff_ram_lock;
	bool *coeff_ram_synced;
	unsigned int r_stat;
	unsigned int r_addr;
	unsigned int r_wr;
	int ret;

	if (strstr(kcontrol->id.name, "DAC")) {
		coeff_ram = tscs454->dac_ram.cache;
		coeff_ram_lock = &tscs454->dac_ram.lock;
		coeff_ram_synced = &tscs454->dac_ram.synced;
		r_stat = R_DACCRS;
		r_addr = R_DACCRADD;
		r_wr = R_DACCRWDL;
	} else if (strstr(kcontrol->id.name, "Speaker")) {
		coeff_ram = tscs454->spk_ram.cache;
		coeff_ram_lock = &tscs454->spk_ram.lock;
		coeff_ram_synced = &tscs454->spk_ram.synced;
		r_stat = R_SPKCRS;
		r_addr = R_SPKCRADD;
		r_wr = R_SPKCRWDL;
	} else if (strstr(kcontrol->id.name, "Sub")) {
		coeff_ram = tscs454->sub_ram.cache;
		coeff_ram_lock = &tscs454->sub_ram.lock;
		coeff_ram_synced = &tscs454->sub_ram.synced;
		r_stat = R_SUBCRS;
		r_addr = R_SUBCRADD;
		r_wr = R_SUBCRWDL;
	} else {
		return -EINVAL;
	}

	mutex_lock(coeff_ram_lock);

	*coeff_ram_synced = false;

	memcpy(&coeff_ram[ctl->addr * COEFF_SIZE],
		ucontrol->value.bytes.data, params->max);

	mutex_lock(&tscs454->pll1.lock);
	mutex_lock(&tscs454->pll2.lock);

	ret = snd_soc_read(codec, R_PLLSTAT);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to read PLL status (%d)\n",
				ret);
		goto exit;
	}
	if (ret) { /* PLLs locked */
		ret = write_coeff_ram(codec, coeff_ram,
			r_stat, r_addr, r_wr,
			ctl->addr, coeff_cnt);
		if (ret < 0) {
			dev_err(codec->dev,
				"Failed to flush coeff ram cache (%d)\n", ret);
			goto exit;
		}
		*coeff_ram_synced = true;
	}

	ret = 0;
exit:
	mutex_unlock(&tscs454->pll2.lock);
	mutex_unlock(&tscs454->pll1.lock);
	mutex_unlock(coeff_ram_lock);

	return ret;
}

static int coeff_ram_sync(struct snd_soc_codec *codec,
		struct tscs454 *tscs454)
{
	int ret;

	mutex_lock(&tscs454->dac_ram.lock);
	if (!tscs454->dac_ram.synced) {
		ret = write_coeff_ram(codec, tscs454->dac_ram.cache,
				R_DACCRS, R_DACCRADD, R_DACCRWDL,
				0x00, COEFF_RAM_COEFF_COUNT);
		if (ret < 0) {
			mutex_unlock(&tscs454->dac_ram.lock);
			return ret;
		}
	}
	mutex_unlock(&tscs454->dac_ram.lock);

	mutex_lock(&tscs454->spk_ram.lock);
	if (!tscs454->spk_ram.synced) {
		ret = write_coeff_ram(codec, tscs454->spk_ram.cache,
				R_SPKCRS, R_SPKCRADD, R_SPKCRWDL,
				0x00, COEFF_RAM_COEFF_COUNT);
		if (ret < 0) {
			mutex_unlock(&tscs454->spk_ram.lock);
			return ret;
		}
	}
	mutex_unlock(&tscs454->spk_ram.lock);

	mutex_lock(&tscs454->sub_ram.lock);
	if (!tscs454->sub_ram.synced) {
		ret = write_coeff_ram(codec, tscs454->sub_ram.cache,
				R_SUBCRS, R_SUBCRADD, R_SUBCRWDL,
				0x00, COEFF_RAM_COEFF_COUNT);
		if (ret < 0) {
			mutex_unlock(&tscs454->sub_ram.lock);
			return ret;
		}
	}
	mutex_unlock(&tscs454->sub_ram.lock);

	return 0;
}

/***********************
 * Internal Rate Stuff *
 ***********************/

/**************
 * ASRC Stuff *
 **************/

/*
 * returns true if asrc was reserved
 */
static inline bool reserve_asrc(struct asrc *asrc, unsigned int user_id)
{
	bool ret = false;

	mutex_lock(&asrc->lock);
	if (asrc->available) {
		asrc->available = false;
		asrc->user_id = user_id;
		ret = true;
	}
	mutex_unlock(&asrc->lock);

	return ret;
}

static inline void free_asrc(struct asrc *asrc)
{
	asrc->available = true;
}

static int configure_asrc(struct snd_soc_codec *codec, struct asrc *asrc)
{
	unsigned int user_id = asrc->user_id;
	unsigned int val;
	int ret;

	if (asrc->in) {
		switch(user_id) {
		case TSCS454_DAI1_ID:
			val = FV_ASRCIMUX_I2S1;
			break;
		case TSCS454_DAI2_ID:
			val = FV_ASRCIMUX_I2S2;
			break;
		case TSCS454_DAI3_ID:
			val = FV_ASRCIMUX_I2S3;
			break;
		default:
			ret = -EINVAL;
			dev_err(codec->dev,
					"Unknown interface DAI%u (%d)\n",
					user_id, ret);
			return ret;
		}
		ret = snd_soc_update_bits(codec,
				R_AUDIOMUX1, FM_AUDIOMUX1_ASRCIMUX, val);
		if (ret < 0) {
			dev_err(codec->dev, "Failed to set ASRC in mux (%d)\n",
					ret);
			return ret;
		}
	} else {
		switch(user_id) {
		case TSCS454_DAI1_ID:
			val = FV_ASRCOMUX_I2S1;
			break;
		case TSCS454_DAI2_ID:
			val = FV_ASRCOMUX_I2S2;
			break;
		case TSCS454_DAI3_ID:
			val = FV_ASRCOMUX_I2S3;
			break;
		default:
			ret = -EINVAL;
			dev_err(codec->dev,
					"Unknown interface DAI%u (%d)\n",
					user_id, ret);
			return ret;
		}
		ret = snd_soc_update_bits(codec,
				R_AUDIOMUX2, FM_AUDIOMUX2_ASRCOMUX, val);
		if (ret < 0) {
			dev_err(codec->dev, "Failed to set ASRC out mux (%d)\n",
					ret);
			return ret;
		}
	}

	return 0;
}

static int asrc_connected(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);

	if (strcmp(source->name, "ASRC In") == 0)
		return !tscs454->asrc_in.available;
	else
		return !tscs454->asrc_out.available;
}

/*************
 * PLL Stuff *
 *************/

#define PLL_REG_SETTINGS_COUNT 11
struct pll_ctl {
	int freq_in;
	struct reg_setting settings[PLL_REG_SETTINGS_COUNT];
};

#define PLL_CTL(f,t,c1,r1,o1,f1l,f1h,c2,r2,o2,f2l,f2h)		\
	{							\
		.freq_in = f,					\
		.settings = {					\
			{ R_PLL1CTL  , c1 },			\
			{ R_PLL1RDIV , r1 },			\
			{ R_PLL1ODIV , o1 },			\
			{ R_PLL1FDIVL, f1l },			\
			{ R_PLL1FDIVH, f1h },			\
			{ R_PLL2CTL  , c2 },			\
			{ R_PLL2RDIV , r2 },			\
			{ R_PLL2ODIV , o2 },			\
			{ R_PLL2FDIVL, f2l },			\
			{ R_PLL2FDIVH, f2h },			\
			{ R_TIMEBASE , t },			\
		},						\
	}				

static const struct pll_ctl pll_ctls[] = {
	PLL_CTL(1411200, 0x05,
		0xB9, 0x07, 0x02, 0xC3, 0x04,
		0x5A, 0x02, 0x03, 0xE0, 0x01),
	PLL_CTL(1536000, 0x05,
		0x5A, 0x02, 0x03, 0xE0, 0x01,
		0x5A, 0x02, 0x03, 0xB9, 0x01),
	PLL_CTL(2822400, 0x0A,
		0x63, 0x07, 0x04, 0xC3, 0x04,
		0x62, 0x07, 0x03, 0x48, 0x03),
	PLL_CTL(3072000, 0x0B,
		0x62, 0x07, 0x03, 0x48, 0x03,
		0x5A, 0x04, 0x03, 0xB9, 0x01),
	PLL_CTL(5644800, 0x15,
		0x63, 0x0E, 0x04, 0xC3, 0x04,
		0x5A, 0x08, 0x03, 0xE0, 0x01),
	PLL_CTL(6144000, 0x17,
		0x5A, 0x08, 0x03, 0xE0, 0x01,
		0x5A, 0x08, 0x03, 0xB9, 0x01),
	PLL_CTL(12000000, 0x2E,
		0x5B, 0x19, 0x03, 0x00, 0x03,
		0x6A, 0x19, 0x05, 0x98, 0x04),
	PLL_CTL(19200000, 0x4A,
		0x53, 0x14, 0x03, 0x80, 0x01,
		0x5A, 0x19, 0x03, 0xB9, 0x01),
	PLL_CTL(22000000, 0x55,
		0x6A, 0x37, 0x05, 0x00, 0x06,
		0x62, 0x26, 0x03, 0x49, 0x02),
	PLL_CTL(22579200, 0x57,
		0x62, 0x31, 0x03, 0x20, 0x03,
		0x53, 0x1D, 0x03, 0xB3, 0x01),
	PLL_CTL(24000000, 0x5D,
		0x53, 0x19, 0x03, 0x80, 0x01,
		0x5B, 0x19, 0x05, 0x4C, 0x02),
	PLL_CTL(24576000, 0x5F,
		0x53, 0x1D, 0x03, 0xB3, 0x01,
		0x62, 0x40, 0x03, 0x72, 0x03),
	PLL_CTL(27000000, 0x68,
		0x62, 0x4B, 0x03, 0x00, 0x04,
		0x6A, 0x7D, 0x03, 0x20, 0x06),
	PLL_CTL(36000000, 0x8C,
		0x5B, 0x4B, 0x03, 0x00, 0x03,
		0x6A, 0x7D, 0x03, 0x98, 0x04),
	PLL_CTL(11289600, 0x2B,
		0x6A, 0x31, 0x03, 0x40, 0x06,
		0x5A, 0x12, 0x03, 0x1C, 0x02),
	PLL_CTL(26000000, 0x65,
		0x63, 0x41, 0x05, 0x00, 0x06,
		0x5A, 0x26, 0x03, 0xEF, 0x01),
	PLL_CTL(12288000, 0x2F,
		0x5A, 0x12, 0x03, 0x1C, 0x02,
		0x62, 0x20, 0x03, 0x72, 0x03),
	PLL_CTL(40000000, 0x9B,
		0xA2, 0x7D, 0x03, 0x80, 0x04,
		0x63, 0x7D, 0x05, 0xE4, 0x06),
	PLL_CTL(512000, 0x01,
		0x62, 0x01, 0x03, 0xD0, 0x02,
		0x5B, 0x01, 0x04, 0x72, 0x03),
	PLL_CTL(705600, 0x02,
		0x62, 0x02, 0x03, 0x15, 0x04,
		0x62, 0x01, 0x04, 0x80, 0x02),
	PLL_CTL(1024000, 0x03,
		0x62, 0x02, 0x03, 0xD0, 0x02,
		0x5B, 0x02, 0x04, 0x72, 0x03),
	PLL_CTL(2048000, 0x07,
		0x62, 0x04, 0x03, 0xD0, 0x02,
		0x5B, 0x04, 0x04, 0x72, 0x03),
	PLL_CTL(2400000, 0x08,
		0x62, 0x05, 0x03, 0x00, 0x03,
		0x63, 0x05, 0x05, 0x98, 0x04),
};

static const struct pll_ctl *get_pll_ctl(unsigned long freq_in)
{
	int i;
	struct pll_ctl const *pll_ctl = NULL;

	for (i = 0; i < ARRAY_SIZE(pll_ctls); ++i)
		if (pll_ctls[i].freq_in == freq_in) {
			pll_ctl = &pll_ctls[i];
			break;
		}

	return pll_ctl;
}

static int set_sysclk(struct snd_soc_codec *codec)
{
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	struct pll_ctl const *pll_ctl;
	unsigned long freq;
	int i;
	int ret;

	if (tscs454->sysclk_src_id != FV_PLLISEL_BCLK)
		freq = clk_get_rate(tscs454->sysclk);
	else
		freq = tscs454->bclk_freq;
	pll_ctl = get_pll_ctl(freq);
	if (!pll_ctl) {
		ret = -EINVAL;
		dev_err(codec->dev,
				"Invalid PLL input %lu (%d)\n", freq, ret);
		return ret;
	}

	ret = snd_soc_update_bits(codec, R_PLLCTL, FM_PLLCTL_PLLISEL,
			tscs454->sysclk_src_id);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set PLL input (%d)\n", ret);
		return ret;
	}

	for (i = 0; i < PLL_REG_SETTINGS_COUNT; ++i) {
		ret = snd_soc_write(codec, pll_ctl->settings[i].addr, 
			pll_ctl->settings[i].val);
		if (ret < 0) {
			dev_err(codec->dev, "Failed to set pll setting (%d)\n",
					ret);
			return ret;
		}
	}

	return 0;
}

static inline void reserve_pll(struct pll *pll)
{
	mutex_lock(&pll->lock);
	pll->users++;
	mutex_unlock(&pll->lock);
}

static inline void free_pll(struct pll *pll)
{
	mutex_lock(&pll->lock);
	pll->users--;
	mutex_unlock(&pll->lock);
}

static int pll_connected(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);

	if (strcmp(source->name, "PLL 1 Clock") == 0) {
		return tscs454->pll1.users;
	} else {
		return tscs454->pll2.users;
	}
}

/*
 * PLL must be enabled after power up and must be disabled before power down
 * for proper clock switching.
 */
static int pll_power_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	bool enable = event == SND_SOC_DAPM_POST_PMU;
	bool pll1 = true;
	unsigned int msk;
	unsigned int val;
	int ret;

	if (strstr(w->name, "PLL 2"))
		pll1 = false;
	
	msk = pll1 ? FM_PLLCTL_PLL1CLKEN : FM_PLLCTL_PLL2CLKEN;

	if (enable)
		val = pll1 ? FV_PLL1CLKEN_ENABLE : FV_PLL2CLKEN_ENABLE;
	else
		val = pll1 ? FV_PLL1CLKEN_DISABLE : FV_PLL2CLKEN_DISABLE;

	ret = snd_soc_update_bits(codec, R_PLLCTL, msk, val);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to %s PLL %d  (%d)\n",
				enable ? "enable" : "disable",
				pll1 ? 1 : 2,
				ret);
		return ret;
	}

	if (enable) {
		msleep(20);
		ret = coeff_ram_sync(codec, tscs454);
		if (ret < 0)
			dev_err(codec->dev, "Failed to sync coeff ram (%d)\n",
					ret);
	}

	return 0;
}

/******************************
 * Audio Interface Operations *
 ******************************/

static int aif_set_master(struct snd_soc_codec *codec,
		unsigned int aif_id, bool master)
{
	unsigned int reg;
	unsigned int mask;
	unsigned int val;
	int ret;

	switch (aif_id) {
	case TSCS454_DAI1_ID:
		reg = R_I2SP1CTL;
		break;
	case TSCS454_DAI2_ID:
		reg = R_I2SP2CTL;
		break;
	case TSCS454_DAI3_ID:
		reg = R_I2SP3CTL;
		break;
	default:
		ret = -ENODEV;
		dev_err(codec->dev, "Unknown DAI %d (%d)\n", aif_id, ret);
		return ret;
	}
	mask = FM_I2SPCTL_PORTMS;
	val = master ? FV_PORTMS_MASTER : FV_PORTMS_SLAVE;

	ret = snd_soc_update_bits(codec, reg, mask, val);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set DAI %d to %s (%d)\n",
			aif_id, master ? "master" : "slave", ret);
		return ret;
	}

	return 0;
}

static int aif_prepare(struct snd_soc_codec *codec, struct aif *aif)
{
	int ret;

	/* Step 8 */
	ret = aif_set_master(codec, aif->id, aif->master);
	if (ret < 0)
		return ret;

	return 0;
}

static int aif_free(struct snd_soc_codec *codec,
		struct aif *aif, bool playback)
{
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);

	/* Step 1 */
	aif_set_master(codec, aif->id, false);

	if (aif->master) {
		/* Slaves hold onto the asrc */
		if (playback) {
			if (aif->asrc_in) {
				free_asrc(aif->asrc_in);
				aif->asrc_in = NULL;
			}
		} else {
			if (aif->asrc_out) {
				free_asrc(aif->asrc_out);
				aif->asrc_out = NULL;
			}
		}
	}

	free_pll(aif->pll);

	mutex_lock(&tscs454->active_aifs_lock);
	tscs454->active_aifs--;
	if (!tscs454->active_aifs)
		free_pll(tscs454->internal_rate.pll);
	mutex_unlock(&tscs454->active_aifs_lock);

	return 0;
}

/*****************
 * ALSA Controls *
 *****************/

/* R_SCLKCTL PG 0 ADDR 0x18 */
static char const * const modular_rate_txt[] = {
	"Reserved", "Half", "Full", "Auto",};

static struct soc_enum const adc_modular_rate_enum =
	SOC_ENUM_SINGLE(R_SCLKCTL, FB_SCLKCTL_ASDM,
			ARRAY_SIZE(modular_rate_txt), modular_rate_txt);

static struct soc_enum const dac_modular_rate_enum =
	SOC_ENUM_SINGLE(R_SCLKCTL, FB_SCLKCTL_DSDM,
			ARRAY_SIZE(modular_rate_txt), modular_rate_txt);

/* R_I2SIDCTL PG 0 ADDR 0x38 */
static char const * const data_ctrl_txt[] = {
	"L/R", "L/L", "R/R", "R/L"};

static struct soc_enum const data_in_ctrl_enums[] = {
	SOC_ENUM_SINGLE(R_I2SIDCTL, FB_I2SIDCTL_I2SI1DCTL,
			ARRAY_SIZE(data_ctrl_txt), data_ctrl_txt),
	SOC_ENUM_SINGLE(R_I2SIDCTL, FB_I2SIDCTL_I2SI2DCTL,
			ARRAY_SIZE(data_ctrl_txt), data_ctrl_txt),
	SOC_ENUM_SINGLE(R_I2SIDCTL, FB_I2SIDCTL_I2SI3DCTL,
			ARRAY_SIZE(data_ctrl_txt), data_ctrl_txt),
};

/* R_I2SODCTL PG 0 ADDR 0x39 */
static struct soc_enum const data_out_ctrl_enums[] = {
	SOC_ENUM_SINGLE(R_I2SODCTL, FB_I2SODCTL_I2SO1DCTL,
			ARRAY_SIZE(data_ctrl_txt), data_ctrl_txt),
	SOC_ENUM_SINGLE(R_I2SODCTL, FB_I2SODCTL_I2SO2DCTL,
			ARRAY_SIZE(data_ctrl_txt), data_ctrl_txt),
	SOC_ENUM_SINGLE(R_I2SODCTL, FB_I2SODCTL_I2SO3DCTL,
			ARRAY_SIZE(data_ctrl_txt), data_ctrl_txt),
};

/* R_AUDIOMUX1  PG 0 ADDR 0x3A */
static char const * const dai_mux_txt[] = {
		"CH 0_1", "CH 2_3", "CH 4_5", "ADC/DMic 1",
		"DMic 2", "ClassD", "DAC", "Sub"};

static struct soc_enum const dai2_mux_enum =
		SOC_ENUM_SINGLE(R_AUDIOMUX1, FB_AUDIOMUX1_I2S2MUX,
				ARRAY_SIZE(dai_mux_txt), dai_mux_txt);

static struct snd_kcontrol_new const dai2_mux_dapm_enum =
		SOC_DAPM_ENUM("DAI 2 Mux",  dai2_mux_enum);

static struct soc_enum const dai1_mux_enum = 
		SOC_ENUM_SINGLE(R_AUDIOMUX1, FB_AUDIOMUX1_I2S1MUX, 
				ARRAY_SIZE(dai_mux_txt), dai_mux_txt);

static struct snd_kcontrol_new const dai1_mux_dapm_enum =
		SOC_DAPM_ENUM("DAI 1 Mux",  dai1_mux_enum);

/* R_AUDIOMUX2 PG 0 ADDR 0x3B */
static struct soc_enum const dac_mux_enum =
		SOC_ENUM_SINGLE(R_AUDIOMUX2, FB_AUDIOMUX2_DACMUX,
				ARRAY_SIZE(dai_mux_txt), dai_mux_txt);

static struct snd_kcontrol_new const dac_mux_dapm_enum = 
		SOC_DAPM_ENUM("DAC Mux",  dac_mux_enum);

static struct soc_enum const dai3_mux_enum =
		SOC_ENUM_SINGLE(R_AUDIOMUX2, FB_AUDIOMUX2_I2S3MUX,
				ARRAY_SIZE(dai_mux_txt), dai_mux_txt);

static struct snd_kcontrol_new const dai3_mux_dapm_enum =
		SOC_DAPM_ENUM("DAI 3 Mux",  dai3_mux_enum);

/* R_AUDIOMUX3 PG 0 ADDR 0x3C */
static char const * const sub_mux_txt[] = {
		"CH 0", "CH 1", "CH 0 + 1",
		"CH 2", "CH 3", "CH 2 + 3",
		"CH 4", "CH 5", "CH 4 + 5",
		"ADC/DMic 1 Left", "ADC/DMic 1 Right",
		"ADC/DMic 1 Left Plus Right",
		"DMic 2 Left", "DMic 2 Right", "DMic 2 Left Plus Right",
		"ClassD Left", "ClassD Right", "ClassD Left Plus Right"};

static struct soc_enum const sub_mux_enum =
		SOC_ENUM_SINGLE(R_AUDIOMUX3, FB_AUDIOMUX3_SUBMUX,
				ARRAY_SIZE(sub_mux_txt), sub_mux_txt);

static struct snd_kcontrol_new const sub_mux_dapm_enum = 
		SOC_DAPM_ENUM("Sub Mux", sub_mux_enum);

static struct soc_enum const classd_mux_enum =
		SOC_ENUM_SINGLE(R_AUDIOMUX3, FB_AUDIOMUX3_CLSSDMUX,
				ARRAY_SIZE(dai_mux_txt), dai_mux_txt);

static struct snd_kcontrol_new const classd_mux_dapm_enum =
		SOC_DAPM_ENUM("ClassD Mux",  classd_mux_enum);

/* R_HSDCTL1 PG 0 ADDR 0x01 */
static char const * const jack_type_txt[] = {
		"3 Terminal", "4 Terminal"};
static struct soc_enum const hp_jack_type_enum =
		SOC_ENUM_SINGLE(R_HSDCTL1, FB_HSDCTL1_HPJKTYPE,
				ARRAY_SIZE(jack_type_txt), jack_type_txt);

static char const * const hs_det_pol_txt[] = {
		"Rising", "Falling"};
static struct soc_enum const hs_det_pol_enum =
		SOC_ENUM_SINGLE(R_HSDCTL1, FB_HSDCTL1_HSDETPOL,
				ARRAY_SIZE(hs_det_pol_txt), hs_det_pol_txt);

/*************************** 
 * R_CH0AIC PG 1 ADDR 0x06 *
 ***************************/

/* Text */
static char const * const in_bst_mux_txt[] = {
		"Input 1","Input 2","Input 3","D2S"};
/* CH0 Input Boost Mux */
static struct soc_enum const in_bst_mux_ch0_enum =
		SOC_ENUM_SINGLE(R_CH0AIC, FB_CH0AIC_INSELL,
				ARRAY_SIZE(in_bst_mux_txt),
				in_bst_mux_txt);
static struct snd_kcontrol_new const in_bst_mux_ch0_dapm_enum =
		SOC_DAPM_ENUM("Input Boost Channel 0 Enum",
				in_bst_mux_ch0_enum);
/* Text */
static char const * const in_bst_txt[] = {
		"0 dB", "10 dB", "20 dB", "30 dB"};
/* CH0 Input Boost */
static struct soc_enum const in_bst_ch0_enum =
		SOC_ENUM_SINGLE(R_CH0AIC, FB_CH0AIC_MICBST0,
				ARRAY_SIZE(in_bst_txt), in_bst_txt);
/* Text */
static char const * const adc_mux_txt[] = {
		"Input 1 Boost Bypass", "Input 2 Boost Bypass",
		"Input 3 Boost Bypass", "Input Boost"};
/* CH0 ADC Mux */
static struct soc_enum const adc_mux_ch0_enum =
		SOC_ENUM_SINGLE(R_CH0AIC, FB_CH0AIC_LADCIN,
				ARRAY_SIZE(adc_mux_txt), adc_mux_txt);
static struct snd_kcontrol_new const adc_mux_ch0_dapm_enum =
		SOC_DAPM_ENUM("ADC Channel 0 Enum", adc_mux_ch0_enum);
/* Text */
static char const * const in_proc_mux_txt[] = {
		"ADC", "DMic"};
/* CH0 Input Processor Mux */
static struct soc_enum const in_proc_ch0_enum =
		SOC_ENUM_SINGLE(R_CH0AIC, FB_CH0AIC_IPCH0S,
				ARRAY_SIZE(in_proc_mux_txt), in_proc_mux_txt);
static struct snd_kcontrol_new const in_proc_mux_ch0_dapm_enum =
		SOC_DAPM_ENUM("Input Processor Channel 0 Enum",
				in_proc_ch0_enum);

/*************************** 
 * R_CH1AIC PG 1 ADDR 0x07 *
 ***************************/

/* CH1 Input Boost Mux */
static struct soc_enum const in_bst_mux_ch1_enum =
		SOC_ENUM_SINGLE(R_CH1AIC, FB_CH1AIC_INSELR,
				ARRAY_SIZE(in_bst_mux_txt),
				in_bst_mux_txt);
static struct snd_kcontrol_new const in_bst_mux_ch1_dapm_enum =
		SOC_DAPM_ENUM("Input Boost Channel 1 Enum",
				in_bst_mux_ch1_enum);
/* CH1 Input Boost */
static struct soc_enum const in_bst_ch1_enum =
		SOC_ENUM_SINGLE(R_CH1AIC, FB_CH1AIC_MICBST1,
				ARRAY_SIZE(in_bst_txt), in_bst_txt);
/* CH1 ADC Mux */
static struct soc_enum const adc_mux_ch1_enum =
		SOC_ENUM_SINGLE(R_CH1AIC, FB_CH1AIC_RADCIN,
				ARRAY_SIZE(adc_mux_txt), adc_mux_txt);
static struct snd_kcontrol_new const adc_mux_ch1_dapm_enum =
		SOC_DAPM_ENUM("ADC Channel 1 Enum", adc_mux_ch1_enum);
/* CH1 Input Processor Mux */
static struct soc_enum const in_proc_ch1_enum =
		SOC_ENUM_SINGLE(R_CH1AIC, FB_CH1AIC_IPCH1S,
				ARRAY_SIZE(in_proc_mux_txt), in_proc_mux_txt);
static struct snd_kcontrol_new const in_proc_mux_ch1_dapm_enum =
		SOC_DAPM_ENUM("Input Processor Channel 1 Enum",
				in_proc_ch1_enum);

/***************************
 * R_CH2AIC PG 1 ADDR 0x08 *
 ***************************/

/* Ch2 Input Boost */
static struct soc_enum const in_bst_ch2_enum =
		SOC_ENUM_SINGLE(R_CH2AIC, FB_CHAIC_MICBST,
				ARRAY_SIZE(in_bst_txt), in_bst_txt);

/***************************
 * R_CH3AIC PG 1 ADDR 0x09 *
 ***************************/

/* Ch3 Input Boost */
static struct soc_enum const in_bst_ch3_enum =
		SOC_ENUM_SINGLE(R_CH3AIC, FB_CHAIC_MICBST,
				ARRAY_SIZE(in_bst_txt), in_bst_txt);

/**************************
 * R_ICTL0 PG 1 ADDR 0x0A *
 **************************/

/* Text */
static char const * const pol_txt[] = {
		"Normal", "Invert"};

/* CH1 Input Polarity */
static struct soc_enum const in_pol_ch1_enum =
		SOC_ENUM_SINGLE(R_ICTL0, FB_ICTL0_IN0POL,
				ARRAY_SIZE(pol_txt), pol_txt);
/* CH0 Input Polarity */
static struct soc_enum const in_pol_ch0_enum =
		SOC_ENUM_SINGLE(R_ICTL0, FB_ICTL0_IN1POL,
				ARRAY_SIZE(pol_txt), pol_txt);
/* Text */
static char const * const in_proc_ch_sel_txt[] = {
		"Normal", "Mono Mix to Channel 0",
		"Mono Mix to Channel 1", "Add"};
/* Input Processor 0/1 Channel Select */
static struct soc_enum const in_proc_ch01_sel_enum =
		SOC_ENUM_SINGLE(R_ICTL0, FB_ICTL0_INPCH10SEL,
				ARRAY_SIZE(in_proc_ch_sel_txt),
				in_proc_ch_sel_txt);

/**************************
 * R_ICTL1 PG 1 ADDR 0x0B *
 **************************/

/* CH3 Input Polarity */
static struct soc_enum const in_pol_ch3_enum =
		SOC_ENUM_SINGLE(R_ICTL1, FB_ICTL1_IN2POL,
				ARRAY_SIZE(pol_txt), pol_txt);
/* CH2 Input Polarity */
static struct soc_enum const in_pol_ch2_enum =
		SOC_ENUM_SINGLE(R_ICTL1, FB_ICTL1_IN3POL,
				ARRAY_SIZE(pol_txt), pol_txt);
/* Input Processor 2/3 Channel Select */
static struct soc_enum const in_proc_ch23_sel_enum =
		SOC_ENUM_SINGLE(R_ICTL1, FB_ICTL1_INPCH32SEL,
				ARRAY_SIZE(in_proc_ch_sel_txt),
				in_proc_ch_sel_txt);

/****************************
 * R_MICBIAS PG 1 ADDR 0x0C *
 ****************************/

/* Text */
static char const * const mic_bias_txt[] = {
		"2.5V", "2.1V", "1.8V", "Vdd"};
/* Mic Bias 2 */
static struct soc_enum const mic_bias_2_enum =
		SOC_ENUM_SINGLE(R_MICBIAS, FB_MICBIAS_MICBOV2,
				ARRAY_SIZE(mic_bias_txt), mic_bias_txt);
/* Mic Bias 1 */
static struct soc_enum const mic_bias_1_enum =
		SOC_ENUM_SINGLE(R_MICBIAS, FB_MICBIAS_MICBOV1,
				ARRAY_SIZE(mic_bias_txt), mic_bias_txt);

/****************************
 * R_PGACTL0 PG 1 ADDR 0x0D *
 * R_PGACTL1 PG 1 ADDR 0x0E *
 * R_PGACTL2 PG 1 ADDR 0x0F *
 * R_PGACTL3 PG 1 ADDR 0x10 *
 ****************************/

static DECLARE_TLV_DB_SCALE(in_pga_vol_tlv_arr, -1725, 75, 0);

/***************************
 * R_ICH0VOL PG1 ADDR 0x12 *
 * R_ICH1VOL PG1 ADDR 0x13 *
 * R_ICH2VOL PG1 ADDR 0x14 *
 * R_ICH3VOL PG1 ADDR 0x15 *
 ***************************/

static DECLARE_TLV_DB_MINMAX(in_vol_tlv_arr, -7125, 2400);

/***************************** 
 * R_ASRCILVOL PG1 ADDR 0x16 *
 * R_ASRCIRVOL PG1 ADDR 0x17 *
 * R_ASRCOLVOL PG1 ADDR 0x18 *
 * R_ASRCORVOL PG1 ADDR 0x19 *
 *****************************/

static DECLARE_TLV_DB_MINMAX(asrc_vol_tlv_arr, -9562, 600);

/***************************
 * R_ALCCTL0 PG1 ADDR 0x1D *
 ***************************/

/* Text */
static char const * const alc_mode_txt[] = {
		"ALC", "Limiter"};
/* ALC mode */
static struct soc_enum const alc_mode_enum =
		SOC_ENUM_SINGLE(R_ALCCTL0, FB_ALCCTL0_ALCMODE,
				ARRAY_SIZE(alc_mode_txt), alc_mode_txt);
/* Text */
static char const * const alc_ref_text[] = {
		"Channel 0", "Channel 1", "Channel 2", "Channel 3", "Peak"};
/* ALC reference */
static struct soc_enum const alc_ref_enum =
		SOC_ENUM_SINGLE(R_ALCCTL0, FB_ALCCTL0_ALCREF,
				ARRAY_SIZE(alc_ref_text), alc_ref_text);

/****************************
 * R_ALCCTL1 PG 1 ADDR 0x1E *
 ****************************/

static DECLARE_TLV_DB_SCALE(alc_max_gain_tlv_arr, -1200, 600, 0);
static DECLARE_TLV_DB_SCALE(alc_target_tlv_arr, -2850, 150, 0);

/****************************
 * R_ALCCTL2 PG 1 ADDR 0x1F *
 ****************************/

static DECLARE_TLV_DB_SCALE(alc_min_gain_tlv_arr, -1725, 600, 0);

/**************************
 * R_NGATE PG 1 ADDR 0x21 *
 **************************/

static DECLARE_TLV_DB_SCALE(ngth_tlv_arr, -7650, 150, 0);
/* Text */
static char const * const ngate_type_txt[] = {
		"PGA Constant", "ADC Mute"};
/* NGate Type */
static struct soc_enum const ngate_type_enum =
		SOC_ENUM_SINGLE(R_NGATE, FB_NGATE_NGG,
				ARRAY_SIZE(ngate_type_txt), ngate_type_txt);

/**************************** 
 * R_DMICCTL PG 1 ADDR 0x22 *
 ****************************/

/* Text */
static char const * const dmic_mono_sel_txt[] = {
		"Stereo", "Mono"};
/* DMIC mono select */
static struct soc_enum const dmic_mono_sel_enum =
		SOC_ENUM_SINGLE(R_DMICCTL, FB_DMICCTL_DMONO,
			ARRAY_SIZE(dmic_mono_sel_txt), dmic_mono_sel_txt);

/***************************
 * R_DACCTL PG 2 ADDR 0x01 *
 ***************************/

/* Rigt DAC Polarity */
static struct soc_enum const dac_pol_r_enum =
		SOC_ENUM_SINGLE(R_DACCTL, FB_DACCTL_DACPOLR,
			ARRAY_SIZE(pol_txt), pol_txt);
/* Left DAC polarity */
static struct soc_enum const dac_pol_l_enum =
		SOC_ENUM_SINGLE(R_DACCTL, FB_DACCTL_DACPOLL,
			ARRAY_SIZE(pol_txt), pol_txt);
/* Text */
static char const * const dac_dith_txt[] = {
		"Half", "Full", "Disabled", "Static"};
/* DAC dither */
static struct soc_enum const dac_dith_enum =
		SOC_ENUM_SINGLE(R_DACCTL, FB_DACCTL_DACDITH,
			ARRAY_SIZE(dac_dith_txt), dac_dith_txt);

/***************************
 * R_SPKCTL PG 2 ADDR 0x02 *
 ***************************/

/* Right speaker polarity */
static struct soc_enum const spk_pol_r_enum =
		SOC_ENUM_SINGLE(R_SPKCTL, FB_SPKCTL_SPKPOLR,
				ARRAY_SIZE(pol_txt), pol_txt);
/* Left speaker polarity */
static struct soc_enum const spk_pol_l_enum =
		SOC_ENUM_SINGLE(R_SPKCTL, FB_SPKCTL_SPKPOLL,
				ARRAY_SIZE(pol_txt), pol_txt);

/***************************
 * R_SUBCTL PG 2 ADDR 0x03 *
 ***************************/

/* SUB polarity */
static struct soc_enum const sub_pol_enum =
		SOC_ENUM_SINGLE(R_SUBCTL, FB_SUBCTL_SUBPOL,
				ARRAY_SIZE(pol_txt), pol_txt);

/**************************
 * R_MVOLL PG 2 ADDR 0x08 *
 * R_MVOLR PG 2 ADDR 0x09 *
 **************************/

/* Master volume */
static DECLARE_TLV_DB_MINMAX(mvol_tlv_arr, -9562, 0);

/***************************
 * R_HPVOLL PG 2 ADDR 0x0A *
 * R_HPVOLR PG 2 ADDR 0x0B *
 ***************************/

/* Headphone volume */
static DECLARE_TLV_DB_SCALE(hp_vol_tlv_arr, -8850, 750, 0);

/****************************
 * R_SPKVOLL PG 2 ADDR 0x0C *
 * R_SPKVOLR PG 2 ADDR 0x0D *
 ****************************/

/* Speaker volume */
static DECLARE_TLV_DB_SCALE(spk_vol_tlv_arr, -7725, 750, 0);

/*******
 * SPK *
 *******/

/******************************
 * R_SPKEQFILT PG 3 ADDR 0x01 *
 ******************************/

/* Text */
static char const * const eq_txt[] = {
	"Pre Scale", 
	"Pre Scale + EQ Band 0",
	"Pre Scale + EQ Band 0 - 1",
	"Pre Scale + EQ Band 0 - 2",
	"Pre Scale + EQ Band 0 - 3",
	"Pre Scale + EQ Band 0 - 4",
	"Pre Scale + EQ Band 0 - 5",
};
/* SPK EQ */
static struct soc_enum const spk_eq_enums[] = {
	SOC_ENUM_SINGLE(R_SPKEQFILT, FB_SPKEQFILT_EQ2BE,
		ARRAY_SIZE(eq_txt), eq_txt),
	SOC_ENUM_SINGLE(R_SPKEQFILT, FB_SPKEQFILT_EQ1BE,
		ARRAY_SIZE(eq_txt), eq_txt),
};

/****************************** 
 * R_SPKMBCCTL PG 3 ADDR 0x0B *
 ******************************/

/* Text */
static char const * const lvl_mode_txt[] = {
		"Average", "Peak"};
/* MBC3 level detect mode */
static struct soc_enum const spk_mbc3_lvl_det_mode_enum =
		SOC_ENUM_SINGLE(R_SPKMBCCTL, FB_SPKMBCCTL_LVLMODE3,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* Text */
static char const * const win_sel_txt[] = {
		"512", "64"};
/* MBC3 window select */
static struct soc_enum const spk_mbc3_win_sel_enum =
		SOC_ENUM_SINGLE(R_SPKMBCCTL, FB_SPKMBCCTL_WINSEL3,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);
/* MBC2 level detect mode */
static struct soc_enum const spk_mbc2_lvl_det_mode_enum =
		SOC_ENUM_SINGLE(R_SPKMBCCTL, FB_SPKMBCCTL_LVLMODE2,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* MBC2 window select */
static struct soc_enum const spk_mbc2_win_sel_enum =
		SOC_ENUM_SINGLE(R_SPKMBCCTL, FB_SPKMBCCTL_WINSEL2,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);
/* MBC1 level detect mode */
static struct soc_enum const spk_mbc1_lvl_det_mode_enum =
		SOC_ENUM_SINGLE(R_SPKMBCCTL, FB_SPKMBCCTL_LVLMODE1,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* MBC1 window select */
static struct soc_enum const spk_mbc1_win_sel_enum =
		SOC_ENUM_SINGLE(R_SPKMBCCTL, FB_SPKMBCCTL_WINSEL1,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);

/*******************************
 * R_SPKMBCMUG1 PG 3 ADDR 0x0C *
 *******************************/

/* Speaker MBC phase invert */
static struct soc_enum const spk_mbc1_phase_pol_enum =
		SOC_ENUM_SINGLE(R_SPKMBCMUG1, FB_SPKMBCMUG_PHASE,
				ARRAY_SIZE(pol_txt), pol_txt);
/* Speaker MBC make up gain */
static DECLARE_TLV_DB_MINMAX(mbc_mug_tlv_arr, -4650, 0);

/*******************************
 * R_SPKMBCTHR1 PG 3 ADDR 0x0D *
 *******************************/

static DECLARE_TLV_DB_MINMAX(thr_tlv_arr, -9562, 0);

/*******************************
 * R_SPKMBCRAT1 PG 3 ADDR 0x0E *
 *******************************/

/* Text */
static char const * const comp_rat_txt[] = {
		"Reserved", "1.5:1", "2:1", "3:1", "4:1", "5:1", "6:1",
		"7:1", "8:1", "9:1", "10:1", "11:1", "12:1", "13:1", "14:1",
		"15:1", "16:1", "17:1", "18:1", "19:1", "20:1"};

/* Speaker compressor ratio */
static struct soc_enum const spk_mbc1_comp_rat_enum =
		SOC_ENUM_SINGLE(R_SPKMBCRAT1, FB_SPKMBCRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/*******************************
 * R_SPKMBCMUG2 PG 3 ADDR 0x13 *
 *******************************/

/* Speaker MBC phase invert */
static struct soc_enum const spk_mbc2_phase_pol_enum =
		SOC_ENUM_SINGLE(R_SPKMBCMUG2, FB_SPKMBCMUG_PHASE,
				ARRAY_SIZE(pol_txt), pol_txt);

/*******************************
 * R_SPKMBCRAT2 PG 3 ADDR 0x15 *
 *******************************/

/* Speaker compressor ratio */
static struct soc_enum const spk_mbc2_comp_rat_enum =
		SOC_ENUM_SINGLE(R_SPKMBCRAT2, FB_SPKMBCRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/*******************************
 * R_SPKMBCMUG3 PG 3 ADDR 0x1A *
 *******************************/

/* Speaker MBC phase invert */
static struct soc_enum const spk_mbc3_phase_pol_enum =
		SOC_ENUM_SINGLE(R_SPKMBCMUG3, FB_SPKMBCMUG_PHASE,
				ARRAY_SIZE(pol_txt), pol_txt);

/*******************************
 * R_SPKMBCRAT3 PG 3 ADDR 0x1C *
 *******************************/

/* Speaker compressor ratio */
static struct soc_enum const spk_mbc3_comp_rat_enum =
		SOC_ENUM_SINGLE(R_SPKMBCRAT3, FB_SPKMBCRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/******************************
 * R_SPKCLECTL PG 3 ADDR 0x21 *
 ******************************/

/* Speaker CLE level mode */
static struct soc_enum const spk_cle_lvl_mode_enum =
		SOC_ENUM_SINGLE(R_SPKCLECTL, FB_SPKCLECTL_LVLMODE,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* Speaker CLE window selection */
static struct soc_enum const spk_cle_win_sel_enum =
		SOC_ENUM_SINGLE(R_SPKCLECTL, FB_SPKCLECTL_WINSEL,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);

/******************************
 * R_SPKCLEMUG PG 3 ADDR 0x22 *
 ******************************/

static DECLARE_TLV_DB_MINMAX(cle_mug_tlv_arr, 0, 4650);

/*******************************
 * R_SPKCOMPRAT PG 3 ADDR 0x24 *
 *******************************/

static struct soc_enum const spk_comp_rat_enum =
		SOC_ENUM_SINGLE(R_SPKCOMPRAT, FB_SPKCOMPRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/******************************
 * R_SPKEXPTHR PG 3 ADDR 0x2F *
 ******************************/

/* Text */
static char const * const exp_rat_txt[] = {
		"Reserved", "Reserved", "1:2", "1:3",
		"1:4", "1:5", "1:6", "1:7"};
/* Speaker expander ratio */
static struct soc_enum const spk_exp_rat_enum =
		SOC_ENUM_SINGLE(R_SPKEXPRAT, FB_SPKEXPRAT_RATIO,
				ARRAY_SIZE(exp_rat_txt), exp_rat_txt);

/*******
 * DAC *
 *******/

/******************************
 * R_DACEQFILT PG 4 ADDR 0x01 *
 ******************************/

/* DAC EQ */
static struct soc_enum const dac_eq_enums[] = {
	SOC_ENUM_SINGLE(R_DACEQFILT, FB_DACEQFILT_EQ2BE,
		ARRAY_SIZE(eq_txt), eq_txt),
	SOC_ENUM_SINGLE(R_DACEQFILT, FB_DACEQFILT_EQ1BE,
		ARRAY_SIZE(eq_txt), eq_txt),
};

/****************************** 
 * R_DACMBCCTL PG 4 ADDR 0x0B *
 ******************************/

/* MBC3 level detect mode */
static struct soc_enum const dac_mbc3_lvl_det_mode_enum =
		SOC_ENUM_SINGLE(R_DACMBCCTL, FB_DACMBCCTL_LVLMODE3,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* MBC3 window select */
static struct soc_enum const dac_mbc3_win_sel_enum =
		SOC_ENUM_SINGLE(R_DACMBCCTL, FB_DACMBCCTL_WINSEL3,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);
/* MBC2 level detect mode */
static struct soc_enum const dac_mbc2_lvl_det_mode_enum =
		SOC_ENUM_SINGLE(R_DACMBCCTL, FB_DACMBCCTL_LVLMODE2,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* MBC2 window select */
static struct soc_enum const dac_mbc2_win_sel_enum =
		SOC_ENUM_SINGLE(R_DACMBCCTL, FB_DACMBCCTL_WINSEL2,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);
/* MBC1 level detect mode */
static struct soc_enum const dac_mbc1_lvl_det_mode_enum =
		SOC_ENUM_SINGLE(R_DACMBCCTL, FB_DACMBCCTL_LVLMODE1,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* MBC1 window select */
static struct soc_enum const dac_mbc1_win_sel_enum =
		SOC_ENUM_SINGLE(R_DACMBCCTL, FB_DACMBCCTL_WINSEL1,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);

/*******************************
 * R_DACMBCMUG1 PG 4 ADDR 0x0C *
 *******************************/

/* DAC MBC phase invert */
static struct soc_enum const dac_mbc1_phase_pol_enum =
		SOC_ENUM_SINGLE(R_DACMBCMUG1, FB_DACMBCMUG_PHASE,
				ARRAY_SIZE(pol_txt), pol_txt);

/*******************************
 * R_DACMBCRAT1 PG 4 ADDR 0x0E *
 *******************************/

/* DAC compressor ratio */
static struct soc_enum const dac_mbc1_comp_rat_enum =
		SOC_ENUM_SINGLE(R_DACMBCRAT1, FB_DACMBCRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/*******************************
 * R_DACMBCMUG2 PG 4 ADDR 0x13 *
 *******************************/

/* DAC MBC phase invert */
static struct soc_enum const dac_mbc2_phase_pol_enum =
		SOC_ENUM_SINGLE(R_DACMBCMUG2, FB_DACMBCMUG_PHASE,
				ARRAY_SIZE(pol_txt), pol_txt);

/*******************************
 * R_DACMBCRAT2 PG 4 ADDR 0x15 *
 *******************************/

/* DAC compressor ratio */
static struct soc_enum const dac_mbc2_comp_rat_enum =
		SOC_ENUM_SINGLE(R_DACMBCRAT2, FB_DACMBCRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/*******************************
 * R_DACMBCMUG3 PG 4 ADDR 0x1A *
 *******************************/

/* DAC MBC phase invert */
static struct soc_enum const dac_mbc3_phase_pol_enum =
		SOC_ENUM_SINGLE(R_DACMBCMUG3, FB_DACMBCMUG_PHASE,
				ARRAY_SIZE(pol_txt), pol_txt);

/*******************************
 * R_DACMBCRAT3 PG 4 ADDR 0x1C *
 *******************************/

/* DAC compressor ratio */
static struct soc_enum const dac_mbc3_comp_rat_enum =
		SOC_ENUM_SINGLE(R_DACMBCRAT3, FB_DACMBCRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/******************************
 * R_DACCLECTL PG 4 ADDR 0x21 *
 ******************************/

/* DAC CLE level mode */
static struct soc_enum const dac_cle_lvl_mode_enum =
		SOC_ENUM_SINGLE(R_DACCLECTL, FB_DACCLECTL_LVLMODE,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* DAC CLE window selection */
static struct soc_enum const dac_cle_win_sel_enum =
		SOC_ENUM_SINGLE(R_DACCLECTL, FB_DACCLECTL_WINSEL,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);

/*******************************
 * R_DACCOMPRAT PG 4 ADDR 0x24 *
 *******************************/

static struct soc_enum const dac_comp_rat_enum =
		SOC_ENUM_SINGLE(R_DACCOMPRAT, FB_DACCOMPRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/******************************
 * R_DACEXPRAT PG 4 ADDR 0x30 *
 ******************************/

/* DAC expander ratio */
static struct soc_enum const dac_exp_rat_enum =
		SOC_ENUM_SINGLE(R_DACEXPRAT, FB_DACEXPRAT_RATIO,
				ARRAY_SIZE(exp_rat_txt), exp_rat_txt);

/*******
 * SUB *
 *******/

/******************************
 * R_SUBEQFILT PG 5 ADDR 0x01 *
 ******************************/

/* SUB EQ */
static struct soc_enum const sub_eq_enums[] = {
	SOC_ENUM_SINGLE(R_SUBEQFILT, FB_SUBEQFILT_EQ2BE,
		ARRAY_SIZE(eq_txt), eq_txt),
	SOC_ENUM_SINGLE(R_SUBEQFILT, FB_SUBEQFILT_EQ1BE,
		ARRAY_SIZE(eq_txt), eq_txt),
};

/****************************** 
 * R_SUBMBCCTL PG 5 ADDR 0x0B *
 ******************************/

/* MBC3 level detect mode */
static struct soc_enum const sub_mbc3_lvl_det_mode_enum =
		SOC_ENUM_SINGLE(R_SUBMBCCTL, FB_SUBMBCCTL_LVLMODE3,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* MBC3 window select */
static struct soc_enum const sub_mbc3_win_sel_enum =
		SOC_ENUM_SINGLE(R_SUBMBCCTL, FB_SUBMBCCTL_WINSEL3,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);
/* MBC2 level detect mode */
static struct soc_enum const sub_mbc2_lvl_det_mode_enum =
		SOC_ENUM_SINGLE(R_SUBMBCCTL, FB_SUBMBCCTL_LVLMODE2,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* MBC2 window select */
static struct soc_enum const sub_mbc2_win_sel_enum =
		SOC_ENUM_SINGLE(R_SUBMBCCTL, FB_SUBMBCCTL_WINSEL2,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);
/* MBC1 level detect mode */
static struct soc_enum const sub_mbc1_lvl_det_mode_enum =
		SOC_ENUM_SINGLE(R_SUBMBCCTL, FB_SUBMBCCTL_LVLMODE1,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* MBC1 window select */
static struct soc_enum const sub_mbc1_win_sel_enum =
		SOC_ENUM_SINGLE(R_SUBMBCCTL, FB_SUBMBCCTL_WINSEL1,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);

/*******************************
 * R_SUBMBCMUG1 PG 5 ADDR 0x0C *
 *******************************/

/* SUB MBC phase invert */
static struct soc_enum const sub_mbc1_phase_pol_enum =
		SOC_ENUM_SINGLE(R_SUBMBCMUG1, FB_SUBMBCMUG_PHASE,
				ARRAY_SIZE(pol_txt), pol_txt);

/*******************************
 * R_SUBMBCRAT1 PG 5 ADDR 0x0E *
 *******************************/

/* SUB compressor ratio */
static struct soc_enum const sub_mbc1_comp_rat_enum =
		SOC_ENUM_SINGLE(R_SUBMBCRAT1, FB_SUBMBCRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/*******************************
 * R_SUBMBCMUG2 PG 5 ADDR 0x13 *
 *******************************/

/* SUB MBC phase invert */
static struct soc_enum const sub_mbc2_phase_pol_enum =
		SOC_ENUM_SINGLE(R_SUBMBCMUG2, FB_SUBMBCMUG_PHASE,
				ARRAY_SIZE(pol_txt), pol_txt);

/*******************************
 * R_SUBMBCRAT2 PG 5 ADDR 0x15 *
 *******************************/

/* SUB compressor ratio */
static struct soc_enum const sub_mbc2_comp_rat_enum =
		SOC_ENUM_SINGLE(R_SUBMBCRAT2, FB_SUBMBCRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/*******************************
 * R_SUBMBCMUG3 PG 5 ADDR 0x1A *
 *******************************/

/* SUB MBC phase invert */
static struct soc_enum const sub_mbc3_phase_pol_enum =
		SOC_ENUM_SINGLE(R_SUBMBCMUG3, FB_SUBMBCMUG_PHASE,
				ARRAY_SIZE(pol_txt), pol_txt);

/*******************************
 * R_SUBMBCRAT3 PG 5 ADDR 0x1C *
 *******************************/

/* SUB compressor ratio */
static struct soc_enum const sub_mbc3_comp_rat_enum =
		SOC_ENUM_SINGLE(R_SUBMBCRAT3, FB_SUBMBCRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/******************************
 * R_SUBCLECTL PG 5 ADDR 0x21 *
 ******************************/

/* SUB CLE level mode */
static struct soc_enum const sub_cle_lvl_mode_enum =
		SOC_ENUM_SINGLE(R_SUBCLECTL, FB_SUBCLECTL_LVLMODE,
				ARRAY_SIZE(lvl_mode_txt), lvl_mode_txt);
/* SUB CLE window selection */
static struct soc_enum const sub_cle_win_sel_enum =
		SOC_ENUM_SINGLE(R_SUBCLECTL, FB_SUBCLECTL_WINSEL,
				ARRAY_SIZE(win_sel_txt), win_sel_txt);

/*******************************
 * R_SUBCOMPRAT PG 5 ADDR 0x24 *
 *******************************/

static struct soc_enum const sub_comp_rat_enum =
		SOC_ENUM_SINGLE(R_SUBCOMPRAT, FB_SUBCOMPRAT_RATIO,
				ARRAY_SIZE(comp_rat_txt), comp_rat_txt);

/******************************
 * R_SUBEXPRAT PG 5 ADDR 0x30 *
 ******************************/

/* SUB expander ratio */
static struct soc_enum const sub_exp_rat_enum =
		SOC_ENUM_SINGLE(R_SUBEXPRAT, FB_SUBEXPRAT_RATIO,
				ARRAY_SIZE(exp_rat_txt), exp_rat_txt);

static int bytes_info_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *ucontrol)
{
	struct coeff_ram_ctl *ctl =
		(struct coeff_ram_ctl *)kcontrol->private_value;
	struct soc_bytes_ext *params = &ctl->bytes_ext;

	ucontrol->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	ucontrol->count = params->max;

	return 0;
}

/********************
 * CH 0_1 Input Mux *
 ********************/

static char const * const ch_0_1_mux_txt[] = {"DAI 1", "TDM 0_1"};

static struct soc_enum const ch_0_1_mux_enum =
		SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
				ARRAY_SIZE(ch_0_1_mux_txt), ch_0_1_mux_txt);

static struct snd_kcontrol_new const ch_0_1_mux_dapm_enum =
		SOC_DAPM_ENUM("CH 0_1 Input Mux",  ch_0_1_mux_enum);

/********************
 * CH 2_3 Input Mux *
 ********************/

static char const * const ch_2_3_mux_txt[] = {"DAI 2", "TDM 2_3"};

static struct soc_enum const ch_2_3_mux_enum =
		SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
				ARRAY_SIZE(ch_2_3_mux_txt), ch_2_3_mux_txt);

static struct snd_kcontrol_new const ch_2_3_mux_dapm_enum =
		SOC_DAPM_ENUM("CH 2_3 Input Mux",  ch_2_3_mux_enum);

/********************
 * CH 4_5 Input Mux *
 ********************/

static char const * const ch_4_5_mux_txt[] = {"DAI 3", "TDM 4_5"};

static struct soc_enum const ch_4_5_mux_enum =
		SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
				ARRAY_SIZE(ch_4_5_mux_txt), ch_4_5_mux_txt);

static struct snd_kcontrol_new const ch_4_5_mux_dapm_enum =
		SOC_DAPM_ENUM("CH 4_5 Input Mux",  ch_4_5_mux_enum);

#define COEFF_RAM_CTL(xname, xcount, xaddr) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = bytes_info_ext, \
	.get = coeff_ram_get, .put = coeff_ram_put, \
	.private_value = (unsigned long)&(struct coeff_ram_ctl) { \
		.addr = xaddr, \
		.bytes_ext = {.max = xcount, }, \
	} \
}

static struct snd_kcontrol_new const tscs454_snd_controls[] = {
	/* R_SCLKCTL PG 0 ADDR 0x18 */
	SOC_ENUM("ADC Modular Rate", adc_modular_rate_enum),
	SOC_ENUM("DAC Modular Rate", dac_modular_rate_enum),
	/* R_ASRC PG 0 ADDR 0x28 */
	SOC_SINGLE("ASRC Out High Bandwidth Capture Switch",
			R_ASRC, FB_ASRC_ASRCOBW, 1, 0),
	SOC_SINGLE("ASRC In High Bandwidth Playback Switch",
			R_ASRC, FB_ASRC_ASRCIBW, 1, 0),
	/* R_I2SIDCTL PG 0 ADDR 0x38 */
	SOC_ENUM("I2S 1 Data In Control", data_in_ctrl_enums[0]),
	SOC_ENUM("I2S 2 Data In Control", data_in_ctrl_enums[1]),
	SOC_ENUM("I2S 3 Data In Control", data_in_ctrl_enums[2]),
	/* R_I2SODCTL PG 0 ADDR 0x39 */
	SOC_ENUM("I2S 1 Data Out Control", data_out_ctrl_enums[0]),
	SOC_ENUM("I2S 2 Data Out Control", data_out_ctrl_enums[1]),
	SOC_ENUM("I2S 3 Data Out Control", data_out_ctrl_enums[2]),
	/* R_HSDCTL1 PG 1 ADDR 0x01 */
	SOC_ENUM("Headphone Jack Type", hp_jack_type_enum),
	SOC_ENUM("Headset Detection Polarity", hs_det_pol_enum),
	SOC_SINGLE("Headphone Detection Switch",
			R_HSDCTL1, FB_HSDCTL1_HPID_EN, 1, 0),
	SOC_SINGLE("Headset OMTP/CTIA Switch",
			R_HSDCTL1, FB_HSDCTL1_GBLHS_EN, 1, 0),
	/* R_CH0AIC PG 1 ADDR 0x06 */
	SOC_ENUM("Input Boost Channel 0", in_bst_ch0_enum),
	/* R_CH1AIC PG 1 ADDR 0x07 */
	SOC_ENUM("Input Boost Channel 1", in_bst_ch1_enum),
	/* R_CH2AIC PG 1 ADDR 0x08 */
	SOC_ENUM("Input Boost Channel 2", in_bst_ch2_enum),
	/* R_CH3AIC PG 1 ADDR 0x09 */
	SOC_ENUM("Input Boost Channel 3", in_bst_ch3_enum),
	/* R_ICTL0 PG 1 ADDR 0x0A */
	SOC_ENUM("Input Channel 1 Polarity", in_pol_ch1_enum),
	SOC_ENUM("Input Channel 0 Polarity", in_pol_ch0_enum),
	SOC_ENUM("Input Processor Channel 0/1 Operation",
			in_proc_ch01_sel_enum),
	SOC_SINGLE("Input Channel 1 Mute Capture Switch",
			R_ICTL0, FB_ICTL0_IN1MUTE, 1, 0),
	SOC_SINGLE("Input Channel 0 Mute Capture Switch",
			R_ICTL0, FB_ICTL0_IN0MUTE, 1, 0),
	SOC_SINGLE("Input Channel 1 HPF Disable Capture Switch",
			R_ICTL0, FB_ICTL0_IN1HP, 1, 0),
	SOC_SINGLE("Input Channel 0 HPF Disable Capture Switch",
			R_ICTL0, FB_ICTL0_IN0HP, 1, 0),
	/* R_ICTL1 PG 1 ADDR 0x0B */
	SOC_ENUM("Input Channel 3 Polarity", in_pol_ch3_enum),
	SOC_ENUM("Input Channel 2 Polarity", in_pol_ch2_enum),
	SOC_ENUM("Input Processor Channel 2/3 Operation",
			in_proc_ch23_sel_enum),
	SOC_SINGLE("Input Channel 3 Mute Capture Switch",
			R_ICTL1, FB_ICTL1_IN3MUTE, 1, 0),
	SOC_SINGLE("Input Channel 2 Mute Capture Switch",
			R_ICTL1, FB_ICTL1_IN2MUTE, 1, 0),
	SOC_SINGLE("Input Channel 3 HPF Disable Capture Switch",
			R_ICTL1, FB_ICTL1_IN3HP, 1, 0),
	SOC_SINGLE("Input Channel 2 HPF Disable Capture Switch",
			R_ICTL1, FB_ICTL1_IN2HP, 1, 0),
	/* R_MICBIAS PG 1 ADDR 0x0C */
	SOC_ENUM("Mic Bias 2 Voltage", mic_bias_2_enum),
	SOC_ENUM("Mic Bias 1 Voltage", mic_bias_1_enum),
	/* R_PGACTL0 PG 1 ADDR 0x0D */
	SOC_SINGLE("Input Channel 0 PGA Mute Capture Switch",
			R_PGACTL0, FB_PGACTL_PGAMUTE, 1, 0),
	SOC_SINGLE_TLV("Input Channel 0 PGA Capture Volume", R_PGACTL0,
			FB_PGACTL_PGAVOL,
			FM_PGACTL_PGAVOL, 0, in_pga_vol_tlv_arr),
	/* R_PGACTL1 PG 1 ADDR 0x0E */
	SOC_SINGLE("Input Channel 1 PGA Mute Capture Switch",
			R_PGACTL1, FB_PGACTL_PGAMUTE, 1, 0),
	SOC_SINGLE_TLV("Input Channel 1 PGA Capture Volume", R_PGACTL1,
			FB_PGACTL_PGAVOL,
			FM_PGACTL_PGAVOL, 0, in_pga_vol_tlv_arr),
	/* R_PGACTL2 PG 1 ADDR 0x0F */
	SOC_SINGLE("Input Channel 2 PGA Mute Capture Switch",
			R_PGACTL2, FB_PGACTL_PGAMUTE, 1, 0),
	SOC_SINGLE_TLV("Input Channel 2 PGA Capture Volume", R_PGACTL2,
			FB_PGACTL_PGAVOL,
			FM_PGACTL_PGAVOL, 0, in_pga_vol_tlv_arr),
	/* R_PGACTL3 PG 1 ADDR 0x10 */
	SOC_SINGLE("Input Channel 3 PGA Mute Capture Switch",
			R_PGACTL3, FB_PGACTL_PGAMUTE, 1, 0),
	SOC_SINGLE_TLV("Input Channel 3 PGA Capture Volume", R_PGACTL3,
			FB_PGACTL_PGAVOL,
			FM_PGACTL_PGAVOL, 0, in_pga_vol_tlv_arr),
	/* R_ICH0VOL PG 1 ADDR 0x12 */
	SOC_SINGLE_TLV("Input Channel 0 Capture Volume", R_ICH0VOL,
			FB_ICHVOL_ICHVOL, FM_ICHVOL_ICHVOL, 0, in_vol_tlv_arr),
	/* R_ICH1VOL PG 1 ADDR 0x13 */
	SOC_SINGLE_TLV("Input Channel 1 Capture Volume", R_ICH1VOL,
			FB_ICHVOL_ICHVOL, FM_ICHVOL_ICHVOL, 0, in_vol_tlv_arr),
	/* R_ICH2VOL PG 1 ADDR 0x14 */
	SOC_SINGLE_TLV("Input Channel 2 Capture Volume", R_ICH2VOL,
			FB_ICHVOL_ICHVOL, FM_ICHVOL_ICHVOL, 0, in_vol_tlv_arr),
	/* R_ICH3VOL PG 1 ADDR 0x15 */
	SOC_SINGLE_TLV("Input Channel 3 Capture Volume", R_ICH3VOL,
			FB_ICHVOL_ICHVOL, FM_ICHVOL_ICHVOL, 0, in_vol_tlv_arr),
	/* R_ASRCILVOL PG 1 ADDR 0x16 */
	SOC_SINGLE_TLV("ASRC Input Left Volume", R_ASRCILVOL,
			FB_ASRCILVOL_ASRCILVOL, FM_ASRCILVOL_ASRCILVOL,
			0, asrc_vol_tlv_arr),
	/* R_ASRCIRVOL PG 1 ADDR 0x17 */
	SOC_SINGLE_TLV("ASRC Input Right Volume", R_ASRCIRVOL,
			FB_ASRCIRVOL_ASRCIRVOL, FM_ASRCIRVOL_ASRCIRVOL,
			0, asrc_vol_tlv_arr),
	/* R_ASRCOLVOL PG 1 ADDR 0x18 */
	SOC_SINGLE_TLV("ASRC Output Left Volume", R_ASRCOLVOL,
			FB_ASRCOLVOL_ASRCOLVOL, FM_ASRCOLVOL_ASRCOLVOL,
			0, asrc_vol_tlv_arr),
	/* R_ASRCORVOL PG 1 ADDR 0x19 */
	SOC_SINGLE_TLV("ASRC Output Right Volume", R_ASRCORVOL,
			FB_ASRCORVOL_ASRCOLVOL, FM_ASRCORVOL_ASRCOLVOL,
			0, asrc_vol_tlv_arr),
	/* R_IVOLCTLU PG 1 ADDR 0x1C */
	/* R_ALCCTL0 PG 1 ADDR 0x1D */
	SOC_ENUM("ALC Mode", alc_mode_enum),
	SOC_ENUM("ALC Reference", alc_ref_enum),
	SOC_SINGLE("Input Channel 3 ALC Capture Switch",
			R_ALCCTL0, FB_ALCCTL0_ALCEN3, 1, 0),
	SOC_SINGLE("Input Channel 2 ALC Capture Switch",
			R_ALCCTL0, FB_ALCCTL0_ALCEN2, 1, 0),
	SOC_SINGLE("Input Channel 1 ALC Capture Switch",
			R_ALCCTL0, FB_ALCCTL0_ALCEN1, 1, 0),
	SOC_SINGLE("Input Channel 0 ALC Capture Switch",
			R_ALCCTL0, FB_ALCCTL0_ALCEN0, 1, 0),
	/* R_ALCCTL1 PG 1 ADDR 0x1E */
	SOC_SINGLE_TLV("ALC Max Gain", R_ALCCTL1,
			FB_ALCCTL1_MAXGAIN, FM_ALCCTL1_MAXGAIN,
			0, alc_max_gain_tlv_arr),
	SOC_SINGLE_TLV("ALC Target", R_ALCCTL1,
			FB_ALCCTL1_ALCL, FM_ALCCTL1_ALCL,
			0, alc_target_tlv_arr),
	/* R_ALCCTL2 PG 1 ADDR 0x1F */
	SOC_SINGLE("ALC Zero Cross Capture Switch",
			R_ALCCTL2, FB_ALCCTL2_ALCZC, 1, 0),
	SOC_SINGLE_TLV("ALC Min Gain", R_ALCCTL2,
			FB_ALCCTL2_MINGAIN, FM_ALCCTL2_MINGAIN,
			0, alc_min_gain_tlv_arr),
	SOC_SINGLE_RANGE("ALC Hold", R_ALCCTL2,
			FB_ALCCTL2_HLD, 0, FM_ALCCTL2_HLD, 0),
	/* R_ALCCTL3 PG 1 ADDR 0x20 */
	SOC_SINGLE_RANGE("ALC Decay", R_ALCCTL3,
			FB_ALCCTL3_DCY, 0, FM_ALCCTL3_DCY, 0),
	SOC_SINGLE_RANGE("ALC Attack", R_ALCCTL3,
			FB_ALCCTL3_ATK, 0, FM_ALCCTL3_ATK, 0),
	/* R_NGATE PG 1 ADDR 0x21 */
	SOC_SINGLE_TLV("Noise Gate Threshold", R_NGATE,
			FB_NGATE_NGTH, FM_NGATE_NGTH, 0, ngth_tlv_arr),
	SOC_ENUM("Noise Gate Type", ngate_type_enum),
	SOC_SINGLE("Noise Gate Switch", R_NGATE, FB_NGATE_NGAT, 1, 0),
	/* R_DMICCTL PG 1 ADDR 0x22 */
	SOC_SINGLE("Digital Mic 2 Switch", R_DMICCTL, FB_DMICCTL_DMIC2EN, 1, 0),
	SOC_SINGLE("Digital Mic 1 Switch", R_DMICCTL, FB_DMICCTL_DMIC1EN, 1, 0),
	SOC_ENUM("Digital Mic Mono Select", dmic_mono_sel_enum),
	/* R_DACCTL PG 2 ADDR 0x01 */
	SOC_ENUM("DAC Polarity Left", dac_pol_r_enum),
	SOC_ENUM("DAC Polarity Right", dac_pol_l_enum),
	SOC_ENUM("DAC Dither", dac_dith_enum),
	SOC_SINGLE("DAC Mute Switch", R_DACCTL, FB_DACCTL_DACMUTE, 1, 0),
	SOC_SINGLE("DAC De-Emphasis Switch", R_DACCTL, FB_DACCTL_DACDEM, 1, 0),
	/* R_SPKCTL PG 2 ADDR 0x02 */
	SOC_ENUM("Speaker Polarity Right", spk_pol_r_enum),
	SOC_ENUM("Speaker Polarity Left", spk_pol_l_enum),
	SOC_SINGLE("Speaker Mute Switch", R_SPKCTL, FB_SPKCTL_SPKMUTE, 1, 0),
	SOC_SINGLE("Speaker De-Emphasis Switch",
			R_SPKCTL, FB_SPKCTL_SPKDEM, 1, 0),
	/* R_SUBCTL PG 2 ADDR 0x03 */
	SOC_ENUM("Sub Polarity", sub_pol_enum),
	SOC_SINGLE("SUB Mute Switch", R_SUBCTL, FB_SUBCTL_SUBMUTE, 1, 0),
	SOC_SINGLE("Sub De-Emphasis Switch", R_SUBCTL, FB_SUBCTL_SUBDEM, 1, 0),
	/* R_DCCTL PG 2 ADDR 0x04 */
	SOC_SINGLE("Sub DC Removal Switch", R_DCCTL, FB_DCCTL_SUBDCBYP, 1, 1),
	SOC_SINGLE("DAC DC Removal Switch", R_DCCTL, FB_DCCTL_DACDCBYP, 1, 1),
	SOC_SINGLE("Speaker DC Removal Switch",
			R_DCCTL, FB_DCCTL_SPKDCBYP, 1, 1),
	SOC_SINGLE("DC Removal Coefficient", R_DCCTL, FB_DCCTL_DCCOEFSEL,
			FM_DCCTL_DCCOEFSEL, 0),
	/* R_OVOLCTLU PG 2 ADDR 0x06 */
	SOC_SINGLE("Output Volume Fade Switch",
			R_OVOLCTLU, FB_OVOLCTLU_OFADE, 1, 0),
	/* R_MVOLL PG 2 ADDR 0x08 */
	/* R_MVOLR PG 2 ADDR 0x09 */
	SOC_DOUBLE_R_TLV("Master Playback Volume", R_MVOLL, R_MVOLR,
			FB_MVOLL_MVOL_L, FM_MVOLL_MVOL_L, 0, mvol_tlv_arr),
	/* R_HPVOLL PG 2 ADDR 0x0A */
	/* R_HPVOLR PG 2 ADDR 0x0B */
	SOC_DOUBLE_R_TLV("Headphone Playback Volume", R_HPVOLL, R_HPVOLR,
			FB_HPVOLL_HPVOL_L, FM_HPVOLL_HPVOL_L, 0,
			hp_vol_tlv_arr),
	/* R_SPKVOLL PG 2 ADDR 0x0C */
	/* R_SPKVOLR PG 2 ADDR 0x0D */
	SOC_DOUBLE_R_TLV("Speaker Playback Volume", R_SPKVOLL, R_SPKVOLR,
			FB_SPKVOLL_SPKVOL_L, FM_SPKVOLL_SPKVOL_L, 0,
			spk_vol_tlv_arr),
	/* R_SUBVOL PG 2 ADDR 0x10 */
	SOC_SINGLE_TLV("Sub Playback Volume", R_SUBVOL,
			FB_SUBVOL_SUBVOL, FM_SUBVOL_SUBVOL, 0, spk_vol_tlv_arr),
	/*******
	 * SPK *
	 *******/
	/* R_SPKEQFILT PG 3 ADDR 0x01 */
	SOC_SINGLE("Speaker EQ 2 Switch",
			R_SPKEQFILT, FB_SPKEQFILT_EQ2EN, 1, 0),
	SOC_ENUM("Speaker EQ 2 Band", spk_eq_enums[0]),
	SOC_SINGLE("Speaker EQ 1 Switch",
			R_SPKEQFILT, FB_SPKEQFILT_EQ1EN, 1, 0),
	SOC_ENUM("Speaker EQ 1 Band", spk_eq_enums[1]),
	/* R_SPKMBCEN PG 3 ADDR 0x0A */
	SOC_SINGLE("Speaker MBC 3 Switch",
			R_SPKMBCEN, FB_SPKMBCEN_MBCEN3, 1, 0),
	SOC_SINGLE("Speaker MBC 2 Switch",
			R_SPKMBCEN, FB_SPKMBCEN_MBCEN2, 1, 0),
	SOC_SINGLE("Speaker MBC 1 Switch",
			R_SPKMBCEN, FB_SPKMBCEN_MBCEN1, 1, 0),
	/* R_SPKMBCCTL PG 3 ADDR 0x0B */
	SOC_ENUM("Speaker MBC 3 Mode", spk_mbc3_lvl_det_mode_enum),
	SOC_ENUM("Speaker MBC 3 Window", spk_mbc3_win_sel_enum),
	SOC_ENUM("Speaker MBC 2 Mode", spk_mbc2_lvl_det_mode_enum),
	SOC_ENUM("Speaker MBC 2 Window", spk_mbc2_win_sel_enum),
	SOC_ENUM("Speaker MBC 1 Mode", spk_mbc1_lvl_det_mode_enum),
	SOC_ENUM("Speaker MBC 1 Window", spk_mbc1_win_sel_enum),
	/* R_SPKMBCMUG1 PG 3 ADDR 0x0C */
	SOC_ENUM("Speaker MBC 1 Phase Polarity", spk_mbc1_phase_pol_enum),
	SOC_SINGLE_TLV("Speaker MBC1 Make-Up Gain", R_SPKMBCMUG1,
			FB_SPKMBCMUG_MUGAIN, FM_SPKMBCMUG_MUGAIN,
			0, mbc_mug_tlv_arr),
	/* R_SPKMBCTHR1 PG 3 ADDR 0x0D */
	SOC_SINGLE_TLV("Speaker MBC 1 Compressor Threshold", R_SPKMBCTHR1,
			FB_SPKMBCTHR_THRESH, FM_SPKMBCTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SPKMBCRAT1 PG 3 ADDR 0x0E */
	SOC_ENUM("Speaker MBC 1 Compressor Ratio", spk_mbc1_comp_rat_enum),
	/* R_SPKMBCATK1L PG 3 ADDR 0x0F */
	/* R_SPKMBCATK1H PG 3 ADDR 0x10 */
	SND_SOC_BYTES("Speaker MBC 1 Attack", R_SPKMBCATK1L, 2),
	/* R_SPKMBCREL1L PG 3 ADDR 0x11 */
	/* R_SPKMBCREL1H PG 3 ADDR 0x12 */
	SND_SOC_BYTES("Speaker MBC 1 Release", R_SPKMBCREL1L, 2),
	/* R_SPKMBCMUG2 PG 3 ADDR 0x13 */
	SOC_ENUM("Speaker MBC 2 Phase Polarity", spk_mbc2_phase_pol_enum),
	SOC_SINGLE_TLV("Speaker MBC2 Make-Up Gain", R_SPKMBCMUG2,
			FB_SPKMBCMUG_MUGAIN, FM_SPKMBCMUG_MUGAIN,
			0, mbc_mug_tlv_arr),
	/* R_SPKMBCTHR2 PG 3 ADDR 0x14 */
	SOC_SINGLE_TLV("Speaker MBC 2 Compressor Threshold", R_SPKMBCTHR2,
			FB_SPKMBCTHR_THRESH, FM_SPKMBCTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SPKMBCRAT2 PG 3 ADDR 0x15 */
	SOC_ENUM("Speaker MBC 2 Compressor Ratio", spk_mbc2_comp_rat_enum),
	/* R_SPKMBCATK2L PG 3 ADDR 0x16 */
	/* R_SPKMBCATK2H PG 3 ADDR 0x17 */
	SND_SOC_BYTES("Speaker MBC 2 Attack", R_SPKMBCATK2L, 2),
	/* R_SPKMBCREL2L PG 3 ADDR 0x18 */
	/* R_SPKMBCREL2H PG 3 ADDR 0x19 */
	SND_SOC_BYTES("Speaker MBC 2 Release", R_SPKMBCREL2L, 2),
	/* R_SPKMBCMUG3 PG 3 ADDR 0x1A */
	SOC_ENUM("Speaker MBC 3 Phase Polarity", spk_mbc3_phase_pol_enum),
	SOC_SINGLE_TLV("Speaker MBC 3 Make-Up Gain", R_SPKMBCMUG3,
			FB_SPKMBCMUG_MUGAIN, FM_SPKMBCMUG_MUGAIN,
			0, mbc_mug_tlv_arr),
	/* R_SPKMBCTHR3 PG 3 ADDR 0x1B */
	SOC_SINGLE_TLV("Speaker MBC 3 Threshold", R_SPKMBCTHR3,
			FB_SPKMBCTHR_THRESH, FM_SPKMBCTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SPKMBCRAT3 PG 3 ADDR 0x1C */
	SOC_ENUM("Speaker MBC 3 Compressor Ratio", spk_mbc3_comp_rat_enum),
	/* R_SPKMBCATK3L PG 3 ADDR 0x1D */
	/* R_SPKMBCATK3H PG 3 ADDR 0x1E */
	SND_SOC_BYTES("Speaker MBC 3 Attack", R_SPKMBCATK3L, 3),
	/* R_SPKMBCREL3L PG 3 ADDR 0x1F */
	/* R_SPKMBCREL3H PG 3 ADDR 0x20 */
	SND_SOC_BYTES("Speaker MBC 3 Release", R_SPKMBCREL3L, 3),
	/* R_SPKCLECTL PG 3 ADDR 0x21 */
	SOC_ENUM("Speaker CLE Level Mode", spk_cle_lvl_mode_enum),
	SOC_ENUM("Speaker CLE Window", spk_cle_win_sel_enum),
	SOC_SINGLE("Speaker CLE Expander Playback Switch",
			R_SPKCLECTL, FB_SPKCLECTL_EXPEN, 1, 0),
	SOC_SINGLE("Speaker CLE Limiter Playback Switch",
			R_SPKCLECTL, FB_SPKCLECTL_LIMEN, 1, 0),
	SOC_SINGLE("Speaker CLE Compressor Playback Switch",
			R_SPKCLECTL, FB_SPKCLECTL_COMPEN, 1, 0),
	/* R_SPKCLEMUG PG 3 ADDR 0x22 */
	SOC_SINGLE_TLV("Speaker CLE Make-Up Gain", R_SPKCLEMUG,
			FB_SPKCLEMUG_MUGAIN, FM_SPKCLEMUG_MUGAIN,
			0, cle_mug_tlv_arr),
	/* R_SPKCOMPTHR PG 3 ADDR 0x23 */
	SOC_SINGLE_TLV("Speaker Compressor Threshold", R_SPKCOMPTHR,
			FB_SPKCOMPTHR_THRESH, FM_SPKCOMPTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SPKCOMPRAT PG 3 ADDR 0x24 */
	SOC_ENUM("Speaker Compressor Ratio", spk_comp_rat_enum),
	/* R_SPKCOMPATKL PG 3 ADDR 0x25 */
	/* R_SPKCOMPATKH PG 3 ADDR 0x26 */
	SND_SOC_BYTES("Speaker Compressor Attack", R_SPKCOMPATKL, 2),
	/* R_SPKCOMPRELL PG 3 ADDR 0x27 */
	/* R_SPKCOMPRELH PG 3 ADDR 0x28 */
	SND_SOC_BYTES("Speaker Compressor Release", R_SPKCOMPRELL, 2),
	/* R_SPKLIMTHR PG 3 ADDR 0x29 */
	SOC_SINGLE_TLV("Speaker Limiter Threshold", R_SPKLIMTHR,
			FB_SPKLIMTHR_THRESH, FM_SPKLIMTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SPKLIMTGT PG 3 ADDR 0x2A */
	SOC_SINGLE_TLV("Speaker Limiter Target", R_SPKLIMTGT,
			FB_SPKLIMTGT_TARGET, FM_SPKLIMTGT_TARGET,
			0, thr_tlv_arr),
	/* R_SPKLIMATKL PG 3 ADDR 0x2B */
	/* R_SPKLIMATKH PG 3 ADDR 0x2C */
	SND_SOC_BYTES("Speaker Limiter Attack", R_SPKLIMATKL, 2),
	/* R_SPKLIMRELL PG 3 ADDR 0x2D */
	/* R_SPKLIMRELR PG 3 ADDR 0x2E */
	SND_SOC_BYTES("Speaker Limiter Release", R_SPKLIMRELL, 2),
	/* R_SPKEXPTHR PG 3 ADDR 0x2F */
	SOC_SINGLE_TLV("Speaker Expander Threshold", R_SPKEXPTHR,
			FB_SPKEXPTHR_THRESH, FM_SPKEXPTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SPKEXPRAT PG 3 ADDR 0x30 */
	SOC_ENUM("Speaker Expander Ratio", spk_exp_rat_enum),
	/* R_SPKEXPATKL PG 3 ADDR 0x31 */
	/* R_SPKEXPATKR PG 3 ADDR 0x32 */
	SND_SOC_BYTES("Speaker Expander Attack", R_SPKEXPATKL, 2),
	/* R_SPKEXPRELL PG 3 ADDR 0x33 */
	/* R_SPKEXPRELR PG 3 ADDR 0x34 */
	SND_SOC_BYTES("Speaker Expander Release", R_SPKEXPRELL, 2),
	/* R_SPKFXCTL PG 3 ADDR 0x35 */
	SOC_SINGLE("Speaker 3D Switch", R_SPKFXCTL, FB_SPKFXCTL_3DEN, 1, 0),
	SOC_SINGLE("Speaker Treble Enhancement Switch",
			R_SPKFXCTL, FB_SPKFXCTL_TEEN, 1, 0),
	SOC_SINGLE("Speaker Treble NLF Switch",
			R_SPKFXCTL, FB_SPKFXCTL_TNLFBYP, 1, 1),
	SOC_SINGLE("Speaker Bass Enhancement Switch",
			R_SPKFXCTL, FB_SPKFXCTL_BEEN, 1, 0),
	SOC_SINGLE("Speaker Bass NLF Switch",
			R_SPKFXCTL, FB_SPKFXCTL_BNLFBYP, 1, 1),
	/*******
	 * DAC *
	 *******/
	/* R_DACEQFILT PG 4 ADDR 0x01 */
	SOC_SINGLE("DAC EQ 2 Switch",
			R_DACEQFILT, FB_DACEQFILT_EQ2EN, 1, 0),
	SOC_ENUM("DAC EQ 2 Band", dac_eq_enums[0]),
	SOC_SINGLE("DAC EQ 1 Switch", R_DACEQFILT, FB_DACEQFILT_EQ1EN, 1, 0),
	SOC_ENUM("DAC EQ 1 Band", dac_eq_enums[1]),
	/* R_DACMBCEN PG 4 ADDR 0x0A */
	SOC_SINGLE("DAC MBC 3 Switch", R_DACMBCEN, FB_DACMBCEN_MBCEN3, 1, 0),
	SOC_SINGLE("DAC MBC 2 Switch", R_DACMBCEN, FB_DACMBCEN_MBCEN2, 1, 0),
	SOC_SINGLE("DAC MBC 1 Switch", R_DACMBCEN, FB_DACMBCEN_MBCEN1, 1, 0),
	/* R_DACMBCCTL PG 4 ADDR 0x0B */
	SOC_ENUM("DAC MBC 3 Mode", dac_mbc3_lvl_det_mode_enum),
	SOC_ENUM("DAC MBC 3 Window", dac_mbc3_win_sel_enum),
	SOC_ENUM("DAC MBC 2 Mode", dac_mbc2_lvl_det_mode_enum),
	SOC_ENUM("DAC MBC 2 Window", dac_mbc2_win_sel_enum),
	SOC_ENUM("DAC MBC 1 Mode", dac_mbc1_lvl_det_mode_enum),
	SOC_ENUM("DAC MBC 1 Window", dac_mbc1_win_sel_enum),
	/* R_DACMBCMUG1 PG 4 ADDR 0x0C */
	SOC_ENUM("DAC MBC 1 Phase Polarity", dac_mbc1_phase_pol_enum),
	SOC_SINGLE_TLV("DAC MBC 1 Make-Up Gain", R_DACMBCMUG1,
			FB_DACMBCMUG_MUGAIN, FM_DACMBCMUG_MUGAIN,
			0, mbc_mug_tlv_arr),
	/* R_DACMBCTHR1 PG 4 ADDR 0x0D */
	SOC_SINGLE_TLV("DAC MBC 1 Compressor Threshold", R_DACMBCTHR1,
			FB_DACMBCTHR_THRESH, FM_DACMBCTHR_THRESH,
			0, thr_tlv_arr),
	/* R_DACMBCRAT1 PG 4 ADDR 0x0E */
	SOC_ENUM("DAC MBC 1 Compressor Ratio", dac_mbc1_comp_rat_enum),
	/* R_DACMBCATK1L PG 4 ADDR 0x0F */
	/* R_DACMBCATK1H PG 4 ADDR 0x10 */
	SND_SOC_BYTES("DAC MBC 1 Attack", R_DACMBCATK1L, 2),
	/* R_DACMBCREL1L PG 4 ADDR 0x11 */
	/* R_DACMBCREL1H PG 4 ADDR 0x12 */
	SND_SOC_BYTES("DAC MBC 1 Release", R_DACMBCREL1L, 2),
	/* R_DACMBCMUG2 PG 4 ADDR 0x13 */
	SOC_ENUM("DAC MBC 2 Phase Polarity", dac_mbc2_phase_pol_enum),
	SOC_SINGLE_TLV("DAC MBC 2 Make-Up Gain", R_DACMBCMUG2,
			FB_DACMBCMUG_MUGAIN, FM_DACMBCMUG_MUGAIN,
			0, mbc_mug_tlv_arr),
	/* R_DACMBCTHR2 PG 4 ADDR 0x14 */
	SOC_SINGLE_TLV("DAC MBC 2 Compressor Threshold", R_DACMBCTHR2,
			FB_DACMBCTHR_THRESH, FM_DACMBCTHR_THRESH,
			0, thr_tlv_arr),
	/* R_DACMBCRAT2 PG 4 ADDR 0x15 */
	SOC_ENUM("DAC MBC 2 Compressor Ratio", dac_mbc2_comp_rat_enum),
	/* R_DACMBCATK2L PG 4 ADDR 0x16 */
	/* R_DACMBCATK2H PG 4 ADDR 0x17 */
	SND_SOC_BYTES("DAC MBC 2 Attack", R_DACMBCATK2L, 2),
	/* R_DACMBCREL2L PG 4 ADDR 0x18 */
	/* R_DACMBCREL2H PG 4 ADDR 0x19 */
	SND_SOC_BYTES("DAC MBC 2 Release", R_DACMBCREL2L, 2),
	/* R_DACMBCMUG3 PG 4 ADDR 0x1A */
	SOC_ENUM("DAC MBC 3 Phase Polarity", dac_mbc3_phase_pol_enum),
	SOC_SINGLE_TLV("DAC MBC 3 Make-Up Gain", R_DACMBCMUG3,
			FB_DACMBCMUG_MUGAIN, FM_DACMBCMUG_MUGAIN,
			0, mbc_mug_tlv_arr),
	/* R_DACMBCTHR3 PG 4 ADDR 0x1B */
	SOC_SINGLE_TLV("DAC MBC 3 Threshold", R_DACMBCTHR3,
			FB_DACMBCTHR_THRESH, FM_DACMBCTHR_THRESH,
			0, thr_tlv_arr),
	/* R_DACMBCRAT3 PG 4 ADDR 0x1C */
	SOC_ENUM("DAC MBC 3 Compressor Ratio", dac_mbc3_comp_rat_enum),
	/* R_DACMBCATK3L PG 4 ADDR 0x1D */
	/* R_DACMBCATK3H PG 4 ADDR 0x1E */
	SND_SOC_BYTES("DAC MBC 3 Attack", R_DACMBCATK3L, 3),
	/* R_DACMBCREL3L PG 4 ADDR 0x1F */
	/* R_DACMBCREL3H PG 4 ADDR 0x20 */
	SND_SOC_BYTES("DAC MBC 3 Release", R_DACMBCREL3L, 3),
	/* R_DACCLECTL PG 4 ADDR 0x21 */
	SOC_ENUM("DAC CLE Level Mode", dac_cle_lvl_mode_enum),
	SOC_ENUM("DAC CLE Window", dac_cle_win_sel_enum),
	SOC_SINGLE("DAC CLE Expander Playback Switch",
			R_DACCLECTL, FB_DACCLECTL_EXPEN, 1, 0),
	SOC_SINGLE("DAC CLE Limiter Playback Switch",
			R_DACCLECTL, FB_DACCLECTL_LIMEN, 1, 0),
	SOC_SINGLE("DAC CLE Compressor Playback Switch",
			R_DACCLECTL, FB_DACCLECTL_COMPEN, 1, 0),
	/* R_DACCLEMUG PG 4 ADDR 0x22 */
	SOC_SINGLE_TLV("DAC CLE Make-Up Gain", R_DACCLEMUG,
			FB_DACCLEMUG_MUGAIN, FM_DACCLEMUG_MUGAIN,
			0, cle_mug_tlv_arr),
	/* R_DACCOMPTHR PG 4 ADDR 0x23 */
	SOC_SINGLE_TLV("DAC Compressor Threshold", R_DACCOMPTHR,
			FB_DACCOMPTHR_THRESH, FM_DACCOMPTHR_THRESH,
			0, thr_tlv_arr),
	/* R_DACCOMPRAT PG 4 ADDR 0x24 */
	SOC_ENUM("DAC Compressor Ratio", dac_comp_rat_enum),
	/* R_DACCOMPATKL PG 4 ADDR 0x25 */
	/* R_DACCOMPATKH PG 4 ADDR 0x26 */
	SND_SOC_BYTES("DAC Compressor Attack", R_DACCOMPATKL, 2),
	/* R_DACCOMPRELL PG 4 ADDR 0x27 */
	/* R_DACCOMPRELH PG 4 ADDR 0x28 */
	SND_SOC_BYTES("DAC Compressor Release", R_DACCOMPRELL, 2),
	/* R_DACLIMTHR PG 4 ADDR 0x29 */
	SOC_SINGLE_TLV("DAC Limiter Threshold", R_DACLIMTHR,
			FB_DACLIMTHR_THRESH, FM_DACLIMTHR_THRESH,
			0, thr_tlv_arr),
	/* R_DACLIMTGT PG 4 ADDR 0x2A */
	SOC_SINGLE_TLV("DAC Limiter Target", R_DACLIMTGT,
			FB_DACLIMTGT_TARGET, FM_DACLIMTGT_TARGET,
			0, thr_tlv_arr),
	/* R_DACLIMATKL PG 4 ADDR 0x2B */
	/* R_DACLIMATKH PG 4 ADDR 0x2C */
	SND_SOC_BYTES("DAC Limiter Attack", R_DACLIMATKL, 2),
	/* R_DACLIMRELL PG 4 ADDR 0x2D */
	/* R_DACLIMRELR PG 4 ADDR 0x2E */
	SND_SOC_BYTES("DAC Limiter Release", R_DACLIMRELL, 2),
	/* R_DACEXPTHR PG 4 ADDR 0x2F */
	SOC_SINGLE_TLV("DAC Expander Threshold", R_DACEXPTHR,
			FB_DACEXPTHR_THRESH, FM_DACEXPTHR_THRESH,
			0, thr_tlv_arr),
	/* R_DACEXPRAT PG 4 ADDR 0x30 */
	SOC_ENUM("DAC Expander Ratio", dac_exp_rat_enum),
	/* R_DACEXPATKL PG 4 ADDR 0x31 */
	/* R_DACEXPATKR PG 4 ADDR 0x32 */
	SND_SOC_BYTES("DAC Expander Attack", R_DACEXPATKL, 2),
	/* R_DACEXPRELL PG 4 ADDR 0x33 */
	/* R_DACEXPRELR PG 4 ADDR 0x34 */
	SND_SOC_BYTES("DAC Expander Release", R_DACEXPRELL, 2),
	/* R_DACFXCTL PG 4 ADDR 0x35 */
	SOC_SINGLE("DAC 3D Switch", R_DACFXCTL, FB_DACFXCTL_3DEN, 1, 0),
	SOC_SINGLE("DAC Treble Enhancement Switch",
			R_DACFXCTL, FB_DACFXCTL_TEEN, 1, 0),
	SOC_SINGLE("DAC Treble NLF Switch",
			R_DACFXCTL, FB_DACFXCTL_TNLFBYP, 1, 1),
	SOC_SINGLE("DAC Bass Enhancement Switch",
			R_DACFXCTL, FB_DACFXCTL_BEEN, 1, 0),
	SOC_SINGLE("DAC Bass NLF Switch",
			R_DACFXCTL, FB_DACFXCTL_BNLFBYP, 1, 1),
	/*******
	 * Sub *
	 *******/
	/* R_SUBEQFILT PG 5 ADDR 0x01 */
	SOC_SINGLE("Sub EQ 2 Switch",
			R_SUBEQFILT, FB_SUBEQFILT_EQ2EN, 1, 0),
	SOC_ENUM("Sub EQ 2 Band", sub_eq_enums[0]),
	SOC_SINGLE("Sub EQ 1 Switch", R_SUBEQFILT, FB_SUBEQFILT_EQ1EN, 1, 0),
	SOC_ENUM("Sub EQ 1 Band", sub_eq_enums[1]),
	/* R_SUBMBCEN PG 5 ADDR 0x0A */
	SOC_SINGLE("Sub MBC 3 Switch", R_SUBMBCEN, FB_SUBMBCEN_MBCEN3, 1, 0),
	SOC_SINGLE("Sub MBC 2 Switch", R_SUBMBCEN, FB_SUBMBCEN_MBCEN2, 1, 0),
	SOC_SINGLE("Sub MBC 1 Switch", R_SUBMBCEN, FB_SUBMBCEN_MBCEN1, 1, 0),
	/* R_SUBMBCCTL PG 5 ADDR 0x0B */
	SOC_ENUM("Sub MBC 3 Mode", sub_mbc3_lvl_det_mode_enum),
	SOC_ENUM("Sub MBC 3 Window", sub_mbc3_win_sel_enum),
	SOC_ENUM("Sub MBC 2 Mode", sub_mbc2_lvl_det_mode_enum),
	SOC_ENUM("Sub MBC 2 Window", sub_mbc2_win_sel_enum),
	SOC_ENUM("Sub MBC 1 Mode", sub_mbc1_lvl_det_mode_enum),
	SOC_ENUM("Sub MBC 1 Window", sub_mbc1_win_sel_enum),
	/* R_SUBMBCMUG1 PG 5 ADDR 0x0C */
	SOC_ENUM("Sub MBC 1 Phase Polarity", sub_mbc1_phase_pol_enum),
	SOC_SINGLE_TLV("Sub MBC 1 Make-Up Gain", R_SUBMBCMUG1,
			FB_SUBMBCMUG_MUGAIN, FM_SUBMBCMUG_MUGAIN,
			0, mbc_mug_tlv_arr),
	/* R_SUBMBCTHR1 PG 5 ADDR 0x0D */
	SOC_SINGLE_TLV("Sub MBC 1 Compressor Threshold", R_SUBMBCTHR1,
			FB_SUBMBCTHR_THRESH, FM_SUBMBCTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SUBMBCRAT1 PG 5 ADDR 0x0E */
	SOC_ENUM("Sub MBC 1 Compressor Ratio", sub_mbc1_comp_rat_enum),
	/* R_SUBMBCATK1L PG 5 ADDR 0x0F */
	/* R_SUBMBCATK1H PG 5 ADDR 0x10 */
	SND_SOC_BYTES("Sub MBC 1 Attack", R_SUBMBCATK1L, 2),
	/* R_SUBMBCREL1L PG 5 ADDR 0x11 */
	/* R_SUBMBCREL1H PG 5 ADDR 0x12 */
	SND_SOC_BYTES("Sub MBC 1 Release", R_SUBMBCREL1L, 2),
	/* R_SUBMBCMUG2 PG 5 ADDR 0x13 */
	SOC_ENUM("Sub MBC 2 Phase Polarity", sub_mbc2_phase_pol_enum),
	SOC_SINGLE_TLV("Sub MBC 2 Make-Up Gain", R_SUBMBCMUG2,
			FB_SUBMBCMUG_MUGAIN, FM_SUBMBCMUG_MUGAIN,
			0, mbc_mug_tlv_arr),
	/* R_SUBMBCTHR2 PG 5 ADDR 0x14 */
	SOC_SINGLE_TLV("Sub MBC 2 Compressor Threshold", R_SUBMBCTHR2,
			FB_SUBMBCTHR_THRESH, FM_SUBMBCTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SUBMBCRAT2 PG 5 ADDR 0x15 */
	SOC_ENUM("Sub MBC 2 Compressor Ratio", sub_mbc2_comp_rat_enum),
	/* R_SUBMBCATK2L PG 5 ADDR 0x16 */
	/* R_SUBMBCATK2H PG 5 ADDR 0x17 */
	SND_SOC_BYTES("Sub MBC 2 Attack", R_SUBMBCATK2L, 2),
	/* R_SUBMBCREL2L PG 5 ADDR 0x18 */
	/* R_SUBMBCREL2H PG 5 ADDR 0x19 */
	SND_SOC_BYTES("Sub MBC 2 Release", R_SUBMBCREL2L, 2),
	/* R_SUBMBCMUG3 PG 5 ADDR 0x1A */
	SOC_ENUM("Sub MBC 3 Phase Polarity", sub_mbc3_phase_pol_enum),
	SOC_SINGLE_TLV("Sub MBC 3 Make-Up Gain", R_SUBMBCMUG3,
			FB_SUBMBCMUG_MUGAIN, FM_SUBMBCMUG_MUGAIN,
			0, mbc_mug_tlv_arr),
	/* R_SUBMBCTHR3 PG 5 ADDR 0x1B */
	SOC_SINGLE_TLV("Sub MBC 3 Threshold", R_SUBMBCTHR3,
			FB_SUBMBCTHR_THRESH, FM_SUBMBCTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SUBMBCRAT3 PG 5 ADDR 0x1C */
	SOC_ENUM("Sub MBC 3 Compressor Ratio", sub_mbc3_comp_rat_enum),
	/* R_SUBMBCATK3L PG 5 ADDR 0x1D */
	/* R_SUBMBCATK3H PG 5 ADDR 0x1E */
	SND_SOC_BYTES("Sub MBC 3 Attack", R_SUBMBCATK3L, 3),
	/* R_SUBMBCREL3L PG 5 ADDR 0x1F */
	/* R_SUBMBCREL3H PG 5 ADDR 0x20 */
	SND_SOC_BYTES("Sub MBC 3 Release", R_SUBMBCREL3L, 3),
	/* R_SUBCLECTL PG 5 ADDR 0x21 */
	SOC_ENUM("Sub CLE Level Mode", sub_cle_lvl_mode_enum),
	SOC_ENUM("Sub CLE Window", sub_cle_win_sel_enum),
	SOC_SINGLE("Sub CLE Expander Playback Switch",
			R_SUBCLECTL, FB_SUBCLECTL_EXPEN, 1, 0),
	SOC_SINGLE("Sub CLE Limiter Playback Switch",
			R_SUBCLECTL, FB_SUBCLECTL_LIMEN, 1, 0),
	SOC_SINGLE("Sub CLE Compressor Playback Switch",
			R_SUBCLECTL, FB_SUBCLECTL_COMPEN, 1, 0),
	/* R_SUBCLEMUG PG 5 ADDR 0x22 */
	SOC_SINGLE_TLV("Sub CLE Make-Up Gain", R_SUBCLEMUG,
			FB_SUBCLEMUG_MUGAIN, FM_SUBCLEMUG_MUGAIN,
			0, cle_mug_tlv_arr),
	/* R_SUBCOMPTHR PG 5 ADDR 0x23 */
	SOC_SINGLE_TLV("Sub Compressor Threshold", R_SUBCOMPTHR,
			FB_SUBCOMPTHR_THRESH, FM_SUBCOMPTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SUBCOMPRAT PG 5 ADDR 0x24 */
	SOC_ENUM("Sub Compressor Ratio", sub_comp_rat_enum),
	/* R_SUBCOMPATKL PG 5 ADDR 0x25 */
	/* R_SUBCOMPATKH PG 5 ADDR 0x26 */
	SND_SOC_BYTES("Sub Compressor Attack", R_SUBCOMPATKL, 2),
	/* R_SUBCOMPRELL PG 5 ADDR 0x27 */
	/* R_SUBCOMPRELH PG 5 ADDR 0x28 */
	SND_SOC_BYTES("Sub Compressor Release", R_SUBCOMPRELL, 2),
	/* R_SUBLIMTHR PG 5 ADDR 0x29 */
	SOC_SINGLE_TLV("Sub Limiter Threshold", R_SUBLIMTHR,
			FB_SUBLIMTHR_THRESH, FM_SUBLIMTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SUBLIMTGT PG 5 ADDR 0x2A */
	SOC_SINGLE_TLV("Sub Limiter Target", R_SUBLIMTGT,
			FB_SUBLIMTGT_TARGET, FM_SUBLIMTGT_TARGET,
			0, thr_tlv_arr),
	/* R_SUBLIMATKL PG 5 ADDR 0x2B */
	/* R_SUBLIMATKH PG 5 ADDR 0x2C */
	SND_SOC_BYTES("Sub Limiter Attack", R_SUBLIMATKL, 2),
	/* R_SUBLIMRELL PG 5 ADDR 0x2D */
	/* R_SUBLIMRELR PG 5 ADDR 0x2E */
	SND_SOC_BYTES("Sub Limiter Release", R_SUBLIMRELL, 2),
	/* R_SUBEXPTHR PG 5 ADDR 0x2F */
	SOC_SINGLE_TLV("Sub Expander Threshold", R_SUBEXPTHR,
			FB_SUBEXPTHR_THRESH, FM_SUBEXPTHR_THRESH,
			0, thr_tlv_arr),
	/* R_SUBEXPRAT PG 5 ADDR 0x30 */
	SOC_ENUM("Sub Expander Ratio", sub_exp_rat_enum),
	/* R_SUBEXPATKL PG 5 ADDR 0x31 */
	/* R_SUBEXPATKR PG 5 ADDR 0x32 */
	SND_SOC_BYTES("Sub Expander Attack", R_SUBEXPATKL, 2),
	/* R_SUBEXPRELL PG 5 ADDR 0x33 */
	/* R_SUBEXPRELR PG 5 ADDR 0x34 */
	SND_SOC_BYTES("Sub Expander Release", R_SUBEXPRELL, 2),
	/* R_SUBFXCTL PG 5 ADDR 0x35 */
	SOC_SINGLE("Sub Treble Enhancement Switch",
			R_SUBFXCTL, FB_SUBFXCTL_TEEN, 1, 0),
	SOC_SINGLE("Sub Treble NLF Switch",
			R_SUBFXCTL, FB_SUBFXCTL_TNLFBYP, 1, 1),
	SOC_SINGLE("Sub Bass Enhancement Switch",
			R_SUBFXCTL, FB_SUBFXCTL_BEEN, 1, 0),
	SOC_SINGLE("Sub Bass NLF Switch",
			R_SUBFXCTL, FB_SUBFXCTL_BNLFBYP, 1, 1),
	/*******************
	 * Coefficient RAM *
	 *******************/
	/* DAC */
	COEFF_RAM_CTL("DAC Cascade 1 Left BiQuad 1", BIQUAD_SIZE, 0x00),
	COEFF_RAM_CTL("DAC Cascade 1 Left BiQuad 2", BIQUAD_SIZE, 0x05),
	COEFF_RAM_CTL("DAC Cascade 1 Left BiQuad 3", BIQUAD_SIZE, 0x0a),
	COEFF_RAM_CTL("DAC Cascade 1 Left BiQuad 4", BIQUAD_SIZE, 0x0f),
	COEFF_RAM_CTL("DAC Cascade 1 Left BiQuad 5", BIQUAD_SIZE, 0x14),
	COEFF_RAM_CTL("DAC Cascade 1 Left BiQuad 6", BIQUAD_SIZE, 0x19),

	COEFF_RAM_CTL("DAC Cascade 1 Right BiQuad 1", BIQUAD_SIZE, 0x20),
	COEFF_RAM_CTL("DAC Cascade 1 Right BiQuad 2", BIQUAD_SIZE, 0x25),
	COEFF_RAM_CTL("DAC Cascade 1 Right BiQuad 3", BIQUAD_SIZE, 0x2a),
	COEFF_RAM_CTL("DAC Cascade 1 Right BiQuad 4", BIQUAD_SIZE, 0x2f),
	COEFF_RAM_CTL("DAC Cascade 1 Right BiQuad 5", BIQUAD_SIZE, 0x34),
	COEFF_RAM_CTL("DAC Cascade 1 Right BiQuad 6", BIQUAD_SIZE, 0x39),

	COEFF_RAM_CTL("DAC Cascade 1 Left Prescale", COEFF_SIZE, 0x1f),
	COEFF_RAM_CTL("DAC Cascade 1 Right Prescale", COEFF_SIZE, 0x3f),

	COEFF_RAM_CTL("DAC Cascade 2 Left BiQuad 1", BIQUAD_SIZE, 0x40),
	COEFF_RAM_CTL("DAC Cascade 2 Left BiQuad 2", BIQUAD_SIZE, 0x45),
	COEFF_RAM_CTL("DAC Cascade 2 Left BiQuad 3", BIQUAD_SIZE, 0x4a),
	COEFF_RAM_CTL("DAC Cascade 2 Left BiQuad 4", BIQUAD_SIZE, 0x4f),
	COEFF_RAM_CTL("DAC Cascade 2 Left BiQuad 5", BIQUAD_SIZE, 0x54),
	COEFF_RAM_CTL("DAC Cascade 2 Left BiQuad 6", BIQUAD_SIZE, 0x59),

	COEFF_RAM_CTL("DAC Cascade 2 Right BiQuad 1", BIQUAD_SIZE, 0x60),
	COEFF_RAM_CTL("DAC Cascade 2 Right BiQuad 2", BIQUAD_SIZE, 0x65),
	COEFF_RAM_CTL("DAC Cascade 2 Right BiQuad 3", BIQUAD_SIZE, 0x6a),
	COEFF_RAM_CTL("DAC Cascade 2 Right BiQuad 4", BIQUAD_SIZE, 0x6f),
	COEFF_RAM_CTL("DAC Cascade 2 Right BiQuad 5", BIQUAD_SIZE, 0x74),
	COEFF_RAM_CTL("DAC Cascade 2 Right BiQuad 6", BIQUAD_SIZE, 0x79),

	COEFF_RAM_CTL("DAC Cascade 2 Left Prescale", COEFF_SIZE, 0x5f),
	COEFF_RAM_CTL("DAC Cascade 2 Right Prescale", COEFF_SIZE, 0x7f),

	COEFF_RAM_CTL("DAC Bass Extraction BiQuad 1", BIQUAD_SIZE, 0x80),
	COEFF_RAM_CTL("DAC Bass Extraction BiQuad 2", BIQUAD_SIZE, 0x85),

	COEFF_RAM_CTL("DAC Bass Non Linear Function 1", COEFF_SIZE, 0x8a),
	COEFF_RAM_CTL("DAC Bass Non Linear Function 2", COEFF_SIZE, 0x8b),

	COEFF_RAM_CTL("DAC Bass Limiter BiQuad", BIQUAD_SIZE, 0x8c),

	COEFF_RAM_CTL("DAC Bass Cut Off BiQuad", BIQUAD_SIZE, 0x91),

	COEFF_RAM_CTL("DAC Bass Mix", COEFF_SIZE, 0x96),

	COEFF_RAM_CTL("DAC Treb Extraction BiQuad 1", BIQUAD_SIZE, 0x97),
	COEFF_RAM_CTL("DAC Treb Extraction BiQuad 2", BIQUAD_SIZE, 0x9c),

	COEFF_RAM_CTL("DAC Treb Non Linear Function 1", COEFF_SIZE, 0xa1),
	COEFF_RAM_CTL("DAC Treb Non Linear Function 2", COEFF_SIZE, 0xa2),

	COEFF_RAM_CTL("DAC Treb Limiter BiQuad", BIQUAD_SIZE, 0xa3),

	COEFF_RAM_CTL("DAC Treb Cut Off BiQuad", BIQUAD_SIZE, 0xa8),

	COEFF_RAM_CTL("DAC Treb Mix", COEFF_SIZE, 0xad),

	COEFF_RAM_CTL("DAC 3D", COEFF_SIZE, 0xae),

	COEFF_RAM_CTL("DAC 3D Mix", COEFF_SIZE, 0xaf),

	COEFF_RAM_CTL("DAC MBC 1 BiQuad 1", BIQUAD_SIZE, 0xb0),
	COEFF_RAM_CTL("DAC MBC 1 BiQuad 2", BIQUAD_SIZE, 0xb5),

	COEFF_RAM_CTL("DAC MBC 2 BiQuad 1", BIQUAD_SIZE, 0xba),
	COEFF_RAM_CTL("DAC MBC 2 BiQuad 2", BIQUAD_SIZE, 0xbf),

	COEFF_RAM_CTL("DAC MBC 3 BiQuad 1", BIQUAD_SIZE, 0xc4),
	COEFF_RAM_CTL("DAC MBC 3 BiQuad 2", BIQUAD_SIZE, 0xc9),
	/* Speaker */
	COEFF_RAM_CTL("Speaker Cascade 1 Left BiQuad 1", BIQUAD_SIZE, 0x00),
	COEFF_RAM_CTL("Speaker Cascade 1 Left BiQuad 2", BIQUAD_SIZE, 0x05),
	COEFF_RAM_CTL("Speaker Cascade 1 Left BiQuad 3", BIQUAD_SIZE, 0x0a),
	COEFF_RAM_CTL("Speaker Cascade 1 Left BiQuad 4", BIQUAD_SIZE, 0x0f),
	COEFF_RAM_CTL("Speaker Cascade 1 Left BiQuad 5", BIQUAD_SIZE, 0x14),
	COEFF_RAM_CTL("Speaker Cascade 1 Left BiQuad 6", BIQUAD_SIZE, 0x19),

	COEFF_RAM_CTL("Speaker Cascade 1 Right BiQuad 1", BIQUAD_SIZE, 0x20),
	COEFF_RAM_CTL("Speaker Cascade 1 Right BiQuad 2", BIQUAD_SIZE, 0x25),
	COEFF_RAM_CTL("Speaker Cascade 1 Right BiQuad 3", BIQUAD_SIZE, 0x2a),
	COEFF_RAM_CTL("Speaker Cascade 1 Right BiQuad 4", BIQUAD_SIZE, 0x2f),
	COEFF_RAM_CTL("Speaker Cascade 1 Right BiQuad 5", BIQUAD_SIZE, 0x34),
	COEFF_RAM_CTL("Speaker Cascade 1 Right BiQuad 6", BIQUAD_SIZE, 0x39),

	COEFF_RAM_CTL("Speaker Cascade 1 Left Prescale", COEFF_SIZE, 0x1f),
	COEFF_RAM_CTL("Speaker Cascade 1 Right Prescale", COEFF_SIZE, 0x3f),

	COEFF_RAM_CTL("Speaker Cascade 2 Left BiQuad 1", BIQUAD_SIZE, 0x40),
	COEFF_RAM_CTL("Speaker Cascade 2 Left BiQuad 2", BIQUAD_SIZE, 0x45),
	COEFF_RAM_CTL("Speaker Cascade 2 Left BiQuad 3", BIQUAD_SIZE, 0x4a),
	COEFF_RAM_CTL("Speaker Cascade 2 Left BiQuad 4", BIQUAD_SIZE, 0x4f),
	COEFF_RAM_CTL("Speaker Cascade 2 Left BiQuad 5", BIQUAD_SIZE, 0x54),
	COEFF_RAM_CTL("Speaker Cascade 2 Left BiQuad 6", BIQUAD_SIZE, 0x59),

	COEFF_RAM_CTL("Speaker Cascade 2 Right BiQuad 1", BIQUAD_SIZE, 0x60),
	COEFF_RAM_CTL("Speaker Cascade 2 Right BiQuad 2", BIQUAD_SIZE, 0x65),
	COEFF_RAM_CTL("Speaker Cascade 2 Right BiQuad 3", BIQUAD_SIZE, 0x6a),
	COEFF_RAM_CTL("Speaker Cascade 2 Right BiQuad 4", BIQUAD_SIZE, 0x6f),
	COEFF_RAM_CTL("Speaker Cascade 2 Right BiQuad 5", BIQUAD_SIZE, 0x74),
	COEFF_RAM_CTL("Speaker Cascade 2 Right BiQuad 6", BIQUAD_SIZE, 0x79),

	COEFF_RAM_CTL("Speaker Cascade 2 Left Prescale", COEFF_SIZE, 0x5f),
	COEFF_RAM_CTL("Speaker Cascade 2 Right Prescale", COEFF_SIZE, 0x7f),

	COEFF_RAM_CTL("Speaker Bass Extraction BiQuad 1", BIQUAD_SIZE, 0x80),
	COEFF_RAM_CTL("Speaker Bass Extraction BiQuad 2", BIQUAD_SIZE, 0x85),

	COEFF_RAM_CTL("Speaker Bass Non Linear Function 1", COEFF_SIZE, 0x8a),
	COEFF_RAM_CTL("Speaker Bass Non Linear Function 2", COEFF_SIZE, 0x8b),

	COEFF_RAM_CTL("Speaker Bass Limiter BiQuad", BIQUAD_SIZE, 0x8c),

	COEFF_RAM_CTL("Speaker Bass Cut Off BiQuad", BIQUAD_SIZE, 0x91),

	COEFF_RAM_CTL("Speaker Bass Mix", COEFF_SIZE, 0x96),

	COEFF_RAM_CTL("Speaker Treb Extraction BiQuad 1", BIQUAD_SIZE, 0x97),
	COEFF_RAM_CTL("Speaker Treb Extraction BiQuad 2", BIQUAD_SIZE, 0x9c),

	COEFF_RAM_CTL("Speaker Treb Non Linear Function 1", COEFF_SIZE, 0xa1),
	COEFF_RAM_CTL("Speaker Treb Non Linear Function 2", COEFF_SIZE, 0xa2),

	COEFF_RAM_CTL("Speaker Treb Limiter BiQuad", BIQUAD_SIZE, 0xa3),

	COEFF_RAM_CTL("Speaker Treb Cut Off BiQuad", BIQUAD_SIZE, 0xa8),

	COEFF_RAM_CTL("Speaker Treb Mix", COEFF_SIZE, 0xad),

	COEFF_RAM_CTL("Speaker 3D", COEFF_SIZE, 0xae),

	COEFF_RAM_CTL("Speaker 3D Mix", COEFF_SIZE, 0xaf),

	COEFF_RAM_CTL("Speaker MBC 1 BiQuad 1", BIQUAD_SIZE, 0xb0),
	COEFF_RAM_CTL("Speaker MBC 1 BiQuad 2", BIQUAD_SIZE, 0xb5),

	COEFF_RAM_CTL("Speaker MBC 2 BiQuad 1", BIQUAD_SIZE, 0xba),
	COEFF_RAM_CTL("Speaker MBC 2 BiQuad 2", BIQUAD_SIZE, 0xbf),

	COEFF_RAM_CTL("Speaker MBC 3 BiQuad 1", BIQUAD_SIZE, 0xc4),
	COEFF_RAM_CTL("Speaker MBC 3 BiQuad 2", BIQUAD_SIZE, 0xc9),
	/* Sub */
	COEFF_RAM_CTL("Sub Cascade 1 Left BiQuad 1", BIQUAD_SIZE, 0x00),
	COEFF_RAM_CTL("Sub Cascade 1 Left BiQuad 2", BIQUAD_SIZE, 0x05),
	COEFF_RAM_CTL("Sub Cascade 1 Left BiQuad 3", BIQUAD_SIZE, 0x0a),
	COEFF_RAM_CTL("Sub Cascade 1 Left BiQuad 4", BIQUAD_SIZE, 0x0f),
	COEFF_RAM_CTL("Sub Cascade 1 Left BiQuad 5", BIQUAD_SIZE, 0x14),
	COEFF_RAM_CTL("Sub Cascade 1 Left BiQuad 6", BIQUAD_SIZE, 0x19),

	COEFF_RAM_CTL("Sub Cascade 1 Right BiQuad 1", BIQUAD_SIZE, 0x20),
	COEFF_RAM_CTL("Sub Cascade 1 Right BiQuad 2", BIQUAD_SIZE, 0x25),
	COEFF_RAM_CTL("Sub Cascade 1 Right BiQuad 3", BIQUAD_SIZE, 0x2a),
	COEFF_RAM_CTL("Sub Cascade 1 Right BiQuad 4", BIQUAD_SIZE, 0x2f),
	COEFF_RAM_CTL("Sub Cascade 1 Right BiQuad 5", BIQUAD_SIZE, 0x34),
	COEFF_RAM_CTL("Sub Cascade 1 Right BiQuad 6", BIQUAD_SIZE, 0x39),

	COEFF_RAM_CTL("Sub Cascade 1 Left Prescale", COEFF_SIZE, 0x1f),
	COEFF_RAM_CTL("Sub Cascade 1 Right Prescale", COEFF_SIZE, 0x3f),

	COEFF_RAM_CTL("Sub Cascade 2 Left BiQuad 1", BIQUAD_SIZE, 0x40),
	COEFF_RAM_CTL("Sub Cascade 2 Left BiQuad 2", BIQUAD_SIZE, 0x45),
	COEFF_RAM_CTL("Sub Cascade 2 Left BiQuad 3", BIQUAD_SIZE, 0x4a),
	COEFF_RAM_CTL("Sub Cascade 2 Left BiQuad 4", BIQUAD_SIZE, 0x4f),
	COEFF_RAM_CTL("Sub Cascade 2 Left BiQuad 5", BIQUAD_SIZE, 0x54),
	COEFF_RAM_CTL("Sub Cascade 2 Left BiQuad 6", BIQUAD_SIZE, 0x59),

	COEFF_RAM_CTL("Sub Cascade 2 Right BiQuad 1", BIQUAD_SIZE, 0x60),
	COEFF_RAM_CTL("Sub Cascade 2 Right BiQuad 2", BIQUAD_SIZE, 0x65),
	COEFF_RAM_CTL("Sub Cascade 2 Right BiQuad 3", BIQUAD_SIZE, 0x6a),
	COEFF_RAM_CTL("Sub Cascade 2 Right BiQuad 4", BIQUAD_SIZE, 0x6f),
	COEFF_RAM_CTL("Sub Cascade 2 Right BiQuad 5", BIQUAD_SIZE, 0x74),
	COEFF_RAM_CTL("Sub Cascade 2 Right BiQuad 6", BIQUAD_SIZE, 0x79),

	COEFF_RAM_CTL("Sub Cascade 2 Left Prescale", COEFF_SIZE, 0x5f),
	COEFF_RAM_CTL("Sub Cascade 2 Right Prescale", COEFF_SIZE, 0x7f),

	COEFF_RAM_CTL("Sub Bass Extraction BiQuad 1", BIQUAD_SIZE, 0x80),
	COEFF_RAM_CTL("Sub Bass Extraction BiQuad 2", BIQUAD_SIZE, 0x85),

	COEFF_RAM_CTL("Sub Bass Non Linear Function 1", COEFF_SIZE, 0x8a),
	COEFF_RAM_CTL("Sub Bass Non Linear Function 2", COEFF_SIZE, 0x8b),

	COEFF_RAM_CTL("Sub Bass Limiter BiQuad", BIQUAD_SIZE, 0x8c),

	COEFF_RAM_CTL("Sub Bass Cut Off BiQuad", BIQUAD_SIZE, 0x91),

	COEFF_RAM_CTL("Sub Bass Mix", COEFF_SIZE, 0x96),

	COEFF_RAM_CTL("Sub Treb Extraction BiQuad 1", BIQUAD_SIZE, 0x97),
	COEFF_RAM_CTL("Sub Treb Extraction BiQuad 2", BIQUAD_SIZE, 0x9c),

	COEFF_RAM_CTL("Sub Treb Non Linear Function 1", COEFF_SIZE, 0xa1),
	COEFF_RAM_CTL("Sub Treb Non Linear Function 2", COEFF_SIZE, 0xa2),

	COEFF_RAM_CTL("Sub Treb Limiter BiQuad", BIQUAD_SIZE, 0xa3),

	COEFF_RAM_CTL("Sub Treb Cut Off BiQuad", BIQUAD_SIZE, 0xa8),

	COEFF_RAM_CTL("Sub Treb Mix", COEFF_SIZE, 0xad),

	COEFF_RAM_CTL("Sub 3D", COEFF_SIZE, 0xae),

	COEFF_RAM_CTL("Sub 3D Mix", COEFF_SIZE, 0xaf),

	COEFF_RAM_CTL("Sub MBC 1 BiQuad 1", BIQUAD_SIZE, 0xb0),
	COEFF_RAM_CTL("Sub MBC 1 BiQuad 2", BIQUAD_SIZE, 0xb5),

	COEFF_RAM_CTL("Sub MBC 2 BiQuad 1", BIQUAD_SIZE, 0xba),
	COEFF_RAM_CTL("Sub MBC 2 BiQuad 2", BIQUAD_SIZE, 0xbf),

	COEFF_RAM_CTL("Sub MBC 3 BiQuad 1", BIQUAD_SIZE, 0xc4),
	COEFF_RAM_CTL("Sub MBC 3 BiQuad 2", BIQUAD_SIZE, 0xc9),
	

};

/****************
 * DAPM Widgets *
 ****************/

static struct snd_soc_dapm_widget const tscs454_dapm_widgets[] = {
	/* R_PLLCTL PG 0 ADDR 0x15 */
	SND_SOC_DAPM_SUPPLY("PLL 1 Power", R_PLLCTL, FB_PLLCTL_PU_PLL1, 0,
			pll_power_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("PLL 2 Power", R_PLLCTL, FB_PLLCTL_PU_PLL2, 0,
			pll_power_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("PLL 2 Clock", R_PLLCTL, FB_PLLCTL_PLL2CLKEN, 0, 0, 0),
	SND_SOC_DAPM_SUPPLY("PLL 1 Clock", R_PLLCTL, FB_PLLCTL_PLL1CLKEN, 0, 0, 0),
	/* R_I2SPINC0 PG 0 ADDR 0x22 */
	SND_SOC_DAPM_AIF_OUT("DAI 3 Out", "DAI 3 Capture", 0,
			R_I2SPINC0, FB_I2SPINC0_SDO3TRI, 1),
	SND_SOC_DAPM_AIF_OUT("DAI 2 Out", "DAI 2 Capture", 0,
			R_I2SPINC0, FB_I2SPINC0_SDO2TRI, 1),
	SND_SOC_DAPM_AIF_OUT("DAI 1 Out", "DAI 1 Capture", 0,
			R_I2SPINC0, FB_I2SPINC0_SDO1TRI, 1),
	/* R_ASRC PG 0 ADDR 0x28 */
	SND_SOC_DAPM_SUPPLY("ASRC In Clock", R_ASRC, FB_ASRC_ASRCIB, 1, 0, 0),
	SND_SOC_DAPM_SUPPLY("ASRC Out Clock", R_ASRC, FB_ASRC_ASRCOB, 1, 0, 0),
	/* R_PWRM0 PG 0 ADDR 0x33 */
	SND_SOC_DAPM_ADC("Input Processor Channel 3", NULL,
			R_PWRM0, FB_PWRM0_INPROC3PU, 0),
	SND_SOC_DAPM_ADC("Input Processor Channel 2", NULL,
			R_PWRM0, FB_PWRM0_INPROC2PU, 0),
	SND_SOC_DAPM_ADC("Input Processor Channel 1", NULL,
			R_PWRM0, FB_PWRM0_INPROC1PU, 0),
	SND_SOC_DAPM_ADC("Input Processor Channel 0", NULL,
			R_PWRM0, FB_PWRM0_INPROC0PU, 0),
	SND_SOC_DAPM_SUPPLY("Mic Bias 2",
			R_PWRM0, FB_PWRM0_MICB2PU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Mic Bias 1",
			R_PWRM0, FB_PWRM0_MICB1PU, 0, NULL, 0),
	/* R_PWRM1 PG 0 ADDR 0x34 */
	SND_SOC_DAPM_SUPPLY("Sub Power", R_PWRM1, FB_PWRM1_SUBPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Headphone Left Power",
			R_PWRM1, FB_PWRM1_HPLPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Headphone Right Power",
			R_PWRM1, FB_PWRM1_HPRPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Speaker Left Power",
			R_PWRM1, FB_PWRM1_SPKLPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Speaker Right Power",
			R_PWRM1, FB_PWRM1_SPKRPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Differential Input 2 Power",
			R_PWRM1, FB_PWRM1_D2S2PU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Differential Input 1 Power",
			R_PWRM1, FB_PWRM1_D2S1PU, 0, NULL, 0),
	/* R_PWRM2 PG 0 ADDR 0x35 */
	SND_SOC_DAPM_SUPPLY("DAI 3 Out Power",
			R_PWRM2, FB_PWRM2_I2S3OPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAI 2 Out Power",
			R_PWRM2, FB_PWRM2_I2S2OPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAI 1 Out Power",
			R_PWRM2, FB_PWRM2_I2S1OPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAI 3 In Power",
			R_PWRM2, FB_PWRM2_I2S3IPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAI 2 In Power",
			R_PWRM2, FB_PWRM2_I2S2IPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAI 1 In Power",
			R_PWRM2, FB_PWRM2_I2S1IPU, 0, NULL, 0),
	/* R_PWRM3 PG 0 ADDR 0x36 */
	SND_SOC_DAPM_SUPPLY("Bandgap Self Bias Power",
			R_PWRM3, FB_PWRM3_BGSBUP, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("VGB Power", R_PWRM3, FB_PWRM3_VGBAPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Out Left Power",
			R_PWRM3, FB_PWRM3_LLINEPU, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Out Right Power",
			R_PWRM3, FB_PWRM3_RLINEPU, 0, NULL, 0),
	/* R_PWRM4 PG 0 ADDR 0x37 */
	SND_SOC_DAPM_DAC("Sub", NULL, R_PWRM4, FB_PWRM4_OPSUBPU, 0),
	SND_SOC_DAPM_DAC("DAC Left", NULL, R_PWRM4, FB_PWRM4_OPDACLPU, 0),
	SND_SOC_DAPM_DAC("DAC Right", NULL, R_PWRM4, FB_PWRM4_OPDACRPU, 0),
	SND_SOC_DAPM_DAC("ClassD Left", NULL, R_PWRM4, FB_PWRM4_OPSPKLPU, 0),
	SND_SOC_DAPM_DAC("ClassD Right", NULL, R_PWRM4, FB_PWRM4_OPSPKRPU, 0),
	/* R_AUDIOMUX1  PG 0 ADDR 0x3A */
	SND_SOC_DAPM_MUX("DAI 2 Out Mux", SND_SOC_NOPM, 0, 0,
			&dai2_mux_dapm_enum),
	SND_SOC_DAPM_MUX("DAI 1 Out Mux", SND_SOC_NOPM, 0, 0,
			&dai1_mux_dapm_enum),
	/* R_AUDIOMUX2 PG 0 ADDR 0x3B */
	SND_SOC_DAPM_MUX("Headphone Mux", SND_SOC_NOPM, 0, 0,
			&dac_mux_dapm_enum),
	SND_SOC_DAPM_MUX("DAI 3 Out Mux", SND_SOC_NOPM, 0, 0,
			&dai3_mux_dapm_enum),
	/* R_AUDIOMUX3 PG 0 ADDR 0x3C */
	SND_SOC_DAPM_MUX("Sub Mux", SND_SOC_NOPM, 0, 0,
			&sub_mux_dapm_enum),
	SND_SOC_DAPM_MUX("Speaker Mux", SND_SOC_NOPM, 0, 0,
			&classd_mux_dapm_enum),
	/* R_CH0AIC PG 1 ADDR 0x06 */
	SND_SOC_DAPM_MUX("Input Boost Channel 0 Mux", SND_SOC_NOPM, 0, 0,
			&in_bst_mux_ch0_dapm_enum),
	SND_SOC_DAPM_MUX("ADC Channel 0 Mux", SND_SOC_NOPM, 0, 0, 
			&adc_mux_ch0_dapm_enum),
	SND_SOC_DAPM_MUX("Input Processor Channel 0 Mux", SND_SOC_NOPM, 0, 0,
			&in_proc_mux_ch0_dapm_enum),
	/* R_CH1AIC PG 1 ADDR 0x07 */
	SND_SOC_DAPM_MUX("Input Boost Channel 1 Mux", SND_SOC_NOPM, 0, 0,
			&in_bst_mux_ch1_dapm_enum),
	SND_SOC_DAPM_MUX("ADC Channel 1 Mux", SND_SOC_NOPM, 0, 0,
			&adc_mux_ch1_dapm_enum),
	SND_SOC_DAPM_MUX("Input Processor Channel 1 Mux", SND_SOC_NOPM, 0, 0,
			&in_proc_mux_ch1_dapm_enum),
	/* Virtual */
	SND_SOC_DAPM_AIF_IN("DAI 3 In", "DAI 3 Playback", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DAI 2 In", "DAI 2 Playback", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DAI 1 In", "DAI 1 Playback", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("PLLs", SND_SOC_NOPM, 0, 0, 0, 0),
	SND_SOC_DAPM_SUPPLY("ASRCs", SND_SOC_NOPM, 0, 0, 0, 0),
	SND_SOC_DAPM_OUTPUT("Sub Out"),
	SND_SOC_DAPM_OUTPUT("Headphone Left"),
	SND_SOC_DAPM_OUTPUT("Headphone Right"),
	SND_SOC_DAPM_OUTPUT("Speaker Left"),
	SND_SOC_DAPM_OUTPUT("Speaker Right"),
	SND_SOC_DAPM_OUTPUT("Line Out Left"),
	SND_SOC_DAPM_OUTPUT("Line Out Right"),
	SND_SOC_DAPM_INPUT("D2S 2"),
	SND_SOC_DAPM_INPUT("D2S 1"),
	SND_SOC_DAPM_INPUT("Line In 1 Left"),
	SND_SOC_DAPM_INPUT("Line In 1 Right"),
	SND_SOC_DAPM_INPUT("Line In 2 Left"),
	SND_SOC_DAPM_INPUT("Line In 2 Right"),
	SND_SOC_DAPM_INPUT("Line In 3 Left"),
	SND_SOC_DAPM_INPUT("Line In 3 Right"),
	SND_SOC_DAPM_INPUT("DMic 1"),
	SND_SOC_DAPM_INPUT("DMic 2"),

	SND_SOC_DAPM_MUX("CH 0_1 Mux", SND_SOC_NOPM, 0, 0,
			&ch_0_1_mux_dapm_enum),
	SND_SOC_DAPM_MUX("CH 2_3 Mux", SND_SOC_NOPM, 0, 0,
			&ch_2_3_mux_dapm_enum),
	SND_SOC_DAPM_MUX("CH 4_5 Mux", SND_SOC_NOPM, 0, 0,
			&ch_4_5_mux_dapm_enum),
};

static struct snd_soc_dapm_route const tscs454_intercon[] = {
	/* PLLs */
	{"PLL 1 Clock", NULL, "PLL 1 Power"},
	{"PLL 2 Clock", NULL, "PLL 2 Power"},
	{"PLLs", NULL, "PLL 1 Clock", pll_connected},
	{"PLLs", NULL, "PLL 2 Clock", pll_connected},
	/* ASRCs */
	{"ASRCs", NULL, "ASRC In Clock", asrc_connected},
	{"ASRCs", NULL, "ASRC Out Clock", asrc_connected},
	/* Inputs */
	{"DAI 3 In", NULL, "DAI 3 In Power"},
	{"DAI 2 In", NULL, "DAI 2 In Power"},
	{"DAI 1 In", NULL, "DAI 1 In Power"},
	{"DAI 3 In", NULL, "ASRCs"},
	{"DAI 2 In", NULL, "ASRCs"},
	{"DAI 1 In", NULL, "ASRCs"},
	/* Outputs */
	{"DAI 3 Out", NULL, "DAI 3 Out Power"},
	{"DAI 2 Out", NULL, "DAI 2 Out Power"},
	{"DAI 1 Out", NULL, "DAI 1 Out Power"},
	{"DAI 3 Out", NULL, "ASRCs"},
	{"DAI 2 Out", NULL, "ASRCs"},
	{"DAI 1 Out", NULL, "ASRCs"},
	/* Ch Muxing */
	{"CH 0_1 Mux", "DAI 1", "DAI 1 In"},
	{"CH 0_1 Mux", "TDM 0_1", "DAI 1 In"},
	{"CH 2_3 Mux", "DAI 2", "DAI 2 In"},
	{"CH 2_3 Mux", "TDM 2_3", "DAI 1 In"},
	{"CH 4_5 Mux", "DAI 3", "DAI 2 In"},
	{"CH 4_5 Mux", "TDM 4_5", "DAI 1 In"},
	/* In/Out Muxing */
	{"DAI 1 Out Mux", "CH 0_1", "CH 0_1 Mux"},
	{"DAI 1 Out Mux", "CH 2_3", "CH 2_3 Mux"},
	{"DAI 1 Out Mux", "CH 4_5", "CH 4_5 Mux"},
	{"DAI 2 Out Mux", "CH 0_1", "CH 0_1 Mux"},
	{"DAI 2 Out Mux", "CH 2_3", "CH 2_3 Mux"},
	{"DAI 2 Out Mux", "CH 4_5", "CH 4_5 Mux"},
	{"DAI 3 Out Mux", "CH 0_1", "CH 0_1 Mux"},
	{"DAI 3 Out Mux", "CH 2_3", "CH 2_3 Mux"},
	{"DAI 3 Out Mux", "CH 4_5", "CH 4_5 Mux"},
	/****************** 
	 * Playback Paths *
	 ******************/
	/* DAC Path */
	{"Headphone Mux", "CH 4_5", "CH 4_5 Mux"},
	{"Headphone Mux", "CH 2_3", "CH 2_3 Mux"},
	{"Headphone Mux", "CH 0_1", "CH 0_1 Mux"},
	{"DAC Left", NULL, "Headphone Mux"},
	{"DAC Right", NULL, "Headphone Mux"},
	{"DAC Left", NULL, "PLLs"},
	{"DAC Right", NULL, "PLLs"},
	{"Headphone Left", NULL, "Headphone Left Power"},
	{"Headphone Right", NULL, "Headphone Right Power"},
	{"Headphone Left", NULL, "DAC Left"},
	{"Headphone Right", NULL, "DAC Right"},
	/* ClassD Path */
	{"Speaker Mux", "CH 4_5", "CH 4_5 Mux"},
	{"Speaker Mux", "CH 2_3", "CH 2_3 Mux"},
	{"Speaker Mux", "CH 0_1", "CH 0_1 Mux"},
	{"ClassD Left", NULL, "Speaker Mux"},
	{"ClassD Right", NULL, "Speaker Mux"},
	{"ClassD Left", NULL, "PLLs"},
	{"ClassD Right", NULL, "PLLs"},
	{"Speaker Left", NULL, "Speaker Left Power"},
	{"Speaker Right", NULL, "Speaker Right Power"},
	{"Speaker Left", NULL, "ClassD Left"},
	{"Speaker Right", NULL, "ClassD Right"},
	/* Sub Path */
	{"Sub Mux", "CH 4", "CH 4_5 Mux"},
	{"Sub Mux", "CH 5", "CH 4_5 Mux"},
	{"Sub Mux", "CH 4 + 5", "CH 4_5 Mux"},
	{"Sub Mux", "CH 2", "CH 2_3 Mux"},
	{"Sub Mux", "CH 3", "CH 2_3 Mux"},
	{"Sub Mux", "CH 2 + 3", "CH 2_3 Mux"},
	{"Sub Mux", "CH 0", "CH 0_1 Mux"},
	{"Sub Mux", "CH 1", "CH 0_1 Mux"},
	{"Sub Mux", "CH 0 + 1", "CH 0_1 Mux"},
	{"Sub Mux", "ADC/DMic 1 Left", "Input Processor Channel 0"},
	{"Sub Mux", "ADC/DMic 1 Right", "Input Processor Channel 1"},
	{"Sub Mux", "ADC/DMic 1 Left Plus Right", "Input Processor Channel 0"},
	{"Sub Mux", "ADC/DMic 1 Left Plus Right", "Input Processor Channel 1"},
	{"Sub Mux", "DMic 2 Left", "DMic 2"},
	{"Sub Mux", "DMic 2 Right", "DMic 2"},
	{"Sub Mux", "DMic 2 Left Plus Right", "DMic 2"},
	{"Sub Mux", "ClassD Left", "ClassD Left"},
	{"Sub Mux", "ClassD Right", "ClassD Right"},
	{"Sub Mux", "ClassD Left Plus Right", "ClassD Left"},
	{"Sub Mux", "ClassD Left Plus Right", "ClassD Right"},
	{"Sub", NULL, "Sub Mux"},
	{"Sub", NULL, "PLLs"},
	{"Sub Out", NULL, "Sub Power"},
	{"Sub Out", NULL, "Sub"},
	/***************** 
	 * Capture Paths *
	 *****************/
	{"Input Boost Channel 0 Mux", "Input 3", "Line In 3 Left"},
	{"Input Boost Channel 0 Mux", "Input 2", "Line In 2 Left"},
	{"Input Boost Channel 0 Mux", "Input 1", "Line In 1 Left"},
	{"Input Boost Channel 0 Mux", "D2S", "D2S 1"},

	{"Input Boost Channel 1 Mux", "Input 3", "Line In 3 Right"},
	{"Input Boost Channel 1 Mux", "Input 2", "Line In 2 Right"},
	{"Input Boost Channel 1 Mux", "Input 1", "Line In 1 Right"},
	{"Input Boost Channel 1 Mux", "D2S", "D2S 2"},

	{"ADC Channel 0 Mux", "Input 3 Boost Bypass", "Line In 3 Left"},
	{"ADC Channel 0 Mux", "Input 2 Boost Bypass", "Line In 2 Left"},
	{"ADC Channel 0 Mux", "Input 1 Boost Bypass", "Line In 1 Left"},
	{"ADC Channel 0 Mux", "Input Boost", "Input Boost Channel 0 Mux"},

	{"ADC Channel 1 Mux", "Input 3 Boost Bypass", "Line In 3 Right"},
	{"ADC Channel 1 Mux", "Input 2 Boost Bypass", "Line In 2 Right"},
	{"ADC Channel 1 Mux", "Input 1 Boost Bypass", "Line In 1 Right"},
	{"ADC Channel 1 Mux", "Input Boost", "Input Boost Channel 1 Mux"},

	{"Input Processor Channel 0 Mux", "ADC", "ADC Channel 0 Mux"},
	{"Input Processor Channel 0 Mux", "DMic", "DMic 1"},

	{"Input Processor Channel 0", NULL, "PLLs"},
	{"Input Processor Channel 0", NULL, "Input Processor Channel 0 Mux"},

	{"Input Processor Channel 1 Mux", "ADC", "ADC Channel 1 Mux"},
	{"Input Processor Channel 1 Mux", "DMic", "DMic 1"},

	{"Input Processor Channel 1", NULL, "PLLs"},
	{"Input Processor Channel 1", NULL, "Input Processor Channel 1 Mux"},

	{"Input Processor Channel 2", NULL , "PLLs"},
	{"Input Processor Channel 2", NULL , "DMic 2"},

	{"Input Processor Channel 3", NULL , "PLLs"},
	{"Input Processor Channel 3", NULL , "DMic 2"},

	{"DAI 1 Out Mux", "ADC/DMic 1", "Input Processor Channel 0"},
	{"DAI 1 Out Mux", "ADC/DMic 1", "Input Processor Channel 1"},
	{"DAI 1 Out Mux", "DMic 2", "Input Processor Channel 2"},
	{"DAI 1 Out Mux", "DMic 2", "Input Processor Channel 3"},

	{"DAI 2 Out Mux", "ADC/DMic 1", "Input Processor Channel 0"},
	{"DAI 2 Out Mux", "ADC/DMic 1", "Input Processor Channel 1"},
	{"DAI 2 Out Mux", "DMic 2", "Input Processor Channel 2"},
	{"DAI 2 Out Mux", "DMic 2", "Input Processor Channel 3"},

	{"DAI 3 Out Mux", "ADC/DMic 1", "Input Processor Channel 0"},
	{"DAI 3 Out Mux", "ADC/DMic 1", "Input Processor Channel 1"},
	{"DAI 3 Out Mux", "DMic 2", "Input Processor Channel 2"},
	{"DAI 3 Out Mux", "DMic 2", "Input Processor Channel 3"},

	{"DAI 1 Out", NULL, "DAI 1 Out Mux"},
	{"DAI 2 Out", NULL, "DAI 2 Out Mux"},
	{"DAI 3 Out", NULL, "DAI 3 Out Mux"},
};

/************************
 * Core DAI Operationss *
 ************************/

/* Clocking */
int tscs454_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);

	tscs454->bclk_freq = freq;
	return set_sysclk(codec);
}

static int tscs454_set_bclk_ratio(struct snd_soc_dai *dai,
		unsigned int ratio)
{
	unsigned int mask;
	int ret;
	struct snd_soc_codec *codec = dai->codec;
	unsigned int val;
	int shift;

	dev_info(codec->dev, "set_bclk_ratio() id = %d ratio = %u\n", dai->id, ratio);

	switch (dai->id) {
	case TSCS454_DAI1_ID:
		mask = FM_I2SCMC_BCMP1;
		shift = FB_I2SCMC_BCMP1;
		break;
	case TSCS454_DAI2_ID:
		mask = FM_I2SCMC_BCMP2;
		shift = FB_I2SCMC_BCMP2;
		break;
	case TSCS454_DAI3_ID:
		mask = FM_I2SCMC_BCMP3;
		shift = FB_I2SCMC_BCMP3;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Unknown audio interface (%d)\n", ret);
		return ret;
	}

	switch (ratio) {
	case 32:
		val = I2SCMC_BCMP_32X;
		break;
	case 40:
		val = I2SCMC_BCMP_40X;
		break;
	case 64:
		val = I2SCMC_BCMP_64X;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Unsupported bclk ratio (%d)\n", ret);
		return ret;
	}

	ret = snd_soc_update_bits(codec, R_I2SCMC, mask, val << shift); 
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set DAI BCLK ratio (%d)\n", ret);
		return ret;
	}

	return 0;
}

static inline int set_aif_master_from_fmt(struct snd_soc_codec *codec,
		struct aif *aif, unsigned int fmt)
{
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	int ret;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aif->master = true;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aif->master = false;
		if (reserve_asrc(&tscs454->asrc_in, aif->id)) {
			aif->asrc_in = &tscs454->asrc_in;
		} else {
			ret = -EBUSY;
			dev_err(codec->dev,
					"ASRC in is not available (%d)\n",
					ret);
			return ret;
		}
		if (reserve_asrc(&tscs454->asrc_out, aif->id)) {
			aif->asrc_out = &tscs454->asrc_out;
		} else {
			ret = -EBUSY;
			dev_err(codec->dev,
					"ASRC out is not available (%d)\n",
					ret);
			return ret;
		}
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Unsupported format (%d)\n", ret);
		return ret;
	}

	return 0;
}

static inline int set_aif_tdm_delay(struct snd_soc_codec *codec,
		unsigned int dai_id, bool delay)
{
	unsigned int reg;
	int ret;

	switch (dai_id) {
	case TSCS454_DAI1_ID:
		reg = R_TDMCTL0;
		break;
	case TSCS454_DAI2_ID:
		reg = R_PCMP2CTL0;
		break;
	case TSCS454_DAI3_ID:
		reg = R_PCMP3CTL0;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "DAI %d unknown (%d)\n", dai_id + 1, ret);
		return ret;
	}
	ret = snd_soc_update_bits(codec,
			reg, FM_TDMCTL0_BDELAY, delay);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to setup tdm format (%d)\n",
				ret);
		return ret;
	}

	return 0;
}

static inline int set_aif_format_from_fmt(struct snd_soc_codec *codec,
		unsigned int dai_id, unsigned int fmt)
{
	unsigned int reg;
	unsigned int val;
	int ret;

	switch (dai_id) {
	case TSCS454_DAI1_ID:
		reg = R_I2SP1CTL;
		break;
	case TSCS454_DAI2_ID:
		reg = R_I2SP2CTL;
		break;
	case TSCS454_DAI3_ID:
		reg = R_I2SP3CTL;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "DAI %d unknown (%d)\n", dai_id + 1, ret);
		return ret;
	}

	switch(fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		val = FV_FORMAT_RIGHT;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val = FV_FORMAT_LEFT;
		break;
	case SND_SOC_DAIFMT_I2S:
		val = FV_FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ret = set_aif_tdm_delay(codec, dai_id, true);
		if (ret < 0)
			return ret;
		val = FV_FORMAT_TDM;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ret = set_aif_tdm_delay(codec, dai_id, false);
		if (ret < 0)
			return ret;
		val = FV_FORMAT_TDM;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Format unsupported (%d)\n", ret);
		return ret;
	}

	ret = snd_soc_update_bits(codec, reg, FM_I2SPCTL_FORMAT, val);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set DAI %d format (%d)\n",
				dai_id + 1, ret);
		return ret;
	}

	return 0;
}

static inline int set_aif_clock_format_from_fmt(struct snd_soc_codec *codec,
		unsigned int dai_id, unsigned int fmt)
{
	unsigned int reg;
	unsigned int val;
	int ret;

	switch (dai_id) {
	case TSCS454_DAI1_ID:
		reg = R_I2SP1CTL;
		break;
	case TSCS454_DAI2_ID:
		reg = R_I2SP2CTL;
		break;
	case TSCS454_DAI3_ID:
		reg = R_I2SP3CTL;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "DAI %d unknown (%d)\n", dai_id + 1, ret);
		return ret;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		val = FV_BCLKP_NOT_INVERTED | FV_LRCLKP_NOT_INVERTED;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		val = FV_BCLKP_NOT_INVERTED | FV_LRCLKP_INVERTED;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		val = FV_BCLKP_INVERTED | FV_LRCLKP_NOT_INVERTED;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		val = FV_BCLKP_INVERTED | FV_LRCLKP_INVERTED;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Format unknown (%d)\n", ret);
		return ret;
	}

	ret = snd_soc_update_bits(codec, reg, 
			FM_I2SPCTL_BCLKP | FM_I2SPCTL_LRCLKP, val);
	if (ret < 0) {
		dev_err(codec->dev,
				"Failed to set clock polarity for DAI%d (%d)\n",
				dai_id + 1, ret);
		return ret;
	}

	return 0;
}

static int tscs454_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	struct aif *aif = &tscs454->dais[dai->id];
	int ret;

	ret = set_aif_master_from_fmt(codec, aif, fmt);
	if (ret < 0)
		return ret;

	ret = set_aif_format_from_fmt(codec, dai->id, fmt);
	if (ret < 0)
		return ret;

	ret = set_aif_clock_format_from_fmt(codec, dai->id, fmt);
	if (ret < 0)
		return ret;

	return 0;
}

static int tscs454_dai1_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
		unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int val;
	int ret;

	if (!slots)
		return 0;

	if (tx_mask >= (1 << slots) || rx_mask >= (1 << slots)) {
		ret = -EINVAL;
		dev_err(codec->dev, "Invalid TDM slot mask (%d)\n", ret);
		return ret;
	}

	switch (slots) {
	case 2:
		val = FV_TDMSO_2 | FV_TDMSI_2;
		break;
	case 4:
		val = FV_TDMSO_4 | FV_TDMSI_4;
		break;
	case 6:
		val = FV_TDMSO_6 | FV_TDMSI_6;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Invalid number of slots (%d)\n", ret);
		return ret;
	}

	switch (slot_width) {
	case 16:
		val = val | FV_TDMDSS_16;
		break;
	case 24:
		val = val | FV_TDMDSS_24;
		break;
	case 32:
		val = val | FV_TDMDSS_32;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Invalid TDM slot width (%d)\n", ret);
		return ret;
	}
	ret = snd_soc_write(codec, R_TDMCTL1, val);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set slots (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int tscs454_dai23_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
		unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int reg;
	unsigned int val;
	int ret;

	if (!slots)
		return 0;

	if (tx_mask >= (1 << slots) || rx_mask >= (1 << slots)) {
		ret = -EINVAL;
		dev_err(codec->dev, "Invalid TDM slot mask (%d)\n", ret);
		return ret;
	}

	switch (dai->id) {
	case TSCS454_DAI2_ID:
		reg = R_PCMP2CTL1;
		break;
	case TSCS454_DAI3_ID:
		reg = R_PCMP3CTL1;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Unrecognized interface %d (%d)\n",
				dai->id, ret);
		return ret;
	}

	switch (slots) {
	case 1:
		val = FV_PCMSOP_1 | FV_PCMSIP_1;
		break;
	case 2:
		val = FV_PCMSOP_2 | FV_PCMSIP_2;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Invalid number of slots (%d)\n", ret);
		return ret;
	}

	switch (slot_width) {
	case 16:
		val = val | FV_PCMDSSP_16;
		break;
	case 24:
		val = val | FV_PCMDSSP_24;
		break;
	case 32:
		val = val | FV_PCMDSSP_32;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Invalid TDM slot width (%d)\n", ret);
		return ret;
	}
	ret = snd_soc_write(codec, reg, val);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set slots (%d)\n", ret);
		return ret;
	}

	return 0;
}
/* Digital mute */

static int tscs454_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	return 0;
}

/* PCM Operationss */

static int tscs454_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return 0;
}

static void tscs454_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
}

static int set_aif_fs(struct snd_soc_codec *codec, 
		unsigned int id,
		unsigned int rate)
{
	unsigned int reg; 
	unsigned int br; 
	unsigned int bm;
	int ret;

	switch (rate) {
	case 8000:
		br = FV_I2SMBR_32;
		bm = FV_I2SMBM_0PT25;
		break;
	case 16000:
		br = FV_I2SMBR_32;
		bm = FV_I2SMBM_0PT5;
		break;
	case 24000:
		br = FV_I2SMBR_48;
		bm = FV_I2SMBM_0PT5;
		break;
	case 32000:
		br = FV_I2SMBR_32;
		bm = FV_I2SMBM_1;
		break;
	case 48000:
		br = FV_I2SMBR_48;
		bm = FV_I2SMBM_1;
		break;
	case 96000:
		br = FV_I2SMBR_48;
		bm = FV_I2SMBM_2;
		break;
	case 11025:
		br = FV_I2SMBR_44PT1;
		bm = FV_I2SMBM_0PT25;
		break;
	case 22050:
		br = FV_I2SMBR_44PT1;
		bm = FV_I2SMBM_0PT5;
		break;
	case 44100:
		br = FV_I2SMBR_44PT1;
		bm = FV_I2SMBM_1;
		break;
	case 88200:
		br = FV_I2SMBR_44PT1;
		bm = FV_I2SMBM_2;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Unsupported sample rate (%d)\n", ret);
		return ret;
	}

	switch(id) {
	case TSCS454_DAI1_ID:
		reg = R_I2S1MRATE;
		break;
	case TSCS454_DAI2_ID:
		reg = R_I2S2MRATE;
		break;
	case TSCS454_DAI3_ID:
		reg = R_I2S3MRATE;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "DAI ID not recognized (%d)\n", ret);
		return ret;
	}

	ret = snd_soc_update_bits(codec, reg,
			FM_I2SMRATE_I2SMBR | FM_I2SMRATE_I2SMBM, br|bm);
	if (ret < 0) {
		dev_err (codec->dev, "Failed to update register (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int set_aif_sample_format(struct snd_soc_codec *codec, 
		snd_pcm_format_t format, 
		int aif_id)
{
	unsigned int reg; 
	unsigned int width;
	int ret;

	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		width = FV_WL_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		width = FV_WL_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		width = FV_WL_32;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Unsupported format width (%d)\n", ret);
		return ret;
	}

	switch(aif_id) {
	case TSCS454_DAI1_ID:
		reg = R_I2SP1CTL;
		break;
	case TSCS454_DAI2_ID:
		reg = R_I2SP2CTL;
		break;
	case TSCS454_DAI3_ID:
		reg = R_I2SP3CTL;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "AIF ID not recognized (%d)\n", ret);
		return ret;
	}

	ret = snd_soc_update_bits(codec, reg, FM_I2SPCTL_WL, width);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set sample width (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int tscs454_hw_params(struct snd_pcm_substream *substream, 
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	bool isrc_configurable = false;
	unsigned int val;
	struct asrc *asrc = NULL;
	bool playback = SNDRV_PCM_STREAM_PLAYBACK == substream->stream;
	unsigned int fs = params_rate(params);
	struct pll *pll = PLL_44_1K_RATE % fs ?
			&tscs454->pll1 : &tscs454->pll2;
	struct aif *aif = &tscs454->dais[dai->id];
	int ret;
	struct pll *isrc_pll = NULL;
	unsigned int fs_gcf = fs % PLL_48K_FS_GCF ?
			PLL_44_1K_FS_GCF : PLL_48K_FS_GCF;

	dev_info(codec->dev, "hw_params() fs = %u\n", fs);

	mutex_lock(&tscs454->active_aifs_lock);
	if (!tscs454->active_aifs)
		isrc_configurable = true;
	tscs454->active_aifs++;
	mutex_unlock(&tscs454->active_aifs_lock);

	if (isrc_configurable) {
		ret = snd_soc_read(codec, R_PLLSTAT);
		if (ret < 0) {
			dev_err(codec->dev, "Failed to read PLL status (%d)\n",
					ret);
			return ret;
		}
		if (ret) { // Check if PLLs are powered by reading lock bits
			//TODO: Note returning an error breaks interface until reboot
			// These needs to be resolved.
			ret = -EBUSY;
			dev_err(codec->dev,
					"Failed to configure ISRC (%d)\n", ret);
			//return ret; // TODO: Throwing error breaks interface until reboot
		}
		if (PLL_48K_RATE % fs == 0) {
			dev_info(codec->dev, "hw_params() configuring ISRC 48K\n");
			tscs454->internal_rate.fs = ISRC_FS_48K;
			val = FV_IBR_48 | FV_IBM_1;
		} else {
			dev_info(codec->dev, "hw_params() configuring ISRC 44.1K\n");
			tscs454->internal_rate.fs = ISRC_FS_44_1K;
			val = FV_IBR_44PT1 | FV_IBM_1;
		}
		dev_info(codec->dev, "hw_params() writing %u to R_ISRC\n", val);
		ret = snd_soc_write(codec, R_ISRC, val);
		if (ret < 0) {
			dev_err(codec->dev, 
			"Failed to set internal fs (%d)\n", ret);
			//return ret; // TODO: Throwing error breaks interface until reboot
		}
		isrc_pll = pll;
	} else if (tscs454->internal_rate.fs % fs_gcf) {
		asrc = playback ? &tscs454->asrc_in : &tscs454->asrc_out;
		if (!reserve_asrc(asrc, aif->id)) {
			ret = -EBUSY;
			dev_err(codec->dev, "Failed to reserve asrc DAI (%d)\n",
				ret);
			return ret;
		}
		ret = configure_asrc(codec, asrc);
		if (ret < 0) {
			dev_err(codec->dev,
					"Failed to configure ASRC (%d)\n", ret);
			return ret;
		}
	}

	ret = set_aif_fs(codec, aif->id, fs);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set DAI fs (%d)\n", ret);
		return ret;
	}

	ret = set_aif_sample_format(codec, params_format(params), dai->id);
	if (ret < 0) {
		dev_err(codec->dev,
				"Failed to set DAI sample format (%d)\n", ret);
		return ret;
	}

	reserve_pll(pll);
	aif->pll = pll;
	if (isrc_pll) {
		reserve_pll(isrc_pll);
		tscs454->internal_rate.pll = isrc_pll;
	}
	if (asrc) {
		if (playback)
			aif->asrc_in = asrc;
		else
			aif->asrc_out = asrc;
	}

	return 0;
}

static int tscs454_hw_free(struct snd_pcm_substream * substream,
		struct snd_soc_dai *dai)
{
	int ret;
	struct snd_soc_codec *codec = dai->codec;
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	struct aif *aif = &tscs454->dais[dai->id];
	bool playback = SNDRV_PCM_STREAM_PLAYBACK == substream->stream;

	dev_info(codec->dev, "hw_free()\n");

	ret = aif_free(codec, aif, playback);
	if (ret < 0)
		return ret;

	return 0;
}

static int tscs454_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	int ret;
	struct snd_soc_codec *codec = dai->codec;
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	struct aif *aif = &tscs454->dais[dai->id];

	ret = aif_prepare(codec, aif);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_dai_ops const tscs454_dai1_ops = {
	.set_sysclk	= tscs454_set_sysclk,
	.set_bclk_ratio = tscs454_set_bclk_ratio,
	.set_fmt	= tscs454_set_dai_fmt,
	.set_tdm_slot	= tscs454_dai1_set_tdm_slot,
	.mute_stream	= tscs454_mute_stream,
	.startup	= tscs454_startup,
	.shutdown	= tscs454_shutdown,
	.hw_params	= tscs454_hw_params,
	.hw_free	= tscs454_hw_free,
	.prepare	= tscs454_prepare,
};

static struct snd_soc_dai_ops const tscs454_dai23_ops = {
	.set_bclk_ratio = tscs454_set_bclk_ratio,
	.set_fmt	= tscs454_set_dai_fmt,
	.set_tdm_slot	= tscs454_dai23_set_tdm_slot,
	.mute_stream	= tscs454_mute_stream,
	.startup	= tscs454_startup,
	.shutdown	= tscs454_shutdown,
	.hw_params	= tscs454_hw_params,
	.hw_free	= tscs454_hw_free,
	.prepare	= tscs454_prepare,
};

/****************
 * Constructors *
 ****************/

static int tscs454_probe(struct snd_soc_codec *codec)
{
	struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	if (tscs454->sysclk_src_id != FV_PLLISEL_BCLK)
		ret = set_sysclk(codec);

	return ret; 
}

static int tscs454_remove(struct snd_soc_codec *codec)
{
	//struct tscs454 *tscs454 = snd_soc_codec_get_drvdata(codec);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_tscs454 = {
	.probe =	tscs454_probe,
	.remove =	tscs454_remove,
	.component_driver = {
		.dapm_widgets = tscs454_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(tscs454_dapm_widgets),
		.dapm_routes = tscs454_intercon,
		.num_dapm_routes = ARRAY_SIZE(tscs454_intercon),
		.controls =	tscs454_snd_controls,
		.num_controls = ARRAY_SIZE(tscs454_snd_controls),
	},
};

#define TSCS454_RATES SNDRV_PCM_RATE_8000_96000

#define TSCS454_FORMATS (SNDRV_PCM_FMTBIT_S16_LE \
	| SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver tscs454_dais[] = {
	{
		.name = "tscs454-dai1",
		.id = TSCS454_DAI1_ID,
		.playback = {
			.stream_name = "DAI 1 Playback",
			.channels_min = 1,
			.channels_max = 6,
			.rates = TSCS454_RATES,
			.formats = TSCS454_FORMATS,},
		.capture = {
			.stream_name = "DAI 1 Capture",
			.channels_min = 1,
			.channels_max = 6,
			.rates = TSCS454_RATES,
			.formats = TSCS454_FORMATS,},
		.ops = &tscs454_dai1_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tscs454-dai2",
		.id = TSCS454_DAI2_ID,
		.playback = {
			.stream_name = "DAI 2 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TSCS454_RATES,
			.formats = TSCS454_FORMATS,},
		.capture = {
			.stream_name = "DAI 2 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TSCS454_RATES,
			.formats = TSCS454_FORMATS,},
		.ops = &tscs454_dai23_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tscs454-dai3",
		.id = TSCS454_DAI3_ID,
		.playback = {
			.stream_name = "DAI 3 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TSCS454_RATES,
			.formats = TSCS454_FORMATS,},
		.capture = {
			.stream_name = "DAI 3 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TSCS454_RATES,
			.formats = TSCS454_FORMATS,},
		.ops = &tscs454_dai23_ops,
		.symmetric_rates = 1,
	},
};

enum {
	TSCS454_PLL_MCLK_SRC_XTAL = FV_PLLISEL_XTAL,
	TSCS454_PLL_MCLK_SRC_MCLK1,
	TSCS454_PLL_MCLK_SRC_MCLK2,
	TSCS454_PLL_MCLK_SRC_CNT
};

/*******
 * I2C *
 *******/
static char const * const src_names[TSCS454_PLL_MCLK_SRC_CNT] = {
	"xtal", "mclk1", "mclk2"};

static int tscs454_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct tscs454 *tscs454;
	int src;
	int ret;

	tscs454 = devm_kzalloc(&i2c->dev, sizeof(*tscs454), GFP_KERNEL);
	if (!tscs454)
		return -ENOMEM;

	ret = tscs454_data_init(tscs454, i2c);
	if (ret < 0)
		return ret;

	i2c_set_clientdata(i2c, tscs454);

	for (src = TSCS454_PLL_MCLK_SRC_XTAL; src < TSCS454_PLL_MCLK_SRC_CNT; src++) {
		tscs454->sysclk = devm_clk_get(&i2c->dev, src_names[src]);
		if (!IS_ERR(tscs454->sysclk)) {
			break;
		} else if (PTR_ERR(tscs454->sysclk) != -ENOENT) {
			ret = PTR_ERR(tscs454->sysclk);
			dev_err(&i2c->dev, "Failed to get sysclk (%d)\n", ret);
			return ret;
		}
	}
	if (src == TSCS454_PLL_MCLK_SRC_CNT)
		tscs454->sysclk_src_id = FV_PLLISEL_BCLK;
	else
		tscs454->sysclk_src_id = src;

	ret = regmap_write(tscs454->regmap,
			R_RESET, FV_RESET_PWR_ON_DEFAULTS);
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to reset the codec (%d)\n", ret);
		return ret;
	}
	regcache_mark_dirty(tscs454->regmap);

	ret = regmap_register_patch(tscs454->regmap, tscs454_patch,
			ARRAY_SIZE(tscs454_patch));
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to apply patch (%d)\n", ret);
		return ret;
	}
	/* Sync pg sel reg with cache */
	regmap_write(tscs454->regmap, R_PAGESEL, 0x00);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_tscs454,
			tscs454_dais, ARRAY_SIZE(tscs454_dais));
	if (ret) {
		dev_err(&i2c->dev, "Failed to register codec (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int tscs454_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tscs454_i2c_id[] = {
	{ "tscs454", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tscs454_i2c_id);

static const struct of_device_id tscs454_of_match[] = {
	{ .compatible = "tempo,tscs454", },
	{ }
};
MODULE_DEVICE_TABLE(of, tscs454_of_match);

static struct i2c_driver tscs454_i2c_driver = {
	.driver = {
		.name = "tscs454",
		.owner = THIS_MODULE,
		.of_match_table = tscs454_of_match,
	},
	.probe =    tscs454_i2c_probe,
	.remove =   tscs454_i2c_remove,
	.id_table = tscs454_i2c_id,
};

module_i2c_driver(tscs454_i2c_driver);

MODULE_AUTHOR("Tempo Semiconductor <steven.eckhoff.opensource@gmail.com");
MODULE_DESCRIPTION("ASoC TSCS454 driver");
MODULE_LICENSE("GPL");
