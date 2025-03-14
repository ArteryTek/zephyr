/*
 * Copyright (c) 2018 Kokoon Technology Limited
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 * Copyright (c) 2019 Endre Karlson
 * Copyright (c) 2020 Teslabs Engineering S.L.
 * Copyright (c) 2021 Marius Scholtz, RIC Electronics
 * Copyright (c) 2023 Hein Wessels, Nobleo Technology
 * Copyright (c) 2025 Maxjta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT at_at32_adc

#include <errno.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/drivers/clock_control.h>
#include <at32_adc.h>

#ifdef CONFIG_ADC_AT32_DMA
#include <zephyr/drivers/dma/dma_at32.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/toolchain.h>
#include <at32_dma.h>
#endif

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_at32);

#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/mem_mgmt/mem_attr.h>

#ifdef CONFIG_NOCACHE_MEMORY
#include <zephyr/linker/linker-defs.h>
#elif defined(CONFIG_CACHE_MANAGEMENT)
#include <zephyr/arch/cache.h>
#endif /* CONFIG_NOCACHE_MEMORY */

/* Clock source values */
#define SYNC			1
#define ASYNC			2

/* Sequencer type */
#define NOT_FULLY_CONFIGURABLE	0
#define FULLY_CONFIGURABLE	1

/* Oversampler type */
#define OVERSAMPLER_NONE	0
#define OVERSAMPLER_MINIMAL	1
#define OVERSAMPLER_EXTENDED	2

#define ANY_NUM_COMMON_SAMPLING_TIME_CHANNELS_IS(value) \
	(DT_INST_FOREACH_STATUS_OKAY_VARGS(IS_EQ_PROP_OR, \
					   num_sampling_time_common_channels,\
					   0, value) 0)

#define ANY_ADC_SEQUENCER_TYPE_IS(value) \
	(DT_INST_FOREACH_STATUS_OKAY_VARGS(IS_EQ_STRING_PROP, \
					   st_adc_sequencer,\
					   value) 0)

#define ANY_ADC_OVERSAMPLER_TYPE_IS(value) \
	(DT_INST_FOREACH_STATUS_OKAY_VARGS(IS_EQ_STRING_PROP, \
					   st_adc_oversampler,\
					   value) 0)

#define IS_EQ_PROP_OR(inst, prop, default_value, compare_value) \
	IS_EQ(DT_INST_PROP_OR(inst, prop, default_value), compare_value) ||

#define IS_EQ_STRING_PROP(inst, prop, compare_value) \
	IS_EQ(DT_INST_STRING_UPPER_TOKEN(inst, prop), compare_value) ||

/* reference voltage for the ADC */
#define AT32_ADC_VREF_MV DT_INST_PROP(0, vref_mv)

/* Number of different sampling time values */
#define AT32_NB_SAMPLING_TIME	8

#define ADC1_NODE		DT_NODELABEL(adc1)
#define ADC2_NODE		DT_NODELABEL(adc2)
#define ADC3_NODE		DT_NODELABEL(adc3)

#define ADC1_ENABLE		DT_NODE_HAS_STATUS(ADC1_NODE, okay)
#define ADC2_ENABLE		DT_NODE_HAS_STATUS(ADC2_NODE, okay)
#define ADC3_ENABLE		DT_NODE_HAS_STATUS(ADC3_NODE, okay)

#define HAS_CALIBRATION

#ifdef CONFIG_ADC_AT32_DMA
struct stream {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
};
#endif /* CONFIG_ADC_AT32_DMA */

typedef uint16_t adc_data_size_t;

struct adc_at32_data {
	struct adc_context ctx;
	const struct device *dev;
	adc_data_size_t *buffer;
	adc_data_size_t *repeat_buffer;

	uint8_t resolution;
	uint32_t channels;
	uint8_t channel_count;
	uint8_t samples_count;
	int8_t acq_time_index[2];

#ifdef CONFIG_ADC_AT32_DMA
	volatile int dma_error;
	struct stream dma;
#endif
};

struct adc_at32_cfg {
	uint32_t reg;
	adc_type *base;
	void (*irq_cfg_func)(void);
	uint32_t irq_num;
	uint32_t clkid;
	uint32_t clk_prescaler;
	const struct pinctrl_dev_config *pcfg;
	const uint16_t sampling_time_table[AT32_NB_SAMPLING_TIME];
	int8_t num_sampling_time_common_channels;
	int8_t sequencer_type;
	int8_t oversampler_type;
	int8_t res_table_size;
	const uint32_t res_table[];
};

#ifdef CONFIG_ADC_AT32_DMA
static void adc_at32_enable_dma_support(adc_type *adc)
{
	/* Allow ADC to create DMA request and set to one-shot mode as implemented in HAL drivers */
    adc_dma_mode_enable(adc, TRUE);
}

static int adc_at32_dma_start(const struct device *dev,
			       void *buffer, size_t channel_count)
{
	const struct adc_at32_cfg *config = dev->config;
	adc_type *adc = config->base;
	struct adc_at32_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	int ret;

	struct stream *dma = &data->dma;

	blk_cfg = &dma->dma_blk_cfg;

	/* prepare the block */
	blk_cfg->block_size = channel_count; //* sizeof(adc_data_size_t);

	/* Source and destination */
	blk_cfg->source_address = (uint32_t)(&adc->odt);
	blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg->source_reload_en = 0;

	blk_cfg->dest_address = (uint32_t)buffer;
	blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk_cfg->dest_reload_en = 0;
	/* Manually set the FIFO threshold to 1/4 because the
	 * dmamux DTS entry does not contain fifo threshold
	 */
	blk_cfg->fifo_mode_control = 0;

	/* direction is given by the DT */
	dma->dma_cfg.head_block = blk_cfg;
	dma->dma_cfg.user_data = data;

	ret = dma_config(data->dma.dma_dev, data->dma.channel,
			 &dma->dma_cfg);
	if (ret != 0) {
		LOG_ERR("Problem setting up DMA: %d", ret);
		return ret;
	}
//	adc_at32_enable_dma_support(adc);

	data->dma_error = 0;
	ret = dma_start(data->dma.dma_dev, data->dma.channel);
	if (ret != 0) {
		LOG_ERR("Problem starting DMA: %d", ret);
		return ret;
	}

	LOG_DBG("DMA started");

	return ret;
}
#endif /* CONFIG_ADC_AT32_DMA */

static int check_buffer(const struct adc_sequence *sequence,
			     uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(adc_data_size_t);

	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Provided buffer is too small (%u/%u)",
				sequence->buffer_size, needed_buffer_size);
		return -ENOMEM;
	}

	return 0;
}

/*
 * Enable ADC peripheral, and wait until ready if required by SOC.
 */
static int adc_at32_enable(adc_type *adc)
{
	if (adc->ctrl2_bit.adcen == 1UL) {
		return 0;
	}

    adc_enable(adc, TRUE);

	return 0;
}

static void adc_at32_start_conversion(const struct device *dev)
{
	const struct adc_at32_cfg *config = dev->config;
	adc_type *adc = config->base;

	LOG_DBG("Starting conversion");

    adc_ordinary_software_trigger_enable(adc, TRUE);
}

/*
 * Disable ADC peripheral, and wait until it is disabled
 */
static void adc_at32_disable(adc_type *adc)
{
	adc_enable(adc, FALSE);
}


static int adc_at32_calibrate(const struct device *dev)
{
	const struct adc_at32_cfg *config =
		(const struct adc_at32_cfg *)dev->config;
	adc_type *adc = config->base;

	adc_enable(adc, TRUE);
	adc_calibration_init(adc);
    while (adc_calibration_init_status_get(adc)) {
	};
    adc_calibration_start(adc);
    while (adc_calibration_status_get(adc)) {
	};
	return 0;
}


#if defined(HAS_OVERSAMPLING)

#define MAX_OVS_SHIFT	8

#define OVS_SHIFT(i, _)	_CONCAT_1(ADC_OVERSAMPLE_SHIFT_, UTIL_INC(i))
static const uint32_t table_oversampling_shift[] = {
	0,
	LISTIFY(MAX_OVS_SHIFT, OVS_SHIFT, (,))
};

#if ANY_ADC_OVERSAMPLER_TYPE_IS(OVERSAMPLER_MINIMAL)
#define OVS_RATIO(n)		ADC_OVERSAMPLE_RATIO_##n
static const uint32_t table_oversampling_ratio[] = {
	0,
	OVS_RATIO(2),
	OVS_RATIO(4),
	OVS_RATIO(8),
	OVS_RATIO(16),
	OVS_RATIO(32),
	OVS_RATIO(64),
	OVS_RATIO(128),
	OVS_RATIO(256),
};
#endif

/*
 * Function to configure the oversampling scope. It is basically a wrapper over
 * adc_ordinary_oversample_enable() which in addition stops the ADC if needed.
 */
static void adc_at32_oversampling_scope(adc_type *adc, uint32_t ovs_scope)
{
	adc_ordinary_oversample_enable(adc, ovs_scope);
}

/*
 * Function to configure the oversampling ratio and shift. It is basically a
 * wrapper over adc_oversample_ratio_shift_set() which in addition stops the
 * ADC if needed.
 */
static void adc_at32_oversampling_ratioshift(ADC_TypeDef *adc, uint32_t ratio, uint32_t shift)
{
	adc_at32_disable(adc);
	adc_oversample_ratio_shift_set(adc, ratio, shift);
}
/*
 * Function to configure the oversampling ratio and shift using at32 LL
 * ratio is directly the sequence->oversampling (a 2^n value)
 * shift is the corresponding constant
 */
static int adc_at32_oversampling(const struct device *dev, uint8_t ratio)
{
	const struct adc_at32_cfg *config = dev->config;
	adc_type *adc = config->base;

	if (ratio == 0) {
		adc_at32_oversampling_scope(adc, FALSE);
		return 0;
	} else if (ratio < ARRAY_SIZE(table_oversampling_shift)) {
		adc_at32_oversampling_scope(adc, TRUE);
	} else {
		LOG_ERR("Invalid oversampling");
		return -EINVAL;
	}

	uint32_t shift = table_oversampling_shift[ratio];

#if ANY_ADC_OVERSAMPLER_TYPE_IS(OVERSAMPLER_MINIMAL)
	if (config->oversampler_type == OVERSAMPLER_MINIMAL) {
		/* the LL function expects a value LL_ADC_OVS_RATIO_x */
		adc_at32_oversampling_ratioshift(adc, table_oversampling_ratio[ratio], shift);
	}
#endif

#if ANY_ADC_OVERSAMPLER_TYPE_IS(OVERSAMPLER_EXTENDED)
	if (config->oversampler_type == OVERSAMPLER_EXTENDED) {
		/* the LL function expects a value from 1 to 1024 */
		adc_at32_oversampling_ratioshift(adc, 1 << ratio, shift);
	}
#endif

	return 0;
}
#endif

#ifdef CONFIG_ADC_AT32_DMA
static void dma_callback(const struct device *dev, void *user_data,
			 uint32_t channel, int status)
{
	/* user_data directly holds the adc device */
	struct adc_at32_data *data = user_data;
	LOG_DBG("dma callback");
	if (channel == data->dma.channel) {
		if (status >= 0) {
			data->samples_count = data->channel_count;
			data->buffer += data->channel_count;
			/* Stop the DMA engine, only to start it again when the callback returns
			 * ADC_ACTION_REPEAT or ADC_ACTION_CONTINUE, or the number of samples
			 * haven't been reached Starting the DMA engine is done
			 * within adc_context_start_sampling
			 */
			dma_stop(data->dma.dma_dev, data->dma.channel);
			/* No need to invalidate the cache because it's assumed that
			 * the address is in a non-cacheable SRAM region.
			 */
			adc_context_on_sampling_done(&data->ctx, dev);
			pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE,
						 PM_ALL_SUBSTATES);
			if (IS_ENABLED(CONFIG_PM_S2RAM)) {
				pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM,
							 PM_ALL_SUBSTATES);
			}
		} else if (status < 0) {
			LOG_ERR("DMA sampling complete, but DMA reported error %d", status);
			data->dma_error = status;
			dma_stop(data->dma.dma_dev, data->dma.channel);
			adc_context_complete(&data->ctx, status);
		}
	}
}
#endif /* CONFIG_ADC_AT32_DMA */

static int set_resolution(const struct device *dev,
			  const struct adc_sequence *sequence)
{
	const struct adc_at32_cfg *config = dev->config;
	adc_type *adc = config->base;
	uint32_t resolution = 0, res_val = 0;
	int i;
    
	if (config->res_table_size <= 1) {
		return 0;
	}

	for (i = 0; i < config->res_table_size; i++) {
		if (sequence->resolution == config->res_table[i]) {
            resolution = sequence->resolution;
		}
	}

	if (resolution == 0) {
		LOG_ERR("Invalid resolution");
		return -EINVAL;
	}

	res_val = adc->ctrl1;
	res_val &= ~(3 << 24);
	switch (resolution) {
		case 6:
		    res_val |= 3 << 24;
		    break;
		case 8:
		    res_val |= 2 << 24;
			break;
		case 10:
		    res_val |= 1 << 24;
			break;
		case 12:
		    res_val |= 0 << 24;
			break;
		default:
		    break;
	}
	adc->ctrl1 |= res_val;

	return 0;
}

static void at32_adc_sequencer_channel_set(adc_type *adc, uint8_t channel, uint8_t adc_sequence)
{
	uint32_t tmp_reg;
	if(adc_sequence >= 13) {
		tmp_reg = adc->osq1;
		tmp_reg &= ~(0x1F << (5 * (adc_sequence - 13)));
		tmp_reg |= (channel << (5 * (adc_sequence - 13)));
		adc->osq1 = tmp_reg;
	}
	else if(adc_sequence >= 7) {
		tmp_reg = adc->osq2;
		tmp_reg &= ~(0x1F << (5 * (adc_sequence - 7)));
		tmp_reg |= (channel << (5 * (adc_sequence - 7)));
		adc->osq2 = tmp_reg;
	}
	else {
		tmp_reg = adc->osq3;
		tmp_reg &= ~(0x1F << (5 * (adc_sequence - 1)));
		tmp_reg |= (channel << (5 * (adc_sequence - 1)));
		adc->osq3 = tmp_reg;
	}
}

static int set_sequencer(const struct device *dev)
{
	const struct adc_at32_cfg *config = dev->config;
	struct adc_at32_data *data = dev->data;
	adc_type *adc = config->base;

	uint8_t channel_id;
	uint8_t channel_index = 0;
	uint32_t channels_mask = 0;

	/* Iterate over selected channels in bitmask keeping track of:
	 * - channel_index: ranging from 0 -> ( data->channel_count - 1 )
	 * - channel_id: ordinal position of channel in data->channels bitmask
	 */
	for (uint32_t channels = data->channels; channels;
		      channels &= ~BIT(channel_id), channel_index++) {
		channel_id = find_lsb_set(channels) - 1;

		uint32_t channel = channel_id;

		channels_mask |= channel;
        at32_adc_sequencer_channel_set(adc, channel, channel_index + 1);

	}
    adc->ctrl1_bit.sqen = 1;
	adc->osq1_bit.oclen = channel_index - 1;
	return 0;
}

static int start_read(const struct device *dev,
		      const struct adc_sequence *sequence)
{
	const struct adc_at32_cfg *config = dev->config;
	struct adc_at32_data *data = dev->data;
	adc_type *adc = config->base;
	int err;
    
	data->buffer = sequence->buffer;
	data->channels = sequence->channels;
	data->channel_count = POPCOUNT(data->channels);
	data->samples_count = 0;

	if (data->channel_count == 0) {
		LOG_ERR("No channels selected");
		return -EINVAL;
	}

	/* Check and set the resolution */
	err = set_resolution(dev, sequence);
	if (err < 0) {
		return err;
	}

	/* Configure the sequencer */
	err = set_sequencer(dev);
	if (err < 0) {
		return err;
	}

	err = check_buffer(sequence, data->channel_count);
	if (err) {
		return err;
	}

#ifdef HAS_OVERSAMPLING
	err = adc_at32_oversampling(dev, sequence->oversampling);
	if (err) {
		return err;
	}
#else
	if (sequence->oversampling) {
		LOG_ERR("Oversampling not supported");
		return -ENOTSUP;
	}
#endif /* HAS_OVERSAMPLING */

	if (sequence->calibrate) {
#if defined(HAS_CALIBRATION)
		adc_at32_calibrate(dev);
#else
		LOG_ERR("Calibration not supported");
		return -ENOTSUP;
#endif
	}

	/*
	 * Make sure the ADC is enabled as it might have been disabled earlier
	 * to set the resolution, to set the oversampling or to perform the
	 * calibration.
	 */
    adc_at32_enable(adc);

#if !defined(CONFIG_ADC_AT32_DMA)
    adc_interrupt_enable(ADC1, ADC_CCE_INT, TRUE);
#endif /* CONFIG_ADC_AT32_DMA */

	/* This call will start the DMA */
	adc_context_start_read(&data->ctx, sequence);

	int result = adc_context_wait_for_completion(&data->ctx);

#ifdef CONFIG_ADC_AT32_DMA
	/* check if there's anything wrong with dma start */
	result = (data->dma_error ? data->dma_error : result);
#endif

	return result;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_at32_data *data =
		CONTAINER_OF(ctx, struct adc_at32_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_at32_cfg *config = dev->config;
	__maybe_unused adc_type *adc = config->base;

	data->repeat_buffer = data->buffer;
#ifdef CONFIG_ADC_AT32_DMA
	adc_at32_dma_start(dev, data->buffer, data->channel_count);
#endif
	adc_at32_start_conversion(dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct adc_at32_data *data =
		CONTAINER_OF(ctx, struct adc_at32_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

#ifndef CONFIG_ADC_AT32_DMA
static void adc_at32_isr(const struct device *dev)
{
	struct adc_at32_data *data = dev->data;
	const struct adc_at32_cfg *config =
		(const struct adc_at32_cfg *)dev->config;
	adc_type *adc = config->base;
	if (adc_flag_get(adc, 0x2)) {
        *data->buffer++ = adc_ordinary_conversion_data_get(adc);
		/* ISR is triggered after each conversion, and at the end-of-sequence. */
		if (++data->samples_count == data->channel_count) {
			data->samples_count = 0;
			adc_context_on_sampling_done(&data->ctx, dev);
			pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE,
						 PM_ALL_SUBSTATES);
			if (IS_ENABLED(CONFIG_PM_S2RAM)) {
				pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM,
							 PM_ALL_SUBSTATES);
			}
		}
	}
	LOG_DBG("%s ISR triggered.", dev->name);
}
#endif /* !CONFIG_ADC_AT32_DMA */

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_at32_data *data =
		CONTAINER_OF(ctx, struct adc_at32_data, ctx);
	const struct adc_at32_cfg *config = data->dev->config;
	__maybe_unused adc_type *adc = config->base;

	ARG_UNUSED(status);

	/* Reset acquisition time used for the sequence */
	data->acq_time_index[0] = -1;
	data->acq_time_index[1] = -1;
}

static int adc_at32_read(const struct device *dev,
			  const struct adc_sequence *sequence)
{
	struct adc_at32_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	if (IS_ENABLED(CONFIG_PM_S2RAM)) {
		pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
	}
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_at32_read_async(const struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct adc_at32_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	if (IS_ENABLED(CONFIG_PM_S2RAM)) {
		pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
	}
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif

static int adc_at32_sampling_time_check(const struct device *dev, uint16_t acq_time)
{
	const struct adc_at32_cfg *config =
		(const struct adc_at32_cfg *)dev->config;

	if (acq_time == ADC_ACQ_TIME_DEFAULT) {
		return 0;
	}

	if (acq_time == ADC_ACQ_TIME_MAX) {
		return AT32_NB_SAMPLING_TIME - 1;
	}

	for (int i = 0; i < AT32_NB_SAMPLING_TIME; i++) {
		if (acq_time == ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS,
					     config->sampling_time_table[i])) {
			return i;
		}
	}

	LOG_ERR("Sampling time value not supported.");
	return -EINVAL;
}

static int adc_at32_sampling_time_setup(const struct device *dev, uint8_t id,
					 uint16_t acq_time)
{
	const struct adc_at32_cfg *config =
		(const struct adc_at32_cfg *)dev->config;
	adc_type *adc = config->base;
	__maybe_unused struct adc_at32_data *data = dev->data;
	uint32_t spt_val = 0;

	int acq_time_index;

	acq_time_index = adc_at32_sampling_time_check(dev, acq_time);
	if (acq_time_index < 0) {
		return acq_time_index;
	}
    
	if (id < 10) {
		spt_val = adc->spt2;
		spt_val &= ~(0x7 << (3 * id));
		spt_val |= (acq_time_index << (3 * id));
		adc->spt2 = spt_val;
	} else {
		spt_val = adc->spt1;
		spt_val &= ~(0x7 << (3 * (id - 10)));
		spt_val |= (acq_time_index << (3 * (id - 10)));
		adc->spt1 = spt_val;
	}
	
	return 0;
}

static int adc_at32_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Invalid channel gain");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Invalid channel reference");
		return -EINVAL;
	}

	if (adc_at32_sampling_time_setup(dev, channel_cfg->channel_id,
					  channel_cfg->acquisition_time) != 0) {
		LOG_ERR("Invalid sampling time");
		return -EINVAL;
	}

	LOG_DBG("Channel setup succeeded!");

	return 0;
}

/* This symbol takes the value 1 if one of the device instances */
/* is configured in dts with a domain clock */
#if AT32_DT_INST_DEV_DOMAIN_CLOCK_SUPPORT
#define AT32_ADC_DOMAIN_CLOCK_SUPPORT 1
#else
#define AT32_ADC_DOMAIN_CLOCK_SUPPORT 0
#endif

static int adc_at32_set_clock(const struct device *dev)
{
	const struct adc_at32_cfg *config = dev->config;
	__maybe_unused adc_type *adc = config->base;
	int ret = 0;

	if (clock_control_on(AT32_CLOCK_CONTROLLER,
		(clock_control_subsys_t) &config->clkid) != 0) {
		return -EIO;
	}
#if defined(AT32_ADC_COMM_PSC)
    adc_clock_div_set(config->clk_prescaler);
#endif
	return ret;
}

static int adc_at32_init(const struct device *dev)
{
	struct adc_at32_data *data = dev->data;
	const struct adc_at32_cfg *config = dev->config;
	__maybe_unused adc_type *adc = config->base;
	int err;

	LOG_DBG("Initializing %s", dev->name);
	data->dev = dev;

	/*
	 * For series that use common channels for sampling time, all
	 * conversion time for all channels on one ADC instance has to
	 * be the same.
	 * For series that use two common channels, there can be up to two
	 * conversion times selected for all channels in a sequence.
	 * This additional table is for checking that the conversion time
	 * selection of all channels respects these requirements.
	 */
	data->acq_time_index[0] = -1;
	data->acq_time_index[1] = -1;

	adc_at32_set_clock(dev);

	/* Configure ADC inputs as specified in Device Tree, if any */
	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if ((err < 0) && (err != -ENOENT)) {
		/*
		 * If the ADC is used only with internal channels, then no pinctrl is
		 * provided in Device Tree, and pinctrl_apply_state returns -ENOENT,
		 * but this should not be treated as an error.
		 */
		LOG_ERR("ADC pinctrl setup failed (%d)", err);
		return err;
	}

#ifdef CONFIG_ADC_AT32_DMA
	if ((data->dma.dma_dev != NULL) &&
	    !device_is_ready(data->dma.dma_dev)) {
		LOG_ERR("%s device not ready", data->dma.dma_dev->name);
		return -ENODEV;
	}
	adc_at32_enable_dma_support(adc);
#endif
	if (config->irq_cfg_func) {
		config->irq_cfg_func();
	}
    adc_at32_disable(adc);
#if defined(HAS_CALIBRATION)
	adc_at32_calibrate(dev);
	adc_ordinary_conversion_trigger_set(adc, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
#endif /* HAS_CALIBRATION */

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(adc, api_at32_driver_api) = {
	.channel_setup = adc_at32_channel_setup,
	.read = adc_at32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_at32_read_async,
#endif
	.ref_internal = AT32_ADC_VREF_MV, /* VREF is usually connected to VDD */
};

#define ADC_AT32_DIV(x)	DT_INST_PROP(x, at_adc_prescaler)


#if defined(CONFIG_ADC_AT32_DMA)
#if 1
#define ADC_DMA_CHANNEL_INIT(index, src_dev, dest_dev)					\
	.dma = {									\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, rx)),		\
		.channel = DT_INST_DMAS_CELL_BY_NAME(index, rx, channel),			\
		.dma_cfg = {								\
			.dma_slot = COND_CODE_1(                                           \
				DT_HAS_COMPAT_STATUS_OKAY(at_at32_dma),             \
				(DT_INST_DMAS_CELL_BY_NAME(index, rx, slot)), (0)),     \
			.channel_direction = AT32_DMA_CONFIG_DIRECTION(		\
				DT_INST_DMAS_CELL_BY_IDX(index, 0, config)),		\
			.source_data_size = AT32_DMA_CONFIG_##src_dev##_WIDTH(	\
				DT_INST_DMAS_CELL_BY_IDX(index, 0, config)),		\
			.dest_data_size = AT32_DMA_CONFIG_##dest_dev##_WIDTH(	\
				DT_INST_DMAS_CELL_BY_IDX(index, 0, config)),		\
			.source_burst_length = 1,       /* SINGLE transfer */		\
			.dest_burst_length = 1,         /* SINGLE transfer */		\
			.channel_priority = AT32_DMA_CONFIG_PRIORITY(	      \
				DT_INST_DMAS_CELL_BY_IDX(index, 0, config)),      \
			.dma_callback = dma_callback,					\
			.block_count = 1,						\
		},									\
		.src_addr_increment = AT32_DMA_CONFIG_##src_dev##_ADDR_INC(		\
			DT_INST_DMAS_CELL_BY_IDX(index, 0, config)),                 \
		.dst_addr_increment = AT32_DMA_CONFIG_##dest_dev##_ADDR_INC(		\
			DT_INST_DMAS_CELL_BY_IDX(index, 0, config)),                 \
	}
#endif
#if 0
#define ADC_DMA_CHANNEL_INIT(index, src_dev, dest_dev)					\
	.dma = {									\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, rx)),		\
		.channel = DT_INST_DMAS_CELL_BY_NAME(index, rx, channel),			\
		.dma_cfg = {								\
			.dma_slot = COND_CODE_1(                                           \
				DT_HAS_COMPAT_STATUS_OKAY(at_at32_dma),             \
				(DT_INST_DMAS_CELL_BY_NAME(index, rx, slot)), (0)),     \
			.channel_direction = AT32_DMA_CONFIG_DIRECTION(		\
				DT_INST_DMAS_CELL_BY_IDX(index, 0, config)),		\
			.source_data_size = 2,\
			.dest_data_size = 2,\
			.source_burst_length = 1,       /* SINGLE transfer */		\
			.dest_burst_length = 1,         /* SINGLE transfer */		\
			.channel_priority = 1,      \
			.dma_callback = dma_callback,					\
			.block_count = 1,						\
		},									\
		.src_addr_increment = 0,			\
		.dst_addr_increment = 1,			\
	}
#endif
__maybe_unused static void adc_at32_irq_cfg(void)
{
}

__maybe_unused static void adc_at32_irq_handler(const struct device *dev)
{
}

#else /* CONFIG_ADC_AT32_DMA */

#define HANDLE_SHARED_IRQ(n, active_irq)							\
	static const struct device *const dev_##n = DEVICE_DT_INST_GET(n);			\
	const struct adc_at32_cfg *cfg_##n = dev_##n->config;				\
												\
	if ((cfg_##n->irq_num == active_irq)) {					\
		adc_at32_isr(dev_##n);								\
	}

static void adc_at32_irq_handler(const struct device *dev)
{
	const struct adc_at32_cfg *config = dev->config;

	LOG_DBG("global irq handler: %u", config->irq_num);

	DT_INST_FOREACH_STATUS_OKAY_VARGS(HANDLE_SHARED_IRQ, (config->irq_num));
}


static void adc_at32_irq_cfg(void)
{
#if ADC1_ENABLE
	/* Shared irq config default to adc0. */
	IRQ_CONNECT(DT_IRQN(ADC1_NODE),
		DT_IRQ(ADC1_NODE, priority),
		adc_at32_irq_handler,
		DEVICE_DT_GET(ADC1_NODE),
		0);
	irq_enable(DT_IRQN(ADC1_NODE));
#elif ADC2_ENABLE
	IRQ_CONNECT(DT_IRQN(ADC2_NODE),
		DT_IRQ(ADC2_NODE, priority),
		adc_at32_irq_handler,
		DEVICE_DT_GET(ADC2_NODE),
		0);
	irq_enable(DT_IRQN(ADC2_NODE));
#endif
}
#endif /* CONFIG_ADC_AT32_DMA */

#define ADC_DMA_CHANNEL(id, src, dest)							\
	COND_CODE_1(DT_INST_DMAS_HAS_IDX(id, 0),					\
			(ADC_DMA_CHANNEL_INIT(id, src, dest)),				\
			(/* Required for other adc instances without dma */))

#define ADC_AT32_INIT(index)						\
									\
PINCTRL_DT_INST_DEFINE(index);						\
									\
static const struct adc_at32_cfg adc_at32_cfg_##index = {		\
	.reg = DT_INST_REG_ADDR(index),			\
	.base = (adc_type *)DT_INST_REG_ADDR(index),			\
	.irq_cfg_func = adc_at32_irq_cfg,				\
	.irq_num = DT_INST_IRQN(index),							\
	.clkid = DT_INST_CLOCKS_CELL(index, id),					\
	.clk_prescaler = DT_INST_PROP(index, at_adc_prescaler),			\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),			\
	.sequencer_type = DT_INST_STRING_UPPER_TOKEN(index, at_adc_sequencer),	\
	.sampling_time_table = DT_INST_PROP(index, sampling_times),	\
	.num_sampling_time_common_channels =				\
		DT_INST_PROP_OR(index, num_sampling_time_common_channels, 0),\
	.res_table_size = DT_INST_PROP_LEN(index, resolutions),		\
	.res_table = DT_INST_PROP(index, resolutions),			\
};									\
									\
static struct adc_at32_data adc_at32_data_##index = {			\
	ADC_CONTEXT_INIT_TIMER(adc_at32_data_##index, ctx),		\
	ADC_CONTEXT_INIT_LOCK(adc_at32_data_##index, ctx),		\
	ADC_CONTEXT_INIT_SYNC(adc_at32_data_##index, ctx),		\
	ADC_DMA_CHANNEL(index, PERIPH, MEMORY)			\
};									\
									\
PM_DEVICE_DT_INST_DEFINE(index, adc_at32_pm_action);			\
									\
DEVICE_DT_INST_DEFINE(index,						\
		    &adc_at32_init, PM_DEVICE_DT_INST_GET(index),	\
		    &adc_at32_data_##index, &adc_at32_cfg_##index,	\
		    POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,		\
		    &api_at32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_AT32_INIT)
