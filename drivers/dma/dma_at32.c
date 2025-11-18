/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>

#include <at32_dma.h>
#include <zephyr/irq.h>


#if DT_HAS_COMPAT_STATUS_OKAY(at_at32_dma)
#define DT_DRV_COMPAT at_at32_dma
#endif


#define AT32_DMA_CH(dma, ch) ((dma_channel_type *)((dma + 0x08UL) + 0x14UL * (uint32_t)(ch)))
#define AT32_DMA(dma)	     ((dma_type *)(dma + 0x00UL))

#define DMA_MUX_CHANNEL_OFFSET1    0x0104
#define DMA_MUX_CHANNEL_OFFSET2    0x0108
#define DMA_MUX_CHANNEL_OFFSET3    0x010C
#define DMA_MUX_CHANNEL_OFFSET4    0x0110
#define DMA_MUX_CHANNEL_OFFSET5    0x0114
#define DMA_MUX_CHANNEL_OFFSET6    0x0118
#define DMA_MUX_CHANNEL_OFFSET7    0x011C

#define DMA_CHANNEL_OFFSET1    0x0008
#define DMA_CHANNEL_OFFSET2    0x001C
#define DMA_CHANNEL_OFFSET3    0x0030
#define DMA_CHANNEL_OFFSET4    0x0044
#define DMA_CHANNEL_OFFSET5    0x0058
#define DMA_CHANNEL_OFFSET6    0x006C
#define DMA_CHANNEL_OFFSET7    0x0080

#define dma_channel_enum uint8_t

#define DMA_FLAG_ADD(flag, shift)          ((uint32_t)flag << ((uint32_t)(shift) * 4U))    /*!< DMA channel flag shift */
#define CH_OFFSET(ch) (ch)

LOG_MODULE_REGISTER(dma_at32, CONFIG_DMA_LOG_LEVEL);

struct dma_at32_config {
	uint32_t reg;
	uint32_t channels;
	uint16_t clkid;
	bool mem2mem;
#if DT_HAS_COMPAT_STATUS_OKAY(at_at32_dma)
	struct reset_dt_spec reset;
#endif
	void (*irq_configure)(void);
};

struct dma_at32_channel {
	dma_callback_t callback;
	void *user_data;
	uint32_t direction;
	bool busy;
};

struct dma_at32_data {
	struct dma_context ctx;
	struct dma_at32_channel *channels;
};

struct dma_at32_srcdst_config {
	uint32_t addr;
	uint32_t adj;
	uint32_t width;
};

static  uint32_t  dma_at32_dmamux(uint32_t id)
{
  static const uint32_t dmamux_offset [] = {
    DMA_MUX_CHANNEL_OFFSET1,
	  DMA_MUX_CHANNEL_OFFSET2,
	  DMA_MUX_CHANNEL_OFFSET3,
	  DMA_MUX_CHANNEL_OFFSET4,
	  DMA_MUX_CHANNEL_OFFSET5,
	  DMA_MUX_CHANNEL_OFFSET6,
	  DMA_MUX_CHANNEL_OFFSET7,		
  	};
	__ASSERT_NO_MSG(id < ARRAY_SIZE(dmamux_offset));
	return dmamux_offset[id];
}

static  uint32_t  dma_at32_channel(uint32_t id)
{
  static const uint32_t dmamux_offset [] = {
	  DMA_CHANNEL_OFFSET1,
	  DMA_CHANNEL_OFFSET2,
	  DMA_CHANNEL_OFFSET3,
	  DMA_CHANNEL_OFFSET4,
	  DMA_CHANNEL_OFFSET5,
	  DMA_CHANNEL_OFFSET6,
	  DMA_CHANNEL_OFFSET7,		
  };
	__ASSERT_NO_MSG(id < ARRAY_SIZE(dmamux_offset));
	return dmamux_offset[id];
}

/*
 * Register access functions
 */

static inline void
at32_dma_periph_increase_enable(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.pincm = TRUE;
}

static inline void
at32_dma_periph_increase_disable(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.pincm = FALSE;
}

static inline void
at32_dma_transfer_set_memory_to_memory(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl &= 0xbfef;
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl |= DMA_DIR_MEMORY_TO_MEMORY;
}

static inline void
at32_dma_transfer_set_memory_to_periph(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl &= 0xbfef;
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl |= DMA_DIR_MEMORY_TO_PERIPHERAL;
}

static inline void
at32_dma_transfer_set_periph_to_memory(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl &= 0xbfef;
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl |= DMA_DIR_PERIPHERAL_TO_MEMORY;
}

static inline void
at32_dma_memory_increase_enable(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.mincm = TRUE;
}

static inline void
at32_dma_memory_increase_disable(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.mincm = FALSE;
}

static inline void
at32_dma_circulation_enable(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.lm |= TRUE;
}

static inline void
at32_dma_circulation_disable(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.lm |= FALSE;
}

static inline void at32_dma_channel_enable(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.chen = TRUE;
}

static inline void at32_dma_channel_disable(uint32_t reg, dma_channel_enum ch)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.chen = FALSE;
}

static inline void
at32_dma_interrupt_enable(uint32_t reg, dma_channel_enum ch, uint32_t source)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl |= source;
}

static inline void
at32_dma_interrupt_disable(uint32_t reg, dma_channel_enum ch, uint32_t source)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl &= ~source;
}

static inline void
at32_dma_priority_config(uint32_t reg, dma_channel_enum ch, uint32_t priority)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.chpl = priority;
}

static inline void
at32_dma_memory_width_config(uint32_t reg, dma_channel_enum ch, uint32_t mwidth)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.mwidth = mwidth;
}

static inline void
at32_dma_periph_width_config(uint32_t reg, dma_channel_enum ch, uint32_t pwidth)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->ctrl_bit.pwidth = pwidth;
}

static inline void
at32_dma_periph_address_config(uint32_t reg, dma_channel_enum ch, uint32_t addr)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->paddr = addr;
}

static inline void
at32_dma_memory_address_config(uint32_t reg, dma_channel_enum ch, uint32_t addr)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->maddr = addr;
}

static inline void
at32_dma_transfer_number_config(uint32_t reg, dma_channel_enum ch, uint32_t num)
{
	AT32_DMA_CH(reg, CH_OFFSET(ch))->dtcnt_bit.cnt = num;
}

static inline uint32_t
at32_dma_transfer_number_get(uint32_t reg, dma_channel_enum ch)
{
	return AT32_DMA_CH(reg, CH_OFFSET(ch))->dtcnt_bit.cnt;
}

static inline void
at32_dma_interrupt_flag_clear(uint32_t reg, dma_channel_enum ch, uint32_t flag)
{
	AT32_DMA(reg)->clr = DMA_FLAG_ADD(flag, CH_OFFSET(ch));
}

static inline void
at32_dma_flag_clear(uint32_t reg, dma_channel_enum ch, uint32_t flag)
{
	AT32_DMA(reg)->clr = DMA_FLAG_ADD(flag, CH_OFFSET(ch));
}

static inline uint32_t
at32_dma_interrupt_flag_get(uint32_t reg, dma_channel_enum ch, uint32_t flag)
{
	return (AT32_DMA(reg)->sts & DMA_FLAG_ADD(flag, CH_OFFSET(ch)));
}

static inline void at32_dma_deinit(uint32_t reg, dma_channel_enum ch)
{
	dma_reset(AT32_DMA_CH(reg, CH_OFFSET(ch)));
}

/*
 * Utility functions
 */

static inline uint32_t dma_at32_priority(uint32_t prio)
{
	switch(prio)
	{
		case 0:
			return DMA_PRIORITY_LOW;
		case 1:
			return DMA_PRIORITY_MEDIUM;
		case 2:
			return DMA_PRIORITY_HIGH;
		case 3:
			return DMA_PRIORITY_VERY_HIGH;
		default:
			return DMA_PRIORITY_LOW;
	}
}

static inline uint32_t dma_at32_memory_width(uint32_t width)
{
	switch (width) {
	case 4:
		return (uint32_t)DMA_MEMORY_DATA_WIDTH_WORD;
	case 2:
		return (uint32_t)DMA_MEMORY_DATA_WIDTH_HALFWORD;
	default:
		return (uint32_t)DMA_MEMORY_DATA_WIDTH_BYTE;
	}
}

static inline uint32_t dma_at32_periph_width(uint32_t width)
{
	switch (width) {
	case 4:
		return (uint32_t)DMA_PERIPHERAL_DATA_WIDTH_BYTE;
	case 2:
		return (uint32_t)DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	default:
		return (uint32_t)DMA_PERIPHERAL_DATA_WIDTH_WORD;
	}
}

/*
 * API functions
 */

static int dma_at32_configure(const struct device *dev, uint32_t channel,
			   struct dma_config *dma_cfg)
{
	const struct dma_at32_config *cfg = dev->config;
	struct dma_at32_data *data = dev->data;
	struct dma_at32_srcdst_config src_cfg;
	struct dma_at32_srcdst_config dst_cfg;
	struct dma_at32_srcdst_config *memory_cfg = NULL;
	struct dma_at32_srcdst_config *periph_cfg = NULL;

	dma_init_type dma_init_struct;
	dma_type *dma = (dma_type *)cfg->reg;
	dma_channel_type *dma_channel = (dma_channel_type *)(cfg->reg + dma_at32_channel(CH_OFFSET(channel))) ;
	dmamux_channel_type *dmamux_channelx = (dmamux_channel_type *)(cfg->reg + dma_at32_dmamux(CH_OFFSET(channel)));
	dma_default_para_init(&dma_init_struct);
    
	if (CH_OFFSET(channel) >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}

	if (dma_cfg->block_count != 1) {
		LOG_ERR("chained block transfer not supported.");
		return -ENOTSUP;
	}

	if (dma_cfg->channel_priority > 3) {
		LOG_ERR("channel_priority must be < 4 (%" PRIu32 ")",
			dma_cfg->channel_priority);
		return -EINVAL;
	}

	if (dma_cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		LOG_ERR("source_addr_adj not supported DMA_ADDR_ADJ_DECREMENT");
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		LOG_ERR("dest_addr_adj not supported DMA_ADDR_ADJ_DECREMENT");
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid source_addr_adj %" PRIu16,
            dma_cfg->head_block->source_addr_adj);
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid dest_addr_adj %" PRIu16,
            dma_cfg->head_block->dest_addr_adj);
		return -ENOTSUP;
	}

	if (dma_cfg->source_data_size != 1 && dma_cfg->source_data_size != 2 &&
	    dma_cfg->source_data_size != 4) {
		LOG_ERR("source_data_size must be 1, 2, or 4 (%" PRIu32 ")",
        dma_cfg->source_data_size);
		return -EINVAL;
	}

	if (dma_cfg->dest_data_size != 1 && dma_cfg->dest_data_size != 2 &&
	    dma_cfg->dest_data_size != 4) {
		LOG_ERR("dest_data_size must be 1, 2, or 4 (%" PRIu32 ")",
			dma_cfg->dest_data_size);
		return -EINVAL;
	}

	if (dma_cfg->channel_direction > PERIPHERAL_TO_MEMORY) {
		LOG_ERR("channel_direction must be MEMORY_TO_MEMORY, "
			"MEMORY_TO_PERIPHERAL or PERIPHERAL_TO_MEMORY (%" PRIu32
			")",
			dma_cfg->channel_direction);
		return -ENOTSUP;
	}

	if (dma_cfg->channel_direction == MEMORY_TO_MEMORY && !cfg->mem2mem) {
		LOG_ERR("not supporting MEMORY_TO_MEMORY");
		return -ENOTSUP;
	}

    dma_reset(dma_channel);

	src_cfg.addr = dma_cfg->head_block->source_address;
	src_cfg.adj = dma_cfg->head_block->source_addr_adj;
	src_cfg.width = dma_cfg->source_data_size;

	dst_cfg.addr = dma_cfg->head_block->dest_address;
	dst_cfg.adj = dma_cfg->head_block->dest_addr_adj;
	dst_cfg.width = dma_cfg->dest_data_size;

	switch (dma_cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
		memory_cfg = &dst_cfg;
		periph_cfg = &src_cfg;
		dma_init_struct.direction = DMA_DIR_MEMORY_TO_MEMORY;

		break;
	case PERIPHERAL_TO_MEMORY:
		dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
		memory_cfg = &dst_cfg;
		periph_cfg = &src_cfg;
		break;
	case MEMORY_TO_PERIPHERAL:
		dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
		memory_cfg = &src_cfg;
		periph_cfg = &dst_cfg;
		break;
	}

	dma_init_struct.memory_base_addr = memory_cfg->addr;
	if (memory_cfg->adj == DMA_ADDR_ADJ_INCREMENT) {
        dma_init_struct.memory_inc_enable = TRUE;
	} else {
        dma_init_struct.memory_inc_enable = FALSE;
	}

	dma_init_struct.peripheral_base_addr = periph_cfg->addr;
	if (periph_cfg->adj == DMA_ADDR_ADJ_INCREMENT) {
        dma_init_struct.peripheral_inc_enable = TRUE;
	} else {
        dma_init_struct.peripheral_inc_enable = FALSE;
	}

	dma_init_struct.buffer_size = dma_cfg->head_block->block_size;
    dma_init_struct.priority = (dma_priority_level_type)dma_cfg->channel_priority;
    dma_init_struct.memory_data_width = dma_at32_memory_width(memory_cfg->width);
    dma_init_struct.peripheral_data_width = dma_at32_periph_width(periph_cfg->width);
	if( dma_cfg->head_block->source_reload_en)
	{
		dma_interrupt_enable(dma_channel, DMA_HDT_INT, TRUE);
		dma_init_struct.loop_mode_enable = TRUE;
	}
	else
	{
    dma_init_struct.loop_mode_enable = FALSE;
	}
  
	dma_init(dma_channel, &dma_init_struct);
	dma_interrupt_enable(dma_channel, DMA_FDT_INT, TRUE);
    dmamux_enable(dma, TRUE);
	dmamux_init(dmamux_channelx, dma_cfg->dma_slot);

	data->channels[CH_OFFSET(channel)].callback = dma_cfg->dma_callback;
	data->channels[CH_OFFSET(channel)].user_data = dma_cfg->user_data;
	data->channels[CH_OFFSET(channel)].direction = dma_cfg->channel_direction;

	return 0;
}

static int dma_at32_reload(const struct device *dev, uint32_t ch, uint32_t src,
			   uint32_t dst, size_t size)
{
	const struct dma_at32_config *cfg = dev->config;
	struct dma_at32_data *data = dev->data;

	dma_channel_type *dma_channel = (dma_channel_type *)(cfg->reg + dma_at32_channel(CH_OFFSET(ch))) ;

	if (CH_OFFSET(ch) >= cfg->channels) {
		LOG_ERR("reload channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, ch);
		return -EINVAL;
	}

	if (data->channels[CH_OFFSET(ch)].busy) {
		return -EBUSY;
	}

    dma_channel_enable(dma_channel, FALSE);
    dma_data_number_set(dma_channel, size);

	switch (data->channels[ch].direction) {
    case MEMORY_TO_MEMORY:
    case PERIPHERAL_TO_MEMORY:
        dma_channel->maddr = dst;
        dma_channel->paddr = src;
		break;
    case MEMORY_TO_PERIPHERAL:
        dma_channel->maddr = src;
        dma_channel->paddr = dst;
        break;
	}
    dma_channel_enable(dma_channel, TRUE);

	return 0;
}

static int dma_at32_start(const struct device *dev, uint32_t ch)
{
	const struct dma_at32_config *cfg = dev->config;
	struct dma_at32_data *data = dev->data;
	dma_channel_type *dma_channel = (dma_channel_type *)(cfg->reg + dma_at32_channel(CH_OFFSET(ch))) ;

	if (CH_OFFSET(ch) >= cfg->channels) {
		LOG_ERR("start channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, ch);
		return -EINVAL;
	}

	dma_channel_enable(dma_channel, TRUE);
	data->channels[CH_OFFSET(ch)].busy = true;

	return 0;
}

static int dma_at32_stop(const struct device *dev, uint32_t ch)
{
	const struct dma_at32_config *cfg = dev->config;
	struct dma_at32_data *data = dev->data;

	dma_type *dma = (dma_type *)cfg->reg;
	dma_channel_type *dma_channel = (dma_channel_type *)(cfg->reg + dma_at32_channel(CH_OFFSET(ch))) ;

	if (CH_OFFSET(ch) >= cfg->channels) {
		LOG_ERR("stop channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, ch);
		return -EINVAL;
	}

    dma_interrupt_enable(dma_channel, DMA_FDT_INT | DMA_HDT_INT, FALSE);
	dma->clr = DMA_FLAG_ADD(DMA1_FDT1_FLAG |DMA1_HDT1_FLAG, CH_OFFSET(ch));
	dma_channel_enable(dma_channel, FALSE);

	data->channels[CH_OFFSET(ch)].busy = false;

	return 0;
}

static int dma_at32_get_status(const struct device *dev, uint32_t ch,
			       struct dma_status *stat)
{
	const struct dma_at32_config *cfg = dev->config;
	struct dma_at32_data *data = dev->data;
	dma_channel_type *dma_channel = (dma_channel_type *)(cfg->reg + dma_at32_channel(CH_OFFSET(ch))) ;
	
	if (CH_OFFSET(ch) >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, ch);
		return -EINVAL;
	}

	stat->pending_length = dma_data_number_get(dma_channel);//at32_dma_transfer_number_get(cfg->reg, ch);
	stat->dir = data->channels[CH_OFFSET(ch)].direction;
	stat->busy = data->channels[CH_OFFSET(ch)].busy;

	return 0;
}

static bool dma_at32_api_chan_filter(const struct device *dev, int ch,
				     void *filter_param)
{
	uint32_t filter;

	if (!filter_param) {
		LOG_ERR("filter_param must not be NULL");
		return false;
	}

	filter = *((uint32_t *)filter_param);

	return (filter & BIT(ch));
}

static int dma_at32_init(const struct device *dev)
{
	const struct dma_at32_config *cfg = dev->config;

	(void)clock_control_on(AT32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid);

#if DT_HAS_COMPAT_STATUS_OKAY(at_at32_dma)
	(void)reset_line_toggle_dt(&cfg->reset);
#endif
	for (uint32_t i = 0; i < cfg->channels; i++) {
		at32_dma_interrupt_disable(cfg->reg, i,
			   DMA_FDT_INT | DMA_HDT_INT);
		at32_dma_deinit(cfg->reg, i);
	}

	cfg->irq_configure();

	return 0;
}

static void dma_at32_isr(const struct device *dev)
{
	const struct dma_at32_config *cfg = dev->config;
	struct dma_at32_data *data = dev->data;
	uint32_t errflag, ftfflag, htflag;
	int ret = 0;

	for (uint32_t i = 0; i < cfg->channels; i++) {

		errflag = at32_dma_interrupt_flag_get(cfg->reg, i,DMA1_DTERR1_FLAG);
		htflag = at32_dma_interrupt_flag_get(cfg->reg, i, DMA1_HDT1_FLAG);
		ftfflag = at32_dma_interrupt_flag_get(cfg->reg, i, DMA1_FDT1_FLAG);
		if (errflag == 0 && ftfflag == 0 && htflag) {
			continue;
		}
		if(errflag != 0)
		{
			ret = -EIO;
			data->channels[CH_OFFSET(i)].busy = false;
			at32_dma_interrupt_flag_clear(cfg->reg, i, DMA1_DTERR1_FLAG);
		}
		
		if(htflag != 0)
		{
			ret = DMA_STATUS_BLOCK;
			at32_dma_interrupt_flag_clear(cfg->reg, i,  DMA1_HDT1_FLAG);
		}
		
		if(ftfflag != 0)
		{
			ret = DMA_STATUS_COMPLETE;
			data->channels[CH_OFFSET(i)].busy = false;
			at32_dma_interrupt_flag_clear(cfg->reg, i, DMA1_FDT1_FLAG);
		}
		
		if (data->channels[CH_OFFSET(i)].callback) {
			data->channels[CH_OFFSET(i)].callback(dev, 
			data->channels[CH_OFFSET(i)].user_data, i, ret);
		}
	}
}

static DEVICE_API(dma, dma_at32_driver_api) = {
	.config = dma_at32_configure,
	.reload = dma_at32_reload,
	.start = dma_at32_start,
	.stop = dma_at32_stop,
	.get_status = dma_at32_get_status,
	.chan_filter = dma_at32_api_chan_filter,
};

#define IRQ_CONFIGURE(n, inst)                                                 \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq),                          \
		    DT_INST_IRQ_BY_IDX(inst, n, priority), dma_at32_isr,       \
		    DEVICE_DT_INST_GET(inst), 0);                              \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, n, irq));

#define CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, IRQ_CONFIGURE, (), inst)

#define AT32_DMA_INIT(inst)                                                    \
	static void dma_at32##inst##_irq_configure(void)                       \
	{                                                                      \
		CONFIGURE_ALL_IRQS(inst, DT_NUM_IRQS(DT_DRV_INST(inst)));      \
	}                                                                      \
	static const struct dma_at32_config dma_at32##inst##_config = {        \
		.reg = DT_INST_REG_ADDR(inst),                                 \
		.channels = DT_INST_PROP(inst, dma_channels),                  \
		.clkid = DT_INST_CLOCKS_CELL(inst, id),                        \
		.mem2mem = DT_INST_PROP(inst, at_mem2mem),                     \
		IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(at_at32_dma),          \
			   (.reset = RESET_DT_SPEC_INST_GET(inst),))           \
		.irq_configure = dma_at32##inst##_irq_configure,               \
	};                                                                     \
                                                                               \
	static struct dma_at32_channel                                         \
		dma_at32##inst##_channels[DT_INST_PROP(inst, dma_channels)];   \
	ATOMIC_DEFINE(dma_at32_atomic##inst,                                   \
		      DT_INST_PROP(inst, dma_channels));                       \
	static struct dma_at32_data dma_at32##inst##_data = {                  \
		.ctx =  {                                                      \
			.magic = DMA_MAGIC,                                    \
			.atomic = dma_at32_atomic##inst,                       \
			.dma_channels = DT_INST_PROP(inst, dma_channels),      \
		},                                                             \
		.channels = dma_at32##inst##_channels,                         \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst, &dma_at32_init, NULL,                      \
			      &dma_at32##inst##_data,                          \
			      &dma_at32##inst##_config, POST_KERNEL,           \
			      CONFIG_DMA_INIT_PRIORITY, &dma_at32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AT32_DMA_INIT)
