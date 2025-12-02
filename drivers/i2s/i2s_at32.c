/*
 * Copyright (c) 2025 Maxjta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT at_at32_i2s

#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma/dma_at32.h>
#include <soc.h>
#include <at32_spi.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/cache.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2s_at32);

struct queue_item {
	void *mem_block;
	size_t size;
};

/* Device constant configuration parameters */
struct i2s_at32_cfg {
	spi_type *i2s;
	uint16_t clkid;
	size_t pclk_len;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(const struct device *dev);
	bool master_clk_sel: 1;
	bool ioswp: 1;
};

struct stream {
	int32_t state;
	struct k_msgq *msgq;

	const struct device *dev_dma;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	uint8_t fifo_threshold;
	bool tx_stop_for_drain;

	struct i2s_config cfg;
	void *mem_block;
	bool last_block;
	bool master;
	int (*stream_start)(struct stream *, const struct device *dev);
	void (*stream_disable)(struct stream *, const struct device *dev);
};

/* Device run time data */
struct i2s_at32_data {
	struct stream rx;
	struct stream tx;
};

/* checks that DMA Tx packet is fully transmitted over the I2S */
static inline uint32_t ll_func_i2s_dma_busy(spi_type *i2s)
{
	/* the I2S Tx empty and busy flags are needed */
	return (spi_i2s_flag_get(i2s, SPI_I2S_TDBE_FLAG) &&
		!spi_i2s_flag_get(i2s, SPI_I2S_BF_FLAG));
}

static unsigned int div_round_closest(uint32_t dividend, uint32_t divisor)
{
	return (dividend + (divisor / 2U)) / divisor;
}

static bool queue_is_empty(struct k_msgq *q)
{
	return (k_msgq_num_used_get(q) == 0) ? true : false;
}

/*
 * Get data from the queue
 */
static int queue_get(struct k_msgq *q, void **mem_block, size_t *size, int32_t timeout)
{
	struct queue_item item;
	int result = k_msgq_get(q, &item, SYS_TIMEOUT_MS(timeout));

	if (result == 0) {
		*mem_block = item.mem_block;
		*size = item.size;
	}
	return result;
}

/*
 * Put data in the queue
 */
static int queue_put(struct k_msgq *q, void *mem_block, size_t size, int32_t timeout)
{
	struct queue_item item = {.mem_block = mem_block, .size = size};

	return k_msgq_put(q, &item, SYS_TIMEOUT_MS(timeout));
}

static void stream_queue_drop(struct stream *s)
{
	size_t size;
	void *mem_block;

	while (queue_get(s->msgq, &mem_block, &size, 0) == 0) {
		k_mem_slab_free(s->cfg.mem_slab, mem_block);
	}
}

static int i2s_at32_enable_clock(const struct device *dev)
{
	const struct i2s_at32_cfg *cfg = dev->config;

	(void)clock_control_on(AT32_CLOCK_CONTROLLER,
		(clock_control_subsys_t)&cfg->clkid);

	return 0;
}

static int i2s_at32_set_clock(const struct device *dev,
			       uint32_t bit_clk_freq)
{
	const struct i2s_at32_cfg *cfg = dev->config;
	uint32_t freq_in = 0U;
	uint8_t i2s_div, i2s_odd;

	/* Handle multiple clock sources */
	if (clock_control_get_rate(AT32_CLOCK_CONTROLLER,
				   (clock_control_subsys_t)&cfg->clkid,
				   &freq_in) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(clkid[1])");
		return -EIO;
	}
	/*
	 * The ratio between input clock (I2SxClk) and output
	 * clock on the pad (I2S_CK) is obtained using the
	 * following formula:
	 *   (i2s_div * 2) + i2s_odd
	 */
	i2s_div = div_round_closest(freq_in, bit_clk_freq);
	i2s_odd = (i2s_div & 0x1) ? 1 : 0;
	i2s_div >>= 1;

	/* i2s_div == 0 || i2s_div == 1 are forbidden */
	if (i2s_div < 2U) {
		LOG_ERR("The linear prescaler value is unsupported");
		return -EINVAL;
	}

	LOG_DBG("i2s_div: %d - i2s_odd: %d", i2s_div, i2s_odd);

	cfg->i2s->i2sclk_bit.i2sodd = i2s_odd;
	if (i2s_div > 0x00FF) {
		cfg->i2s->i2sclk_bit.i2sdiv_h = (i2s_div >> 8) & 0x0003;
		cfg->i2s->i2sclk_bit.i2sdiv_l = i2s_div & 0x00FF;
	} else {
		cfg->i2s->i2sclk_bit.i2sdiv_h = 0;
		cfg->i2s->i2sclk_bit.i2sdiv_l = i2s_div;
	}

	return 0;
}

static int i2s_at32_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
	const struct i2s_at32_cfg *const cfg = dev->config;
	struct i2s_at32_data *const dev_data = dev->data;
	/* For words greater than 16-bit the channel length is considered 32-bit */
	const uint32_t channel_length = i2s_cfg->word_size > 16U ? 32U : 16U;
	/*
	 * comply with the i2s_config driver remark:
	 * When I2S data format is selected parameter channels is ignored,
	 * number of words in a frame is always 2.
	 */
	const uint32_t num_channels =
		((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) == I2S_FMT_DATA_FORMAT_I2S)
			? 2U
			: i2s_cfg->channels;
	struct stream *stream;
	uint32_t bit_clk_freq;
	bool enable_mck;
	int ret;

	if (dir == I2S_DIR_RX) {
		stream = &dev_data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &dev_data->tx;
	} else if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
	} else {
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	if (stream->state != I2S_STATE_NOT_READY &&
	    stream->state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}

	stream->master = true;
	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE ||
	    i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
		stream->master = false;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		stream_queue_drop(stream);
		memset(&stream->cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	/* conditions to enable master clock output */
	enable_mck = stream->master && cfg->master_clk_sel;

	/* set I2S bitclock */
	bit_clk_freq = i2s_cfg->frame_clk_freq *
		       channel_length * num_channels;

	if (enable_mck) {
		/*
		 * Compensate for the master clock dividers.
		 * MCK = N * CK, where N:
		 * 8 when the channel frame is 16-bit wide
		 * 4 when the channel frame is 32-bit wide
		 */
		bit_clk_freq *= channel_length == 16U ? 4U * 2U : 4U;
	}

	ret = i2s_at32_set_clock(dev, bit_clk_freq);
	if (ret < 0) {
		return ret;
	}

	/* set I2S Master Clock output in the MCK pin, enabled in the DT */
	if (enable_mck) {
		cfg->i2s->i2sclk_bit.i2smclkoe = TRUE;
	} else {
		cfg->i2s->i2sclk_bit.i2smclkoe = FALSE;
	}

	/*
	 * set I2S Data Format
	 * 16-bit data extended on 32-bit channel length excluded
	 */
	if (i2s_cfg->word_size == 16U) {
		cfg->i2s->i2sctrl_bit.i2scbn = 0;
		cfg->i2s->i2sctrl_bit.i2sdbn = 0;
	} else if (i2s_cfg->word_size == 24U) {
		cfg->i2s->i2sctrl_bit.i2scbn = 1;
		cfg->i2s->i2sctrl_bit.i2sdbn = 1;
	} else if (i2s_cfg->word_size == 32U) {
		cfg->i2s->i2sctrl_bit.i2scbn = 1;
		cfg->i2s->i2sctrl_bit.i2sdbn = 2;
	} else {
		LOG_ERR("invalid word size");
		return -EINVAL;
	}

	/* set I2S Standard */
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		cfg->i2s->i2sctrl_bit.pcmfssel = 0;
		cfg->i2s->i2sctrl_bit.stdsel = 0;
		break;

	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		cfg->i2s->i2sctrl_bit.pcmfssel = 0;
		cfg->i2s->i2sctrl_bit.stdsel = 3;
		break;

	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		cfg->i2s->i2sctrl_bit.pcmfssel = 1;
		cfg->i2s->i2sctrl_bit.stdsel = 3;
		break;

	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		cfg->i2s->i2sctrl_bit.pcmfssel = 0;
		cfg->i2s->i2sctrl_bit.stdsel = 1;
		break;

	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		cfg->i2s->i2sctrl_bit.pcmfssel = 0;
		cfg->i2s->i2sctrl_bit.stdsel = 2;
		break;

	default:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}

	/* set I2S clock polarity */
	if ((i2s_cfg->format & I2S_FMT_CLK_FORMAT_MASK) == I2S_FMT_BIT_CLK_INV) {
		cfg->i2s->i2sctrl_bit.i2sclkpol = TRUE;
	} else {
		cfg->i2s->i2sctrl_bit.i2sclkpol = FALSE;
	}

	stream->state = I2S_STATE_READY;
	return 0;
}

static const struct i2s_config *i2s_at32_config_get(const struct device *dev,
						     enum i2s_dir dir)
{
	struct i2s_at32_data *const dev_data = dev->data;
	struct stream *stream = NULL;

	if (dir == I2S_DIR_RX) {
		stream = &dev_data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &dev_data->tx;
	}

	if (stream != NULL && stream->state != I2S_STATE_NOT_READY) {
		return &stream->cfg;
	}

	return NULL;
}

static int i2s_at32_trigger(const struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
	struct i2s_at32_data *const dev_data = dev->data;
	const struct i2s_at32_cfg *const cfg = dev->config;
	struct stream *stream;
	unsigned int key;
	int ret;

	if (dir == I2S_DIR_RX) {
		stream = &dev_data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &dev_data->tx;
	} else if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
	} else {
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d",
				    stream->state);
			return -EIO;
		}

		__ASSERT_NO_MSG(stream->mem_block == NULL);

		ret = stream->stream_start(stream, dev);
		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}

		stream->state = I2S_STATE_RUNNING;
		stream->last_block = false;
		break;

	case I2S_TRIGGER_STOP:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("STOP trigger: invalid state");
			return -EIO;
		}
do_trigger_stop:
		if (ll_func_i2s_dma_busy(cfg->i2s)) {
			stream->state = I2S_STATE_STOPPING;
			/*
			 * Indicate that the transition to I2S_STATE_STOPPING
			 * is triggered by STOP command
			 */
			stream->tx_stop_for_drain = false;
		} else {
			stream->stream_disable(stream, dev);
			stream->state = I2S_STATE_READY;
			stream->last_block = true;
		}
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("DRAIN trigger: invalid state");
			return -EIO;
		}

		if (dir == I2S_DIR_TX) {
			if ((queue_is_empty(stream->msgq) == false) ||
						(ll_func_i2s_dma_busy(cfg->i2s))) {
				stream->state = I2S_STATE_STOPPING;
				/*
				 * Indicate that the transition to I2S_STATE_STOPPING
				 * is triggered by DRAIN command
				 */
				stream->tx_stop_for_drain = true;
			} else {
				stream->stream_disable(stream, dev);
				stream->state = I2S_STATE_READY;
			}
		} else if (dir == I2S_DIR_RX) {
			goto do_trigger_stop;
		} else {
			LOG_ERR("Unavailable direction");
			return -EINVAL;
		}
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state");
			return -EIO;
		}
		stream->stream_disable(stream, dev);
		stream_queue_drop(stream);
		stream->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state");
			return -EIO;
		}
		stream->state = I2S_STATE_READY;
		stream_queue_drop(stream);
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}

static int i2s_at32_read(const struct device *dev, void **mem_block,
			  size_t *size)
{
	struct i2s_at32_data *const dev_data = dev->data;
	int ret;

	if (dev_data->rx.state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	/* Get data from the beginning of RX queue */
	ret = queue_get(dev_data->rx.msgq, mem_block, size, dev_data->rx.cfg.timeout);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}

static int i2s_at32_write(const struct device *dev, void *mem_block,
			   size_t size)
{
	struct i2s_at32_data *const dev_data = dev->data;

	if (dev_data->tx.state != I2S_STATE_RUNNING &&
	    dev_data->tx.state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	/* Add data to the end of the TX queue */
	return queue_put(dev_data->tx.msgq, mem_block, size, dev_data->tx.cfg.timeout);
}

static DEVICE_API(i2s, i2s_at32_driver_api) = {
	.configure = i2s_at32_configure,
	.config_get = i2s_at32_config_get,
	.read = i2s_at32_read,
	.write = i2s_at32_write,
	.trigger = i2s_at32_trigger,
};

#define AT32_DMA_NUM_CHANNELS		7
static const struct device *active_dma_rx_channel[AT32_DMA_NUM_CHANNELS];
static const struct device *active_dma_tx_channel[AT32_DMA_NUM_CHANNELS];

static int reload_dma(const struct device *dev_dma, uint32_t channel,
		      struct dma_config *dcfg, void *src, void *dst,
		      uint32_t blk_size)
{
	int ret;

	ret = dma_reload(dev_dma, channel, (uint32_t)src, (uint32_t)dst, blk_size);
	if (ret < 0) {
		return ret;
	}

	ret = dma_start(dev_dma, channel);

	return ret;
}

static int start_dma(const struct device *dev_dma, uint32_t channel,
		     struct dma_config *dcfg, void *src,
		     bool src_addr_increment, void *dst,
		     bool dst_addr_increment, uint8_t fifo_threshold,
		     uint32_t blk_size)
{
	struct dma_block_config blk_cfg;
	int ret;

	memset(&blk_cfg, 0, sizeof(blk_cfg));
	blk_cfg.block_size = blk_size;
	blk_cfg.source_address = (uint32_t)src;
	blk_cfg.dest_address = (uint32_t)dst;
	if (src_addr_increment) {
		blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}
	if (dst_addr_increment) {
		blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}
	blk_cfg.fifo_mode_control = fifo_threshold;

	dcfg->head_block = &blk_cfg;

	ret = dma_config(dev_dma, channel, dcfg);
	if (ret < 0) {
		return ret;
	}

	ret = dma_start(dev_dma, channel);

	return ret;
}

static const struct device *get_dev_from_rx_dma_channel(uint32_t dma_channel);
static const struct device *get_dev_from_tx_dma_channel(uint32_t dma_channel);
static void rx_stream_disable(struct stream *stream, const struct device *dev);
static void tx_stream_disable(struct stream *stream, const struct device *dev);

/* This function is executed in the interrupt context */
static void dma_rx_callback(const struct device *dma_dev, void *arg,
			    uint32_t channel, int status)
{
	const struct device *dev = get_dev_from_rx_dma_channel(channel);
	const struct i2s_at32_cfg *cfg = dev->config;
	struct i2s_at32_data *const dev_data = dev->data;
	struct stream *stream = &dev_data->rx;
	void *mblk_tmp;
	int ret;

	if (status < 0) {
		ret = -EIO;
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	__ASSERT_NO_MSG(stream->mem_block != NULL);

	/* Stop reception if there was an error */
	if (stream->state == I2S_STATE_ERROR) {
		goto rx_disable;
	}

	mblk_tmp = stream->mem_block;

	/* Prepare to receive the next data block */
	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	ret = reload_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			(void *)&(cfg->i2s->dt),
			stream->mem_block,
			stream->cfg.block_size);
	if (ret < 0) {
		LOG_DBG("Failed to start RX DMA transfer: %d", ret);
		goto rx_disable;
	}

	/* Assure cache coherency after DMA write operation */
	sys_cache_data_invd_range(mblk_tmp, stream->cfg.block_size);

	/* All block data received */
	ret = queue_put(stream->msgq, mblk_tmp,
			stream->cfg.block_size, 0);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	/* Stop reception if we were requested */
	if (stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
		goto rx_disable;
	}

	return;

rx_disable:
	rx_stream_disable(stream, dev);
}

static void dma_tx_callback(const struct device *dma_dev, void *arg,
			    uint32_t channel, int status)
{
	const struct device *dev = get_dev_from_tx_dma_channel(channel);
	const struct i2s_at32_cfg *cfg = dev->config;
	struct i2s_at32_data *const dev_data = dev->data;
	struct stream *stream = &dev_data->tx;
	size_t mem_block_size;
	int ret;

	if (status < 0) {
		ret = -EIO;
		stream->state = I2S_STATE_ERROR;
		goto tx_disable;
	}

	__ASSERT_NO_MSG(stream->mem_block != NULL);

	/* All block data sent */
	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
	stream->mem_block = NULL;

	/* Stop transmission if there was an error */
	if (stream->state == I2S_STATE_ERROR) {
		LOG_ERR("TX error detected");
		goto tx_disable;
	}

	/* Check if we finished transferring one block and stopping is requested */
	if ((stream->state == I2S_STATE_STOPPING) && (status == DMA_STATUS_COMPLETE)) {
		/*
		 * Check if all tx samples have been completely handled
		 * as stated in zephyr i2s specification, in case of DRAIN command
		 * send all data in the transmit queue and stop the transmission.
		 */
		if (queue_is_empty(stream->msgq) == true) {
			stream_queue_drop(stream);
			stream->state = I2S_STATE_READY;
			goto tx_disable;
		} else if (stream->tx_stop_for_drain == false) {
			/*
			 * In case of STOP command, just stop the transmission
			 * at the current. The transmission can be resumed.
			 */
			stream->state = I2S_STATE_READY;
			goto tx_disable;
		}
		/* else: DRAIN trigger -> continue TX normally until queue is empty */
	}

	/* Stop transmission if we were requested */
	if (stream->last_block) {
		stream->state = I2S_STATE_READY;
		goto tx_disable;
	}

	/* Prepare to send the next data block */
	ret = queue_get(stream->msgq, &stream->mem_block,
			&mem_block_size, 0);
	if (ret < 0) {
		if (stream->state == I2S_STATE_STOPPING) {
			stream->state = I2S_STATE_READY;
		} else {
			stream->state = I2S_STATE_ERROR;
		}
		goto tx_disable;
	}

	/* Assure cache coherency before DMA read operation */
	sys_cache_data_flush_range(stream->mem_block, mem_block_size);

	ret = reload_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			stream->mem_block,
			(void *)&(cfg->i2s->dt),
			mem_block_size);
	if (ret < 0) {
		LOG_DBG("Failed to start TX DMA transfer: %d", ret);
		goto tx_disable;
	}

	return;

tx_disable:
	if ((stream->cfg.options & I2S_OPT_BIT_CLK_GATED) != 0) {
		tx_stream_disable(stream, dev);
	}
}

static uint32_t i2s_at32_irq_count;
static uint32_t i2s_at32_irq_ovr_count;
static uint32_t i2s_at32_irq_udr_count;

static void i2s_at32_isr(const struct device *dev)
{
	const struct i2s_at32_cfg *cfg = dev->config;

	/* OVR error must be explicitly cleared */
	if (spi_i2s_flag_get(cfg->i2s, SPI_I2S_ROERR_FLAG)) {
		i2s_at32_irq_ovr_count++;
		spi_i2s_flag_clear(cfg->i2s, SPI_I2S_ROERR_FLAG);
	}

	if (spi_i2s_flag_get(cfg->i2s, I2S_TUERR_FLAG)) {
		i2s_at32_irq_udr_count++;
		spi_i2s_flag_clear(cfg->i2s, I2S_TUERR_FLAG);
	}

	i2s_at32_irq_count++;
}

static int i2s_at32_initialize(const struct device *dev)
{
	const struct i2s_at32_cfg *cfg = dev->config;
	struct i2s_at32_data *const dev_data = dev->data;
	struct stream *stream = &dev_data->tx;
	int ret, i;

	/* Initialize the variable used to handle the TX */
	stream->tx_stop_for_drain = false;

	/* Enable I2S clock propagation */
	ret = i2s_at32_enable_clock(dev);
	if (ret < 0) {
		LOG_ERR("%s: clock enabling failed: %d",  __func__, ret);
		return -EIO;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2S pinctrl setup failed (%d)", ret);
		return ret;
	}

	cfg->irq_config(dev);

	for (i = 0; i < AT32_DMA_NUM_CHANNELS; i++) {
		active_dma_rx_channel[i] = NULL;
		active_dma_tx_channel[i] = NULL;
	}

	/* Get the binding to the DMA device */
	if (!device_is_ready(dev_data->tx.dev_dma)) {
		LOG_ERR("%s device not ready", dev_data->tx.dev_dma->name);
		return -ENODEV;
	}
	if (!device_is_ready(dev_data->rx.dev_dma)) {
		LOG_ERR("%s device not ready", dev_data->rx.dev_dma->name);
		return -ENODEV;
	}

	LOG_INF("%s inited", dev->name);

	return 0;
}

static int rx_stream_start(struct stream *stream, const struct device *dev)
{
	const struct i2s_at32_cfg *cfg = dev->config;
	int ret;

	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}

	if (stream->master) {
		cfg->i2s->i2sctrl_bit.opersel = I2S_MODE_MASTER_RX;
	} else {
		cfg->i2s->i2sctrl_bit.opersel = I2S_MODE_SLAVE_RX;
	}

	/* remember active RX DMA channel (used in callback) */
	active_dma_rx_channel[stream->dma_channel] = dev;

	ret = start_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			(void *)&(cfg->i2s->dt),
			stream->src_addr_increment, stream->mem_block,
			stream->dst_addr_increment, stream->fifo_threshold,
			stream->cfg.block_size);
	if (ret < 0) {
		LOG_ERR("Failed to start RX DMA transfer: %d", ret);
		return ret;
	}

	spi_i2s_dma_receiver_enable(cfg->i2s, TRUE);

	spi_i2s_interrupt_enable(cfg->i2s, SPI_I2S_ERROR_INT, TRUE);
	i2s_enable(cfg->i2s, TRUE);

	return 0;
}

static int tx_stream_start(struct stream *stream, const struct device *dev)
{
	const struct i2s_at32_cfg *cfg = dev->config;
	size_t mem_block_size;
	int ret;

	ret = queue_get(stream->msgq, &stream->mem_block,
			&mem_block_size, 0);
	if (ret < 0) {
		return ret;
	}

	/* Assure cache coherency before DMA read operation */
	sys_cache_data_flush_range(stream->mem_block, mem_block_size);

	if (stream->master) {
		cfg->i2s->i2sctrl_bit.opersel = I2S_MODE_MASTER_TX;
	} else {
		cfg->i2s->i2sctrl_bit.opersel = I2S_MODE_SLAVE_TX;
	}

	/* remember active TX DMA channel (used in callback) */
	active_dma_tx_channel[stream->dma_channel] = dev;

	ret = start_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			stream->mem_block, stream->src_addr_increment,
			(void *)&(cfg->i2s->dt),
			stream->dst_addr_increment, stream->fifo_threshold,
			stream->cfg.block_size);
	if (ret < 0) {
		LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		return ret;
	}

	spi_i2s_dma_transmitter_enable(cfg->i2s, TRUE);

	spi_i2s_interrupt_enable(cfg->i2s, SPI_I2S_ERROR_INT, TRUE);
	i2s_enable(cfg->i2s, TRUE);

	return 0;
}

static void rx_stream_disable(struct stream *stream, const struct device *dev)
{
	const struct i2s_at32_cfg *cfg = dev->config;

	spi_i2s_dma_receiver_enable(cfg->i2s, FALSE);
	spi_i2s_interrupt_enable(cfg->i2s, SPI_I2S_ERROR_INT, FALSE);
	dma_stop(stream->dev_dma, stream->dma_channel);
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	i2s_enable(cfg->i2s, FALSE);

	active_dma_rx_channel[stream->dma_channel] = NULL;
}

static void tx_stream_disable(struct stream *stream, const struct device *dev)
{
	const struct i2s_at32_cfg *cfg = dev->config;

	spi_i2s_dma_transmitter_enable(cfg->i2s, FALSE);
	spi_i2s_interrupt_enable(cfg->i2s, SPI_I2S_ERROR_INT, FALSE);

	dma_stop(stream->dev_dma, stream->dma_channel);
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	/* Wait for TX queue to drain before disabling */
	k_busy_wait(100);
	i2s_enable(cfg->i2s, FALSE);

	active_dma_tx_channel[stream->dma_channel] = NULL;
}

static const struct device *get_dev_from_rx_dma_channel(uint32_t dma_channel)
{
	return active_dma_rx_channel[dma_channel];
}

static const struct device *get_dev_from_tx_dma_channel(uint32_t dma_channel)
{
	return active_dma_tx_channel[dma_channel];
}

/* src_dev and dest_dev should be 'MEMORY' or 'PERIPHERAL'. */
#define I2S_DMA_CHANNEL_INIT(index, dir, dir_cap, src_dev, dest_dev)		\
	.dir = {								\
		.dev_dma = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, dir)),		\
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),	\
		.dma_cfg = {							\
			.block_count = 2,					\
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, dir, slot),		\
			.channel_direction = src_dev##_TO_##dest_dev,		\
			.source_data_size = 2,  /* 16bit default */		\
			.dest_data_size = 2,    /* 16bit default */		\
			.source_burst_length = 1, /* SINGLE transfer */		\
			.dest_burst_length = 1,					\
			.channel_priority = AT32_DMA_CONFIG_PRIORITY(		\
				DT_INST_DMAS_CELL_BY_NAME(index, dir, config)),		\
			.dma_callback = dma_##dir##_callback,			\
		},								\
		.src_addr_increment = AT32_DMA_CONFIG_##src_dev##_ADDR_INC(	\
				DT_INST_DMAS_CELL_BY_NAME(index, dir, config)),	\
		.dst_addr_increment = AT32_DMA_CONFIG_##dest_dev##_ADDR_INC(	\
				DT_INST_DMAS_CELL_BY_NAME(index, dir, config)),	\
		.fifo_threshold = 0,	\
		.stream_start = dir##_stream_start,				\
		.stream_disable = dir##_stream_disable,				\
		.msgq = &dir##_##index##_queue,					\
	}

#define I2S_AT32_INIT(index)							\
										\
	static void i2s_at32_irq_config_func_##index(const struct device *dev);\
										\
	PINCTRL_DT_INST_DEFINE(index);						\
										\
	static const struct i2s_at32_cfg i2s_at32_config_##index = {		\
		.i2s = (spi_type *)DT_INST_REG_ADDR(index),			\
		.clkid = DT_INST_CLOCKS_CELL(index, id),				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),			\
		.irq_config = i2s_at32_irq_config_func_##index,		\
		.master_clk_sel = 0,		\
		.ioswp = 0,				\
	};									\
										\
	K_MSGQ_DEFINE(rx_##index##_queue, sizeof(struct queue_item),		\
		      CONFIG_I2S_AT32_RX_BLOCK_COUNT, 4);			\
	K_MSGQ_DEFINE(tx_##index##_queue, sizeof(struct queue_item),		\
		      CONFIG_I2S_AT32_TX_BLOCK_COUNT, 4);			\
										\
	static struct i2s_at32_data i2s_at32_data_##index = {			\
		IF_ENABLED(DT_INST_DMAS_HAS_NAME(index, rx),			\
			   (I2S_DMA_CHANNEL_INIT(index, rx, RX, PERIPHERAL, MEMORY))),\
		IF_ENABLED(DT_INST_DMAS_HAS_NAME(index, tx),			\
			   (I2S_DMA_CHANNEL_INIT(index, tx, TX, MEMORY, PERIPHERAL))),\
	};									\
	DEVICE_DT_INST_DEFINE(index,						\
			      &i2s_at32_initialize, NULL,			\
			      &i2s_at32_data_##index,				\
			      &i2s_at32_config_##index, POST_KERNEL,		\
			      CONFIG_I2S_INIT_PRIORITY, &i2s_at32_driver_api);	\
										\
	static void i2s_at32_irq_config_func_##index(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(index),				\
			    DT_INST_IRQ(index, priority),			\
			    i2s_at32_isr, DEVICE_DT_INST_GET(index), 0);	\
		irq_enable(DT_INST_IRQN(index));				\
	}

DT_INST_FOREACH_STATUS_OKAY(I2S_AT32_INIT)
