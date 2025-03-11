/*
 * Copyright (c) 2021 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT at_at32_i2c

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/i2c.h>

#include <at32_i2c.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>
#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include "zephyr/logging/log.h"
LOG_MODULE_REGISTER(i2c_at32, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

#ifndef CONFIG_I2C_AT32_V2_TIMING
#define CONFIG_I2C_AT32_V2_TIMING
#endif
#define AT32_I2C_TRANSFER_TIMEOUT_MSEC              500
#define OPERATION(msg) (((struct i2c_msg *) msg)->flags & I2C_MSG_RW_MASK)
#ifdef CONFIG_I2C_AT32_V2_TIMING
/* Use the algorithm to calcuate the I2C timing */
#ifndef AT32_I2C_VALID_TIMING_NBR
#define AT32_I2C_VALID_TIMING_NBR                 128U
#endif
#define AT32_I2C_SPEED_FREQ_STANDARD                0U    /* 100 kHz */
#define AT32_I2C_SPEED_FREQ_FAST                    1U    /* 400 kHz */
#define AT32_I2C_SPEED_FREQ_FAST_PLUS               2U    /* 1 MHz */
#define AT32_I2C_ANALOG_FILTER_DELAY_MIN            50U   /* ns */
#define AT32_I2C_ANALOG_FILTER_DELAY_MAX            260U  /* ns */
#define AT32_I2C_USE_ANALOG_FILTER                  1U
#define AT32_I2C_DIGITAL_FILTER_COEF                0U
#define AT32_I2C_PRESC_MAX                          256U
#define AT32_I2C_SCLDEL_MAX                         16U
#define AT32_I2C_SDADEL_MAX                         16U
#define AT32_I2C_SCLH_MAX                           256U
#define AT32_I2C_SCLL_MAX                           256U


/* I2C_DEVICE_Private_Types */
struct at32_i2c_charac_t {
	uint32_t freq;       /* Frequency in Hz */
	uint32_t freq_min;   /* Minimum frequency in Hz */
	uint32_t freq_max;   /* Maximum frequency in Hz */
	uint32_t hddat_min;  /* Minimum data hold time in ns */
	uint32_t vddat_max;  /* Maximum data valid time in ns */
	uint32_t sudat_min;  /* Minimum data setup time in ns */
	uint32_t lscl_min;   /* Minimum low period of the SCL clock in ns */
	uint32_t hscl_min;   /* Minimum high period of SCL clock in ns */
	uint32_t trise;      /* Rise time in ns */
	uint32_t tfall;      /* Fall time in ns */
	uint32_t dnf;        /* Digital noise filter coefficient */
};

struct at32_i2c_timings_t {
	uint32_t presc;      /* Timing prescaler */
	uint32_t tscldel;    /* SCL delay */
	uint32_t tsdadel;    /* SDA delay */
	uint32_t sclh;       /* SCL high period */
	uint32_t scll;       /* SCL low period */
};

/* I2C_DEVICE Private Constants */
static const struct at32_i2c_charac_t at32_i2c_charac[] = {
	[AT32_I2C_SPEED_FREQ_STANDARD] = {
		.freq = 100000,
		.freq_min = 80000,
		.freq_max = 120000,
		.hddat_min = 0,
		.vddat_max = 3450,
		.sudat_min = 250,
		.lscl_min = 4700,
		.hscl_min = 4000,
		.trise = 640,
		.tfall = 20,
		.dnf = AT32_I2C_DIGITAL_FILTER_COEF,
	},
	[AT32_I2C_SPEED_FREQ_FAST] = {
		.freq = 400000,
		.freq_min = 320000,
		.freq_max = 480000,
		.hddat_min = 0,
		.vddat_max = 900,
		.sudat_min = 100,
		.lscl_min = 1300,
		.hscl_min = 600,
		.trise = 250,
		.tfall = 100,
		.dnf = AT32_I2C_DIGITAL_FILTER_COEF,
	},
	[AT32_I2C_SPEED_FREQ_FAST_PLUS] = {
		.freq = 1000000,
		.freq_min = 800000,
		.freq_max = 1200000,
		.hddat_min = 0,
		.vddat_max = 450,
		.sudat_min = 50,
		.lscl_min = 500,
		.hscl_min = 260,
		.trise = 60,
		.tfall = 100,
		.dnf = AT32_I2C_DIGITAL_FILTER_COEF,
	},
};

static struct at32_i2c_timings_t i2c_valid_timing[AT32_I2C_VALID_TIMING_NBR];
static uint32_t i2c_valid_timing_nbr;
#endif /* CONFIG_I2C_AT32_V2_TIMING */

struct i2c_config_timing {
	/* i2c peripheral clock in Hz */
	uint32_t periph_clock;
	/* i2c bus speed in Hz */
	uint32_t i2c_speed;
	/* I2C_TIMINGR register value of i2c v2 peripheral */
	uint32_t timing_setting;
};

struct i2c_at32_config {
	uint32_t reg;
	uint32_t bitrate;
	uint16_t clkid;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_cfg_func)(void);
};

struct i2c_at32_data {
	struct k_sem bus_mutex;
	struct k_sem sync_sem;
	uint32_t dev_config;
	uint16_t addr1;
	uint16_t addr2;
	uint32_t xfer_len;
		struct {
#ifdef CONFIG_I2C_AT32_V1
		unsigned int is_restart;
		unsigned int flags;
#endif
		unsigned int is_write;
		unsigned int is_arlo;
		unsigned int is_nack;
		unsigned int is_err;
		struct i2c_msg *msg;
		unsigned int len;
		uint8_t *buf;
	} current;
	uint8_t errs;
	bool is_restart;
  
  struct i2c_config_timing current_timing;
#ifdef CONFIG_I2C_TARGET
	bool master_active;
	struct i2c_target_config *slave_cfg;
#ifdef CONFIG_I2C_AT32_V2
	struct i2c_target_config *slave2_cfg;
#endif
	bool slave_attached;
#endif
  bool is_configured;
	bool smbalert_active;
};

static int i2c_at32_configure(const struct device *dev, uint32_t dev_config);

int i2c_at32_get_config(const struct device *dev, uint32_t *config)
{
	struct i2c_at32_data *data = dev->data;

	if (!data->is_configured) {
		LOG_ERR("I2C controller not configured");
		return -EIO;
	}

	*config = data->dev_config;

#ifdef CONFIG_I2C_AT32_V2_TIMING
	/* Print the timing parameter of device data */
	LOG_INF("I2C timing value, report to the DTS :");

	/* I2C BIT RATE */
	if (data->current_timing.i2c_speed == 100000) {
		LOG_INF("timings = <%d I2C_BITRATE_STANDARD 0x%X>;",
			data->current_timing.periph_clock,
			data->current_timing.timing_setting);
	} else if (data->current_timing.i2c_speed == 400000) {
		LOG_INF("timings = <%d I2C_BITRATE_FAST 0x%X>;",
			data->current_timing.periph_clock,
			data->current_timing.timing_setting);
	} else if (data->current_timing.i2c_speed == 1000000) {
		LOG_INF("timings = <%d I2C_SPEED_FAST_PLUS 0x%X>;",
			data->current_timing.periph_clock,
			data->current_timing.timing_setting);
	}
#endif /* CONFIG_I2C_STM32_V2_TIMING */

	return 0;
}


static inline void msg_init(const struct device *dev, struct i2c_msg *msg,
			    uint8_t *next_msg_flags, uint16_t slave,
			    uint32_t transfer)
{
	const struct i2c_at32_config *cfg = dev->config;
	struct i2c_at32_data *data = dev->data;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
  
  if(i2c_x->ctrl2_bit.rlden)
  {
    i2c_cnt_set(i2c_x, msg->len);
  }
  else
  {
    if (I2C_ADDR_10_BITS & data->dev_config)
    {
      i2c_addr10_mode_enable(i2c_x, TRUE);
      i2c_transfer_addr_set(i2c_x, slave);
    }
    else
    {
      i2c_addr10_mode_enable(i2c_x, FALSE);
      i2c_transfer_addr_set(i2c_x, slave << 1);
    }
    if (!(msg->flags & I2C_MSG_STOP) && next_msg_flags &&
		    !(*next_msg_flags & I2C_MSG_RESTART)) {
			i2c_reload_enable(i2c_x, TRUE);
		} else {
			i2c_reload_enable(i2c_x, FALSE);
		}
    
    i2c_auto_stop_enable(i2c_x, FALSE);
    i2c_transfer_dir_set(i2c_x, (i2c_transfer_dir_type)transfer);
    i2c_cnt_set(i2c_x, msg->len);

#if defined(CONFIG_I2C_TARGET)
		data->master_active = true;
#endif
    i2c_enable(i2c_x, TRUE);
    i2c_start_generate(i2c_x);
    
  }
}

#ifdef CONFIG_I2C_AT32_INTERRUPT

static void at32_i2c_disable_transfer_interrupts(const struct device *dev)
{
	const struct i2c_at32_config *cfg = dev->config;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
  i2c_interrupt_enable(i2c_x, I2C_TD_INT | I2C_RD_INT | I2C_ACKFIAL_INT |
                       I2C_STOP_INT | I2C_TDC_INT | I2C_ERR_INT, FALSE);

}

static void at32_i2c_enable_transfer_interrupts(const struct device *dev)
{
	const struct i2c_at32_config *cfg = dev->config;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
  
  i2c_interrupt_enable(i2c_x,  I2C_ACKFIAL_INT |I2C_STOP_INT | 
                      I2C_TDC_INT | I2C_ERR_INT, TRUE);
}

static void at32_i2c_master_mode_end(const struct device *dev)
{
	const struct i2c_at32_config *cfg = dev->config;
	struct i2c_at32_data *data = dev->data;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;

	at32_i2c_disable_transfer_interrupts(dev);

	if (i2c_x->ctrl2_bit.rlden) {
		i2c_reload_enable(i2c_x, FALSE);
	}

#if defined(CONFIG_I2C_TARGET)
	data->master_active = false;
	if (!data->slave_attached && !data->smbalert_active) {
		i2c_enable(i2c_x, FALSE);
	}
#else
	if (!data->smbalert_active) {
		i2c_enable(i2c_x, FALSE);
	}
#endif
	k_sem_give(&data->sync_sem);
}

#if defined(CONFIG_I2C_TARGET)
static void at32_i2c_slave_event(const struct device *dev)
{
	const struct i2c_at32_config *cfg = dev->config;
	struct i2c_at32_data *data = dev->data;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
	const struct i2c_target_callbacks *slave_cb;
	struct i2c_target_config *slave_cfg;

	if (data->slave_cfg->flags != I2C_TARGET_FLAGS_ADDR_10_BITS) {
		uint8_t slave_address;

		/* Choose the right slave from the address match code */
		slave_address = i2c_matched_addr_get(i2c_x);
		if (data->slave_cfg != NULL &&
				slave_address == data->slave_cfg->address) {
			slave_cfg = data->slave_cfg;
		} else if (data->slave2_cfg != NULL &&
				slave_address == data->slave2_cfg->address) {
			slave_cfg = data->slave2_cfg;
		} else {
			__ASSERT_NO_MSG(0);
			return;
		}
	} else {
		if (data->slave_cfg != NULL) {
			slave_cfg = data->slave_cfg;
		} else {
			__ASSERT_NO_MSG(0);
			return;
		}
	}

	slave_cb = slave_cfg->callbacks;

	if (i2c_flag_get(i2c_x, I2C_TDIS_FLAG)) {
		uint8_t val;

		slave_cb->read_processed(slave_cfg, &val);
    i2c_data_send(i2c_x, val);
		return;
	}

	if (i2c_flag_get(i2c_x, I2C_RDBF_FLAG)) {
		uint8_t val = i2c_data_receive(i2c_x);

		if (slave_cb->write_received(slave_cfg, val)) {
      i2c_ack_enable(hi2c->i2cx, FALSE);
		}
		return;
	}

	if (i2c_flag_get(i2c_x, I2C_ACKFAIL_FLAG)) {
		i2c_flag_clear(i2c_x, I2C_ACKFAIL_FLAG);
	}

	if (i2c_flag_get(i2c_x, I2C_STOPF_FLAG)) {
		at32_i2c_disable_transfer_interrupts(dev);

		/* Flush remaining TX byte before clearing Stop Flag */
    i2c_flag_clear(i2c_x, I2C_TDBE_FLAG);
    i2c_flag_clear(i2c_x, I2C_STOPF_FLAG);

		slave_cb->stop(slave_cfg);

		/* Prepare to ACK next transmissions address byte */
    i2c_ack_enable(hi2c->i2cx, TRUE);
	}

	if (i2c_flag_get(i2c_x, I2C_ADDRF_FLAG))) {
		i2c_transfer_dir_type dir;

    i2c_flag_clear(i2c_x, I2C_ADDRF_FLAG);

    dir = i2c_transfer_dir_get(i2c_x);
		if (dir == I2C_DIR_TRANSMIT) {
			slave_cb->write_requested(slave_cfg);
      i2c_interrupt_enable(i2c_x, I2C_RD_INT, TRUE);
		} else {
			uint8_t val;

			slave_cb->read_requested(slave_cfg, &val);
			i2c_data_send(i2c_x, val);
      i2c_interrupt_enable(i2c_x, I2C_TD_INT, TRUE);
		}

		at32_i2c_enable_transfer_interrupts(dev);
	}
}

/* Attach and start I2C as target */
int i2c_at32_target_register(const struct device *dev,
			     struct i2c_target_config *config)
{
	const struct i2c_at32_config *cfg = dev->config;
	struct i2c_at32_data *data = dev->data;
  i2c_type *i2c_x = (i2c_type *)cfg->reg;
	uint32_t bitrate_cfg;
	int ret;

	if (!config) {
		return -EINVAL;
	}

	if (data->slave_cfg && data->slave2_cfg) {
		return -EBUSY;
	}

	if (data->master_active) {
		return -EBUSY;
	}

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);

	ret = i2c_at32_configure(dev, bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("i2c: failure initializing");
		return ret;
	}

#if defined(CONFIG_PM_DEVICE_RUNTIME)
	if (pm_device_wakeup_is_capable(dev)) {
		/* Mark device as active */
		(void)pm_device_runtime_get(dev);
		/* Enable wake-up from stop */
		LOG_DBG("i2c: enabling wakeup from stop");
//		LL_I2C_EnableWakeUpFromStop(cfg->i2c);
	}
#endif /* defined(CONFIG_PM_DEVICE_RUNTIME) */

  i2c_enable(i2c_x, TRUE);

	if (!data->slave_cfg) {
		data->slave_cfg = config;
		if (data->slave_cfg->flags == I2C_TARGET_FLAGS_ADDR_10_BITS)	{
      i2c_own_address1_set(i2c_x, config->address, I2C_ADDRESS_MODE_10BIT);
			LOG_DBG("i2c: target #1 registered with 10-bit address");
		} else {
      i2c_own_address1_set(i2c_x, config->address << 1, I2C_ADDRESS_MODE_7BIT);
			LOG_DBG("i2c: target #1 registered with 7-bit address");
		}
		LOG_DBG("i2c: target #1 registered");
	} else {
		data->slave2_cfg = config;

		if (data->slave2_cfg->flags == I2C_TARGET_FLAGS_ADDR_10_BITS)	{
			return -EINVAL;
		}
    
    i2c_own_address2_set(i2c_x, config->address << 1U, I2C_ADDR2_NOMASK);
    i2c_own_address2_enable(i2c_x, TRUE);
    
		LOG_DBG("i2c: target #2 registered");
	}

	data->slave_attached = true;
  i2c_interrupt_enable(i2c_x, I2C_ADDR_INT, TRUE);

	return 0;
}

int i2c_at32_target_unregister(const struct device *dev,
			       struct i2c_target_config *config)
{
	const struct i2c_at32_config *cfg = dev->config;
	struct i2c_at32_data *data = dev->data;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;

	if (!data->slave_attached) {
		return -EINVAL;
	}

	if (data->master_active) {
		return -EBUSY;
	}

	if (config == data->slave_cfg) {
    i2c_x->oaddr1_bit.addr1en = FALSE;
		data->slave_cfg = NULL;

		LOG_DBG("i2c: slave #1 unregistered");
	} else if (config == data->slave2_cfg) {
    i2c_own_address2_enable(i2c_x, FALSE);
		data->slave2_cfg = NULL;

		LOG_DBG("i2c: slave #2 unregistered");
	} else {
		return -EINVAL;
	}

	/* Return if there is a slave remaining */
	if (data->slave_cfg || data->slave2_cfg) {
		LOG_DBG("i2c: target#%c still registered", data->slave_cfg?'1':'2');
		return 0;
	}

	/* Otherwise disable I2C */
  i2c_interrupt_enable(i2c_x, I2C_ADDR_INT, FALSE);
	at32_i2c_disable_transfer_interrupts(dev);
  
  i2c_flag_clear(i2c_x, I2C_ACKFAIL_FLAG | I2C_ADDRF_FLAG | I2C_STOPF_FLAG);

	if (!data->smbalert_active) {
		i2c_enable(i2c_x, FALSE);
	}

#if defined(CONFIG_PM_DEVICE_RUNTIME)
	if (pm_device_wakeup_is_capable(dev)) {
		/* Disable wake-up from STOP */
		LOG_DBG("i2c: disabling wakeup from stop");
//		LL_I2C_DisableWakeUpFromStop(i2c);
		/* Release the device */
		(void)pm_device_runtime_put(dev);
	}
#endif /* defined(CONFIG_PM_DEVICE_RUNTIME) */

	data->slave_attached = false;

	return 0;
}

#endif /* defined(CONFIG_I2C_TARGET) */


static void at32_i2c_event(const struct device *dev)
{
	const struct i2c_at32_config *cfg = dev->config;
	struct i2c_at32_data *data = dev->data;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
#if defined(CONFIG_I2C_TARGET)
	if (data->slave_attached && !data->master_active) {
		at32_i2c_slave_event(dev);
		return;
	}
#endif
	if (data->current.len) {
		/* Send next byte */
		if (i2c_flag_get(i2c_x, I2C_TDBE_FLAG)) {
			i2c_data_send(i2c_x, *data->current.buf);
		}

		/* Receive next byte */
		if (i2c_flag_get(i2c_x, I2C_RDBF_FLAG)) {
			*data->current.buf = i2c_data_receive(i2c_x);
		}

		data->current.buf++;
		data->current.len--;
	}

	/* NACK received */
	if (i2c_flag_get(i2c_x, I2C_ACKFAIL_FLAG)) {
		i2c_flag_clear(i2c_x, I2C_ACKFAIL_FLAG);
		data->current.is_nack = 1U;
		/*
		 * AutoEndMode is always disabled in master mode,
		 * so send a stop condition manually
		 */
    i2c_stop_generate(i2c_x);
		return;
	}

	/* STOP received */
	if (i2c_flag_get(i2c_x, I2C_STOPF_FLAG)) {
    i2c_flag_clear(i2c_x, I2C_STOPF_FLAG);
    i2c_reload_enable(i2c_x, FALSE);
		goto end;
	}

	/* Transfer Complete or Transfer Complete Reload */
	if (i2c_flag_get(i2c_x, I2C_TDC_FLAG) ||
	    i2c_flag_get(i2c_x, I2C_TCRLD_FLAG)) {
		/* Issue stop condition if necessary */
		if (data->current.msg->flags & I2C_MSG_STOP) {
			i2c_stop_generate(i2c_x);
		} else {
			at32_i2c_disable_transfer_interrupts(dev);
			k_sem_give(&data->sync_sem);
		}
	}

	return;
end:
	at32_i2c_master_mode_end(dev);
}

static int at32_i2c_error(const struct device *dev)
{
	const struct i2c_at32_config *cfg = dev->config;
	struct i2c_at32_data *data = dev->data;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;

#if defined(CONFIG_I2C_TARGET)
	if (data->slave_attached && !data->master_active) {
		/* No need for a slave error function right now. */
		return 0;
	}
#endif

	if (i2c_flag_get(i2c_x, I2C_ARLOST_FLAG)) {
		i2c_flag_clear(i2c_x, I2C_ARLOST_FLAG);
		data->current.is_arlo = 1U;
		goto end;
	}

	if (i2c_flag_get(i2c_x, I2C_BUSERR_FLAG)) {
        i2c_flag_clear(i2c_x, I2C_BUSERR_FLAG);
		data->current.is_err = 1U;
		goto end;
	}

#if defined(CONFIG_SMBUS_AT32_SMBALERT)
	if (i2c_flag_get(i2c_x, I2C_ALERTF_FLAG)) {
		i2c_flag_clear(i2c_x, I2C_ALERTF_FLAG);
		if (data->smbalert_cb_func != NULL) {
			data->smbalert_cb_func(data->smbalert_cb_dev);
		}
		goto end;
	}
#endif

	return 0;
end:
	at32_i2c_master_mode_end(dev);
	return -EIO;
}

#ifdef CONFIG_I2C_AT32_COMBINED_INTERRUPT
void at32_i2c_combined_isr(void *arg)
{
	const struct device *dev = (const struct device *) arg;

	if (at32_i2c_error(dev)) {
		return;
	}
	at32_i2c_event(dev);
}
#else

void at32_i2c_event_isr(void *arg)
{
	const struct device *dev = (const struct device *) arg;
	at32_i2c_event(dev);
}

void at32_i2c_error_isr(void *arg)
{
	const struct device *dev = (const struct device *) arg;

	at32_i2c_error(dev);
}
#endif


static int at32_i2c_msg_write(const struct device *dev, struct i2c_msg *msg,
			uint8_t *next_msg_flags, uint16_t slave)
{
	const struct i2c_at32_config *cfg = dev->config;
	struct i2c_at32_data *data = dev->data;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
	bool is_timeout = false;
	data->current.len = msg->len;
	data->current.buf = msg->buf;
	data->current.is_write = 1U;
	data->current.is_nack = 0U;
	data->current.is_err = 0U;
	data->current.msg = msg;
	msg_init(dev, msg, next_msg_flags, slave, (uint32_t)I2C_DIR_TRANSMIT);

	at32_i2c_enable_transfer_interrupts(dev);
    i2c_interrupt_enable(i2c_x, I2C_TD_INT, TRUE);
	
	if (k_sem_take(&data->sync_sem,
		       K_MSEC(AT32_I2C_TRANSFER_TIMEOUT_MSEC)) != 0) {
		at32_i2c_master_mode_end(dev);
		k_sem_take(&data->sync_sem, K_FOREVER);
		is_timeout = true;
	}

	if (data->current.is_nack || data->current.is_err ||
	    data->current.is_arlo || is_timeout) {
		goto error;
	}

	return 0;
error:
	if (data->current.is_arlo) {
		LOG_DBG("%s: ARLO %d", __func__,
				    data->current.is_arlo);
		data->current.is_arlo = 0U;
	}

	if (data->current.is_nack) {
		LOG_DBG("%s: NACK", __func__);
		data->current.is_nack = 0U;
	}

	if (data->current.is_err) {
		LOG_DBG("%s: ERR %d", __func__,
				    data->current.is_err);
		data->current.is_err = 0U;
	}

	if (is_timeout) {
		LOG_DBG("%s: TIMEOUT", __func__);
	}
	return -EIO;
}

static int at32_i2c_msg_read(const struct device *dev, struct i2c_msg *msg,
		       uint8_t *next_msg_flags, uint16_t slave)
{
	const struct i2c_at32_config *cfg = dev->config;
	struct i2c_at32_data *data = dev->data;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
	bool is_timeout = false;
	data->current.len = msg->len;
	data->current.buf = msg->buf;
	data->current.is_write = 0U;
	data->current.is_arlo = 0U;
	data->current.is_err = 0U;
	data->current.is_nack = 0U;
	data->current.msg = msg;

	msg_init(dev, msg, next_msg_flags, slave, I2C_DIR_RECEIVE);

	at32_i2c_enable_transfer_interrupts(dev);
	i2c_interrupt_enable(i2c_x, I2C_RD_INT, TRUE);

	if (k_sem_take(&data->sync_sem,
		       K_MSEC(AT32_I2C_TRANSFER_TIMEOUT_MSEC)) != 0) {
		at32_i2c_master_mode_end(dev);
		k_sem_take(&data->sync_sem, K_FOREVER);
		is_timeout = true;
	}

	if (data->current.is_nack || data->current.is_err ||
	    data->current.is_arlo || is_timeout) {
		goto error;
	}

	return 0;
error:
	if (data->current.is_arlo) {
		LOG_DBG("%s: ARLO %d", __func__,
				    data->current.is_arlo);
		data->current.is_arlo = 0U;
	}

	if (data->current.is_nack) {
		LOG_DBG("%s: NACK", __func__);
		data->current.is_nack = 0U;
	}

	if (data->current.is_err) {
		LOG_DBG("%s: ERR %d", __func__,
				    data->current.is_err);
		data->current.is_err = 0U;
	}

	if (is_timeout) {
		LOG_DBG("%s: TIMEOUT", __func__);
	}

	return -EIO;
}
#else
static inline int check_errors(const struct device *dev, const char *funcname)
{
	const struct i2c_at32_config *cfg = dev->config;
    i2c_type *i2c_x = (i2c_type *)cfg->reg;

	if (i2c_flag_get(i2c_x, I2C_ACKFAIL_FLAG)) {
		i2c_flag_clear(i2c_x, I2C_ACKFAIL_FLAG);
		LOG_DBG("%s: NACK", funcname);
		goto error;
	}

	if (i2c_flag_get(i2c_x, I2C_ARLOST_FLAG)) {
		i2c_flag_clear(i2c_x, I2C_ARLOST_FLAG);
		LOG_DBG("%s: ARLO", funcname);
		goto error;
	}

	if (i2c_flag_get(i2c_x, I2C_OUF_FLAG)) {
		i2c_flag_clear(i2c_x, I2C_OUF_FLAG);
		LOG_DBG("%s: OVR", funcname);
		goto error;
	}

	if (i2c_flag_get(i2c_x, I2C_BUSERR_FLAG)) {
		i2c_flag_clear(i2c_x, I2C_BUSERR_FLAG);
		LOG_DBG("%s: BERR", funcname);
		goto error;
	}

	return 0;
error:
	if (i2c_x->ctrl2_bit.rlden) {
    i2c_reload_enable(i2c_x, FALSE);
	}
	return -EIO;
}

static inline int msg_done(const struct device *dev,
			   unsigned int current_msg_flags)
{
	const struct i2c_at32_config *cfg = dev->config;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;

	/* Wait for transfer to complete */
	while (!i2c_flag_get(i2c_x, I2C_TDC_FLAG) && !i2c_flag_get(i2c_x, I2C_TCRLD_FLAG)) {
		if (check_errors(dev, __func__)) {
			return -EIO;
		}
	}
	/* Issue stop condition if necessary */
	if (current_msg_flags & I2C_MSG_STOP) {
		i2c_stop_generate(i2c_x);
		while (!i2c_flag_get(i2c_x, I2C_STOPF_FLAG)) {
		}

		i2c_flag_clear(i2c_x, I2C_STOPF_FLAG);
		i2c_reload_enable(i2c_x, FALSE);
	}

	return 0;
}

static int at32_i2c_msg_write(const struct device *dev, struct i2c_msg *msg,
			uint8_t *next_msg_flags, uint16_t slave)
{
	const struct i2c_at32_config *cfg = dev->config;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
	unsigned int len = 0U;
	uint8_t *buf = msg->buf;

	msg_init(dev, msg, next_msg_flags, slave, I2C_DIR_TRANSMIT);

	len = msg->len;
	while (len) {
		while (1) {
			if (i2c_flag_get(i2c_x, I2C_TDBE_FLAG)) {
				break;
			}

			if (check_errors(dev, __func__)) {
				return -EIO;
			}
		}

		i2c_data_send(i2c_x, *buf);
		buf++;
		len--;
	}

	return msg_done(dev, msg->flags);
}

static int at32_i2c_msg_read(const struct device *dev, struct i2c_msg *msg,
		       uint8_t *next_msg_flags, uint16_t slave)
{
	const struct i2c_at32_config *cfg = dev->config;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
	unsigned int len = 0U;
	uint8_t *buf = msg->buf;

	msg_init(dev, msg, next_msg_flags, slave, I2C_DIR_RECEIVE);

	len = msg->len;
	while (len) {
		while (!i2c_flag_get(i2c_x, I2C_RDBF_FLAG)) {
			if (check_errors(dev, __func__)) {
				return -EIO;
			}
		}

		*buf = i2c_data_receive(i2c_x);
		buf++;
		len--;
	}

	return msg_done(dev, msg->flags);
}
#endif
static int at32_i2c_transaction(const struct device *dev,
						  struct i2c_msg msg, uint8_t *next_msg_flags,
						  uint16_t periph)
{
	const uint32_t i2c_at32_maxchunk = 255U;
	const uint8_t saved_flags = msg.flags;
	uint8_t combine_flags =
		saved_flags & ~(I2C_MSG_STOP | I2C_MSG_RESTART);
	uint8_t *flagsp = NULL;
	uint32_t rest = msg.len;
	int ret = 0;

	do { /* do ... while to allow zero-length transactions */
		if (msg.len > i2c_at32_maxchunk) {
			msg.len = i2c_at32_maxchunk;
			msg.flags &= ~I2C_MSG_STOP;
			flagsp = &combine_flags;
		} else {
			msg.flags = saved_flags;
			flagsp = next_msg_flags;
		}
		if ((msg.flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			ret = at32_i2c_msg_write(dev, &msg, flagsp, periph);
		} else {
			ret = at32_i2c_msg_read(dev, &msg, flagsp, periph);
		}
		if (ret < 0) {
			break;
		}
		rest -= msg.len;
		msg.buf += msg.len;
		msg.len = rest;
	} while (rest > 0U);

	return ret;
}


static int i2c_at32_transfer(const struct device *dev,
			     struct i2c_msg *msgs,
			     uint8_t num_msgs,
			     uint16_t slave)
{
	struct i2c_at32_data *data = dev->data;
	struct i2c_msg *current, *next;
	int ret = 0;
  
    current = msgs;
    current->flags |= I2C_MSG_RESTART;
  
    for (uint8_t i = 1; i <= num_msgs; i++) {
        if (i < num_msgs) {
			next = current + 1;

			/*
			 * Restart condition between messages
			 * of different directions is required
			 */
			if (OPERATION(current) != OPERATION(next)) {
				if (!(next->flags & I2C_MSG_RESTART)) {
					ret = -EINVAL;
					break;
				}
			}

			/* Stop condition is only allowed on last message */
			if (current->flags & I2C_MSG_STOP) {
				ret = -EINVAL;
				break;
			}
		}
    }
  
    /* Send out messages */
	k_sem_take(&data->bus_mutex, K_FOREVER);
  
  /* Prevent the clocks to be stopped during the i2c transaction */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
  
    current = msgs;
  
    while (num_msgs > 0) {
		uint8_t *next_msg_flags = NULL;

		if (num_msgs > 1) {
			next = current + 1;
			next_msg_flags = &(next->flags);
		}
		ret = at32_i2c_transaction(dev, *current, next_msg_flags, slave);
		if (ret < 0) {
			break;
		}
		current++;
		num_msgs--;
	}

	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_put(dev);
#endif

	k_sem_give(&data->bus_mutex);

	return ret;
	
}
#ifdef CONFIG_I2C_AT32_V2_TIMING

/*
 * Macro used to fix the compliance check warning :
 * "DEEP_INDENTATION: Too many leading tabs - consider code refactoring
 * in the i2c_compute_scll_sclh() function below
 */
#define I2C_LOOP_SCLH();						\
	if ((tscl >= clk_min) &&					\
		(tscl <= clk_max) &&					\
		(tscl_h >= at32_i2c_charac[i2c_speed].hscl_min) &&	\
		(ti2cclk < tscl_h)) {					\
									\
		int32_t error = (int32_t)tscl - (int32_t)ti2cspeed;	\
									\
		if (error < 0) {					\
			error = -error;					\
		}							\
									\
		if ((uint32_t)error < prev_error) {			\
			prev_error = (uint32_t)error;			\
			i2c_valid_timing[count].scll = scll;		\
			i2c_valid_timing[count].sclh = sclh;		\
			ret = count;					\
		}							\
	}

/*
 * @brief  Calculate SCLL and SCLH and find best configuration.
 * @param  clock_src_freq I2C source clock in Hz.
 * @param  i2c_speed I2C frequency (index).
 * @retval config index (0 to I2C_VALID_TIMING_NBR], 0xFFFFFFFF for no valid config.
 */
static uint32_t i2c_compute_scll_sclh(uint32_t clock_src_freq, uint32_t i2c_speed)
{
	uint32_t ret = 0xFFFFFFFFU;
	uint32_t ti2cclk;
	uint32_t ti2cspeed;
	uint32_t prev_error;
	uint32_t dnf_delay;
	uint32_t clk_min, clk_max;
	uint32_t scll, sclh;
	uint32_t tafdel_min;

	ti2cclk = (NSEC_PER_SEC + (clock_src_freq / 2U)) / clock_src_freq;
	ti2cspeed = (NSEC_PER_SEC + (at32_i2c_charac[i2c_speed].freq / 2U)) /
		at32_i2c_charac[i2c_speed].freq;

	tafdel_min = (AT32_I2C_USE_ANALOG_FILTER == 1U) ?
		AT32_I2C_ANALOG_FILTER_DELAY_MIN :
		0U;

	/* tDNF = DNF x tI2CCLK */
	dnf_delay = at32_i2c_charac[i2c_speed].dnf * ti2cclk;

	clk_max = NSEC_PER_SEC / at32_i2c_charac[i2c_speed].freq_min;
	clk_min = NSEC_PER_SEC / at32_i2c_charac[i2c_speed].freq_max;

	prev_error = ti2cspeed;

	for (uint32_t count = 0; count < AT32_I2C_VALID_TIMING_NBR; count++) {
		/* tPRESC = (PRESC+1) x tI2CCLK*/
		uint32_t tpresc = (i2c_valid_timing[count].presc + 1U) * ti2cclk;

		for (scll = 0; scll < AT32_I2C_SCLL_MAX; scll++) {
			/* tLOW(min) <= tAF(min) + tDNF + 2 x tI2CCLK + [(SCLL+1) x tPRESC ] */
			uint32_t tscl_l = tafdel_min + dnf_delay +
				(2U * ti2cclk) + ((scll + 1U) * tpresc);

			/*
			 * The I2CCLK period tI2CCLK must respect the following conditions:
			 * tI2CCLK < (tLOW - tfilters) / 4 and tI2CCLK < tHIGH
			 */
			if ((tscl_l > at32_i2c_charac[i2c_speed].lscl_min) &&
				(ti2cclk < ((tscl_l - tafdel_min - dnf_delay) / 4U))) {
				for (sclh = 0; sclh < AT32_I2C_SCLH_MAX; sclh++) {
					/*
					 * tHIGH(min) <= tAF(min) + tDNF +
					 * 2 x tI2CCLK + [(SCLH+1) x tPRESC]
					 */
					uint32_t tscl_h = tafdel_min + dnf_delay +
						(2U * ti2cclk) + ((sclh + 1U) * tpresc);

					/* tSCL = tf + tLOW + tr + tHIGH */
					uint32_t tscl = tscl_l +
						tscl_h + at32_i2c_charac[i2c_speed].trise +
					at32_i2c_charac[i2c_speed].tfall;

					/* get timings with the lowest clock error */
					I2C_LOOP_SCLH();
				}
			}
		}
	}

	return ret;
}

/*
 * Macro used to fix the compliance check warning :
 * "DEEP_INDENTATION: Too many leading tabs - consider code refactoring
 * in the i2c_compute_presc_scldel_sdadel() function below
 */
#define I2C_LOOP_SDADEL();								\
											\
	if ((tsdadel >= (uint32_t)tsdadel_min) &&					\
		(tsdadel <= (uint32_t)tsdadel_max)) {					\
		if (presc != prev_presc) {						\
			i2c_valid_timing[i2c_valid_timing_nbr].presc = presc;		\
			i2c_valid_timing[i2c_valid_timing_nbr].tscldel = scldel;	\
			i2c_valid_timing[i2c_valid_timing_nbr].tsdadel = sdadel;	\
			prev_presc = presc;						\
			i2c_valid_timing_nbr++;						\
											\
			if (i2c_valid_timing_nbr >= AT32_I2C_VALID_TIMING_NBR) {	\
				break;							\
			}								\
		}									\
	}


/*
 * @brief  Compute PRESC, SCLDEL and SDADEL.
 * @param  clock_src_freq I2C source clock in Hz.
 * @param  i2c_speed I2C frequency (index).
 * @retval None.
 */
static void i2c_compute_presc_scldel_sdadel(uint32_t clock_src_freq, uint32_t i2c_speed)
{
	uint32_t prev_presc = AT32_I2C_PRESC_MAX;
	uint32_t ti2cclk;
	int32_t  tsdadel_min, tsdadel_max;
	int32_t  tscldel_min;
	uint32_t presc, scldel, sdadel;
	uint32_t tafdel_min, tafdel_max;

	ti2cclk   = (NSEC_PER_SEC + (clock_src_freq / 2U)) / clock_src_freq;

	tafdel_min = (AT32_I2C_USE_ANALOG_FILTER == 1U) ?
		AT32_I2C_ANALOG_FILTER_DELAY_MIN : 0U;
	tafdel_max = (AT32_I2C_USE_ANALOG_FILTER == 1U) ?
		AT32_I2C_ANALOG_FILTER_DELAY_MAX : 0U;
	/*
	 * tDNF = DNF x tI2CCLK
	 * tPRESC = (PRESC+1) x tI2CCLK
	 * SDADEL >= {tf +tHD;DAT(min) - tAF(min) - tDNF - [3 x tI2CCLK]} / {tPRESC}
	 * SDADEL <= {tVD;DAT(max) - tr - tAF(max) - tDNF- [4 x tI2CCLK]} / {tPRESC}
	 */
	tsdadel_min = (int32_t)at32_i2c_charac[i2c_speed].tfall +
		(int32_t)at32_i2c_charac[i2c_speed].hddat_min -
		(int32_t)tafdel_min -
		(int32_t)(((int32_t)at32_i2c_charac[i2c_speed].dnf + 3) *
		(int32_t)ti2cclk);

	tsdadel_max = (int32_t)at32_i2c_charac[i2c_speed].vddat_max -
		(int32_t)at32_i2c_charac[i2c_speed].trise -
		(int32_t)tafdel_max -
		(int32_t)(((int32_t)at32_i2c_charac[i2c_speed].dnf + 4) *
		(int32_t)ti2cclk);

	/* {[tr+ tSU;DAT(min)] / [tPRESC]} - 1 <= SCLDEL */
	tscldel_min = (int32_t)at32_i2c_charac[i2c_speed].trise +
		(int32_t)at32_i2c_charac[i2c_speed].sudat_min;

	if (tsdadel_min <= 0) {
		tsdadel_min = 0;
	}

	if (tsdadel_max <= 0) {
		tsdadel_max = 0;
	}

	for (presc = 0; presc < AT32_I2C_PRESC_MAX; presc++) {
		for (scldel = 0; scldel < AT32_I2C_SCLDEL_MAX; scldel++) {
			/* TSCLDEL = (SCLDEL+1) * (PRESC+1) * TI2CCLK */
			uint32_t tscldel = (scldel + 1U) * (presc + 1U) * ti2cclk;

			if (tscldel >= (uint32_t)tscldel_min) {
				for (sdadel = 0; sdadel < AT32_I2C_SDADEL_MAX; sdadel++) {
					/* TSDADEL = SDADEL * (PRESC+1) * TI2CCLK */
					uint32_t tsdadel = (sdadel * (presc + 1U)) * ti2cclk;

					I2C_LOOP_SDADEL();
				}

				if (i2c_valid_timing_nbr >= AT32_I2C_VALID_TIMING_NBR) {
					return;
				}
			}
		}
	}
}
  
  
static int at32_i2c_configure_timming(const struct device *dev, uint32_t clock)
{
  struct i2c_at32_data *data = dev->data;
	const struct i2c_at32_config *cfg = dev->config;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;
	uint32_t timing = 0U;
	uint32_t idx;
	uint32_t speed = 0U;
	uint32_t i2c_freq = cfg->bitrate;

	/* Reset valid timing count at the beginning of each new computation */
	i2c_valid_timing_nbr = 0;

	if ((clock != 0U) && (i2c_freq != 0U)) {
		for (speed = 0 ; speed <= (uint32_t)AT32_I2C_SPEED_FREQ_FAST_PLUS ; speed++) {
			if ((i2c_freq >= at32_i2c_charac[speed].freq_min) &&
				(i2c_freq <= at32_i2c_charac[speed].freq_max)) {
				i2c_compute_presc_scldel_sdadel(clock, speed);
				idx = i2c_compute_scll_sclh(clock, speed);
				if (idx < AT32_I2C_VALID_TIMING_NBR) {
					timing = ((i2c_valid_timing[idx].presc & 0x0FU) << 28) |
          (((i2c_valid_timing[idx].presc >> 4) & 0x0F) << 24) |
					((i2c_valid_timing[idx].tscldel & 0x0FU) << 20) |
					((i2c_valid_timing[idx].tsdadel & 0x0FU) << 16) |
					((i2c_valid_timing[idx].sclh & 0xFFU) << 8) |
					((i2c_valid_timing[idx].scll & 0xFFU) << 0);
				}
				break;
			}
		}
	}

	/* Fill the current timing value in data structure at runtime */
	data->current_timing.periph_clock = clock;
	data->current_timing.i2c_speed = i2c_freq;
	data->current_timing.timing_setting = timing;
  
  i2c_init(i2c_x, 0x0F, timing);
	return 0;
}

#endif

static int i2c_at32_configure(const struct device *dev,
			      uint32_t dev_config)
{
	struct i2c_at32_data *data = dev->data;
	const struct i2c_at32_config *cfg = dev->config;
	uint32_t pclk;
	int err = 0;
	i2c_type *i2c_x = (i2c_type *)cfg->reg;

	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* Disable I2C device */
  i2c_reset(i2c_x);
	i2c_enable(i2c_x, FALSE);

	(void)clock_control_get_rate(AT32_CLOCK_CONTROLLER,
				     (clock_control_subsys_t)&cfg->clkid,
				     &pclk);

  data->dev_config = dev_config;
  at32_i2c_configure_timming(dev, pclk);
	k_sem_give(&data->bus_mutex);

	return err;
}

static DEVICE_API(i2c, i2c_at32_driver_api) = {
	.configure = i2c_at32_configure,
	.transfer = i2c_at32_transfer,
	.get_config = i2c_at32_get_config,
#if defined(CONFIG_I2C_TARGET)
	.target_register = i2c_at32_target_register,
	.target_unregister = i2c_at32_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int i2c_at32_init(const struct device *dev)
{
	struct i2c_at32_data *data = dev->data;
	const struct i2c_at32_config *cfg = dev->config;
	uint32_t bitrate_cfg;
	int err;

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	/* Mutex semaphore to protect the i2c api in multi-thread env. */
	k_sem_init(&data->bus_mutex, 1, 1);

	data->is_configured = false;
	
	/* Sync semaphore to sync i2c state between isr and transfer api. */
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);

	(void)clock_control_on(AT32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid);

	(void)reset_line_toggle_dt(&cfg->reset);

	cfg->irq_cfg_func();

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);

	i2c_at32_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
    data->is_configured = true;
	return 0;
}


#define I2C_AT32_INIT(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);						\
	static void i2c_at32_irq_cfg_func_##inst(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, event, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, event, priority),		\
			    at32_i2c_event_isr,					\
			    DEVICE_DT_INST_GET(inst),				\
			    0);							\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, event, irq));		\
										\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, error, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, error, priority),		\
			    at32_i2c_error_isr,					\
			    DEVICE_DT_INST_GET(inst),				\
			    0);							\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, error, irq));		\
	}									\
	static struct i2c_at32_data i2c_at32_data_##inst;			\
	const static struct i2c_at32_config i2c_at32_cfg_##inst = {		\
		.reg = DT_INST_REG_ADDR(inst),					\
		.bitrate = DT_INST_PROP(inst, clock_frequency),			\
		.clkid = DT_INST_CLOCKS_CELL(inst, id),				\
		.reset = RESET_DT_SPEC_INST_GET(inst),				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),			\
		.irq_cfg_func = i2c_at32_irq_cfg_func_##inst,			\
	};									\
	I2C_DEVICE_DT_INST_DEFINE(inst,						\
				  i2c_at32_init, NULL,				\
				  &i2c_at32_data_##inst, &i2c_at32_cfg_##inst,	\
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,	\
				  &i2c_at32_driver_api);			\

DT_INST_FOREACH_STATUS_OKAY(I2C_AT32_INIT)