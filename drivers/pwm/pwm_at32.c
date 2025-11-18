/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2020 Teslabs Engineering S.L.
 * Copyright (c) 2023 Nobleo Technology
 * Copyright (c) 2023 Maxjta
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT at_at32_pwm

#include <errno.h>

#include <soc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/drivers/reset.h>
#include <at32_tmr.h>
#include <at32_regs.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(pwm_at32, CONFIG_PWM_LOG_LEVEL);

#ifndef IS_TIM_32B_COUNTER_INSTANCE
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)
#endif

#define AT32_PWM_COMPLEMENTARY	(1U << 8)

#define PWM_AT32_COMPLEMENTARY	(1U << 8)

#define AT32_PWM_COMPLEMENTARY_MASK	0x100

#ifdef CONFIG_PWM_CAPTURE

/**
 * @brief Capture state when in 4-channel support mode
 */
enum capture_state {
	CAPTURE_STATE_IDLE = 0,
	CAPTURE_STATE_WAIT_FOR_UPDATE_EVENT = 1,
	CAPTURE_STATE_WAIT_FOR_PULSE_START = 2,
	CAPTURE_STATE_WAIT_FOR_PERIOD_END = 3
};

/** Return the complimentary channel number
 * that is used to capture the end of the pulse.
 */
static const uint32_t complimentary_channel[] = {0, 2, 1, 4, 3};

struct pwm_at32_capture_data {
	pwm_capture_callback_handler_t callback;
	void *user_data;
	uint32_t period;
	uint32_t pulse;
	uint32_t overflows;
	uint8_t skip_irq;
	bool capture_period;
	bool capture_pulse;
	bool continuous;
	uint8_t channel;

	/* only used when four_channel_capture_support */
	enum capture_state state;
};

/* When PWM capture is done by resetting the counter with UIF then the
 * first capture is always nonsense, second is nonsense when polarity changed
 * This is not the case when using four-channel-support.
 */
#define SKIPPED_PWM_CAPTURES 2u

#endif /*CONFIG_PWM_CAPTURE*/
#define AT32_CLOCK_ID_OFFSET(id) (((id) >> 6U) & 0xFFU)
/** PWM data. */
struct pwm_at32_data {
	/** Timer clock (Hz). */
	uint32_t tim_clk;
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_at32_capture_data capture;
#endif /* CONFIG_PWM_CAPTURE */
};

/** PWM configuration. */
struct pwm_at32_config {
	uint32_t reg;
    struct reset_dt_spec reset;
	uint32_t prescaler;
	uint32_t countermode;
	const struct pinctrl_dev_config *pcfg;
    uint16_t clkid;
    uint8_t channels;
	bool is_32bit;
	bool is_advanced;
#ifdef CONFIG_PWM_CAPTURE
	void (*irq_config_func)(const struct device *dev);
	const bool four_channel_capture_support;
#endif /* CONFIG_PWM_CAPTURE */
};

#define TIMER_MAX_CH 4u

/** Channel to LL mapping. */
static const uint32_t ch2ll[TIMER_MAX_CH] = {
	TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2,
	TMR_SELECT_CHANNEL_3, TMR_SELECT_CHANNEL_4
};

/** Some AT32 mcus have complementary channels : 3 or 4 */
static const uint32_t ch2ll_n[] = {
#if defined(TMR_SELECT_CHANNEL_1C)
	TMR_SELECT_CHANNEL_1C,
	TMR_SELECT_CHANNEL_2C,
	TMR_SELECT_CHANNEL_3C,
#endif /* LL_TIM_CHANNEL_CH1N */
};
/** Maximum number of complemented timer channels is ARRAY_SIZE(ch2ll_n)*/                 
static uint32_t set_timer_compare(tmr_type *tmr_x, int id, uint32_t pulse_cycles)
{
    tmr_channel_value_set(tmr_x, ch2ll[id], pulse_cycles);
	return 0;
}

static uint32_t get_channel_capture(tmr_type *tmr_x, int id)
{
    return tmr_channel_value_get(tmr_x, ch2ll[id]);
}

static void enable_capture_interrupt(tmr_type *tmr_x, int id)
{
    switch(id)
    {
        case 0:
            tmr_interrupt_enable(tmr_x, TMR_C1_INT, TRUE);
            break;
        case 1:
            tmr_interrupt_enable(tmr_x, TMR_C2_INT, TRUE);
            break;
        case 2:
            tmr_interrupt_enable(tmr_x, TMR_C3_INT, TRUE);
            break;
        case 3:
            tmr_interrupt_enable(tmr_x, TMR_C4_INT, TRUE);
            break;
        default:
            break;
    }
}

static void disable_capture_interrupt(tmr_type *tmr_x, int id)
{
    switch(id)
    {
        case 0:
          tmr_interrupt_enable(tmr_x, TMR_C1_INT, FALSE);
          break;
        case 1:
            tmr_interrupt_enable(tmr_x, TMR_C2_INT, FALSE);
            break;
        case 2:
            tmr_interrupt_enable(tmr_x, TMR_C3_INT, FALSE);
            break;
        case 3:
            tmr_interrupt_enable(tmr_x, TMR_C4_INT, FALSE);
            break;
        default:
            break;
    }
}

static uint32_t is_capture_active(tmr_type *tmr_x, uint8_t id)
{
    switch(id)
    {
        case 0:
            return tmr_flag_get(tmr_x, TMR_C1_FLAG);
        case 1:
            return tmr_flag_get(tmr_x, TMR_C2_FLAG);
        case 2:
            return tmr_flag_get(tmr_x, TMR_C3_FLAG);
        case 3:
            return tmr_flag_get(tmr_x, TMR_C4_FLAG);
        default:
            break;
    }
  return 0;
}

static void clear_capture_interrupt(tmr_type *tmr_x, uint8_t id)
{
    switch(id)
    {
        case 0:
            tmr_flag_clear(tmr_x, TMR_C1_FLAG);
            break;
        case 1:
            tmr_flag_clear(tmr_x, TMR_C2_FLAG);
            break;
        case 2:
            tmr_flag_clear(tmr_x, TMR_C3_FLAG);
            break;
        case 3:
            tmr_flag_clear(tmr_x, TMR_C4_FLAG);
            break;
        default:
            break;
    }
}

/**
 * Obtain LL polarity from PWM flags.
 *
 * @param flags PWM flags.
 *
 * @return LL polarity.
 */
static uint32_t get_polarity(pwm_flags_t flags)
{
	if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
		return TMR_OUTPUT_ACTIVE_HIGH;
	}

	return TMR_OUTPUT_ACTIVE_LOW;
}

/**
 * @brief  Check if LL counter mode is center-aligned.
 *
 * @param  ll_countermode LL counter mode.
 *
 * @return `true` when center-aligned, otherwise `false`.
 */
static inline bool is_center_aligned(const uint32_t ll_countermode)
{
  	return ((ll_countermode == TMR_COUNT_UP) ||
		(ll_countermode == TMR_COUNT_DOWN) ||
		(ll_countermode == TMR_COUNT_TWO_WAY_3));
}

/**
 * Obtain timer clock speed.
 *
 * @param pclken  Timer clock control subsystem.
 * @param tim_clk Where computed timer clock will be stored.
 *
 * @return 0 on success, error code otherwise.
 */
static int pwm_at32_get_tim_clk(const struct device *dev, uint32_t *tim_clk)
{
    const struct pwm_at32_config *cfg = dev->config;
	int r;
	uint32_t bus_clk;

    r = clock_control_get_rate(AT32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid, &bus_clk);
	if (r < 0) {
		return r;
	}
    switch(AT32_CLOCK_ID_OFFSET(cfg->clkid))
    {
        case CRM_APB1EN_OFFSET:
            if((CRM->cfg_bit.apb1div & 0x4) != 0)
            {
                *tim_clk = bus_clk * 2;
            }
            else
            {
                *tim_clk = bus_clk;
            }
            break;
        case CRM_APB2EN_OFFSET:
            if((CRM->cfg_bit.apb2div & 0x4) != 0)
            {
                *tim_clk = bus_clk * 2;
            }
            else
            {
               *tim_clk = bus_clk;
            }
            break;
        default:
            return -ENODEV;
    }
	return 0;
}

static uint8_t tmr_channel_is_enable(tmr_type *tmr_x, tmr_channel_select_type tmr_channel)
{
    uint8_t is_enable = 0;

    switch(tmr_channel)
    {
        case TMR_SELECT_CHANNEL_1:
            if(tmr_x->cctrl_bit.c1en)
                is_enable = 1;
            break;

        case TMR_SELECT_CHANNEL_1C:
            if(tmr_x->cctrl_bit.c1cen)
                is_enable = 1;
            break;

        case TMR_SELECT_CHANNEL_2:
            if(tmr_x->cctrl_bit.c2en)
                is_enable = 1;
            break;
        case TMR_SELECT_CHANNEL_2C:
            if(tmr_x->cctrl_bit.c2cen)
                is_enable = 1;
            break;

        case TMR_SELECT_CHANNEL_3:
            if(tmr_x->cctrl_bit.c3en)
                is_enable = 1;
            break;

        case TMR_SELECT_CHANNEL_3C:
            if(tmr_x->cctrl_bit.c3cen)
               is_enable = 1;
            break;

        case TMR_SELECT_CHANNEL_4:
            if(tmr_x->cctrl_bit.c4en)
               is_enable = 1;
            break;

        default:
            break;
    }
  return is_enable;
}

static uint8_t tmr_output_channel_polarity_get(tmr_type *tmr_x, tmr_channel_select_type tmr_channel)
{
    uint16_t channel;

    channel = tmr_channel;

    switch(channel)
    {
        case TMR_SELECT_CHANNEL_1:
            return tmr_x->cctrl_bit.c1p;

        case TMR_SELECT_CHANNEL_2:
            return tmr_x->cctrl_bit.c2p;

        case TMR_SELECT_CHANNEL_3:
            return tmr_x->cctrl_bit.c3p;

        case TMR_SELECT_CHANNEL_4:
            return tmr_x->cctrl_bit.c4p;

        case TMR_SELECT_CHANNEL_1C:
            return tmr_x->cctrl_bit.c1cp;

        case TMR_SELECT_CHANNEL_2C:
            return tmr_x->cctrl_bit.c2cp;

        case TMR_SELECT_CHANNEL_3C:
            return tmr_x->cctrl_bit.c3cp;

        default:
            break;
  }
  return 0;
}

static int pwm_at32_set_cycles(const struct device *dev, uint32_t channel,
				uint32_t period_cycles, uint32_t pulse_cycles,
				pwm_flags_t flags)
{
	const struct pwm_at32_config *cfg = dev->config;
    tmr_type *tmr_x = (tmr_type *)cfg->reg;
	uint32_t ll_channel;
	uint32_t current_ll_channel; /* complementary output if used */
	uint32_t negative_ll_channel;
  
	printk("set cycles channel = %d, pulse_cycles = %d\n", channel, pulse_cycles);
	if (channel < 1u || channel > TIMER_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	/*
	 * Non 32-bit timers count from 0 up to the value in the ARR register
	 * (16-bit). Thus period_cycles cannot be greater than UINT16_MAX + 1.
	 */
	if (!cfg->is_32bit &&
	    (period_cycles > UINT16_MAX + 1)) {
		LOG_ERR("Cannot set PWM output, value exceeds 16-bit timer limit.");
		return -ENOTSUP;
	}
#ifdef CONFIG_PWM_CAPTURE
  if(tmr_x->iden_bit.c1ien || tmr_x->iden_bit.c2ien ||
    tmr_x->iden_bit.c3ien || tmr_x->iden_bit.c4ien)
  {
    LOG_ERR("Cannot set PWM output, capture in progress");
		return -EBUSY;
  }
#endif /* CONFIG_PWM_CAPTURE */

	ll_channel = ch2ll[channel - 1u];

	if (channel <= ARRAY_SIZE(ch2ll_n)) {
		negative_ll_channel = ch2ll_n[channel - 1u];
	} else {
		negative_ll_channel = 0;
	}

	/* in LL_TIM_CC_DisableChannel and LL_TIM_CC_IsEnabledChannel,
	 * the channel param could be the complementary one
	 */
	if ((flags & AT32_PWM_COMPLEMENTARY_MASK) == AT32_PWM_COMPLEMENTARY) {
		if (!negative_ll_channel) {
			/* setting a flag on a channel that has not this capability */
			LOG_ERR("Channel %d has NO complementary output", channel);
			return -EINVAL;
		}
		current_ll_channel = negative_ll_channel;
	} else {
		current_ll_channel = ll_channel;
	}
	if (period_cycles == 0u) {
        tmr_channel_enable(tmr_x, current_ll_channel, FALSE);
		return 0;
	}

	if (cfg->countermode == TMR_COUNT_UP) {
		/* remove 1 period cycle, accounts for 1 extra low cycle */
		period_cycles -= 1U;
	} else if (cfg->countermode == TMR_COUNT_DOWN) {
		/* remove 1 pulse cycle, accounts for 1 extra high cycle */
		pulse_cycles -= 1U;
		/* remove 1 period cycle, accounts for 1 extra low cycle */
		period_cycles -= 1U;
	} else if (is_center_aligned(cfg->countermode)) {
		pulse_cycles /= 2U;
		period_cycles /= 2U;
	} else {
		return -ENOTSUP;
	}

    if(!tmr_channel_is_enable(tmr_x, current_ll_channel))
    {
        tmr_output_config_type oc_init;

        tmr_output_default_para_init(&oc_init);
    
        oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;

#if defined(TMR_SELECT_CHANNEL_1C)
		/* the flags holds the PWM_COMPLEMENTARY information */
		if ((flags & AT32_PWM_COMPLEMENTARY_MASK) == AT32_PWM_COMPLEMENTARY) {
			
            oc_init.occ_output_state = TRUE;
            oc_init.occ_polarity = get_polarity(flags);

			/* inherit the polarity of the positive output */
            oc_init.oc_output_state = tmr_channel_is_enable(tmr_x, ll_channel) ? TRUE : FALSE;
            oc_init.oc_polarity = tmr_output_channel_polarity_get(tmr_x, ll_channel);
		} else {
            oc_init.oc_output_state = TRUE;
            oc_init.oc_polarity = get_polarity(flags);

			/* inherit the polarity of the negative output */
			if (negative_ll_channel) {
                oc_init.occ_output_state = tmr_channel_is_enable(tmr_x, negative_ll_channel) ? TRUE : FALSE;;
                oc_init.occ_polarity = tmr_output_channel_polarity_get(tmr_x, negative_ll_channel);;
			}
		}
#else /* TMR_SELECT_CHANNEL_1C */
    
        oc_init.oc_output_state = TRUE;
        oc_init.oc_polarity = get_polarity(flags);
#endif /* TMR_SELECT_CHANNEL_1C */
        tmr_channel_value_set(tmr_x, ll_channel, pulse_cycles);

#ifdef CONFIG_PWM_CAPTURE
		if (IS_TMR_SLAVE(tmr_x)) {
            tmr_sub_mode_select(tmr_x, TMR_SUB_MODE_DIABLE);
            tmr_trigger_input_select(tmr_x, TMR_SUB_INPUT_SEL_IS0);
            tmr_sub_sync_mode_set(tmr_x, FALSE);
		}
#endif /* CONFIG_PWM_CAPTURE */
        tmr_output_channel_config(tmr_x, ll_channel, &oc_init);
        tmr_channel_value_set(tmr_x, ll_channel, pulse_cycles);

        tmr_period_buffer_enable(tmr_x, TRUE);
        tmr_output_channel_buffer_enable(tmr_x, ll_channel, TRUE);
        tmr_period_value_set(tmr_x, period_cycles);
        tmr_event_sw_trigger(tmr_x, TMR_OVERFLOW_SWTRIG);
	} else {
        tmr_output_channel_polarity_set(tmr_x, current_ll_channel, get_polarity(flags));
		set_timer_compare(tmr_x, channel - 1u, pulse_cycles);
        tmr_period_value_set(tmr_x, period_cycles);
	}

	return 0;
}

#ifdef CONFIG_PWM_CAPTURE
static int init_capture_channels(const struct device *dev, uint32_t channel,
				pwm_flags_t flags)
{
	const struct pwm_at32_config *cfg = dev->config;
    tmr_type *tmr_x = (tmr_type *)cfg->reg;
	bool is_inverted = (flags & PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED;
    tmr_input_config_type ic;
    
    ic.input_channel_select = ch2ll[channel - 1];
    ic.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    ic.input_polarity_select = is_inverted ? TMR_INPUT_FALLING_EDGE : TMR_INPUT_RISING_EDGE;
    ic.input_filter_value = 0;
  
    tmr_input_channel_init(tmr_x, &ic, TMR_CHANNEL_INPUT_DIV_1);

    ic.input_channel_select = ch2ll[complimentary_channel[channel] - 1];
    ic.input_mapped_select = TMR_CC_CHANNEL_MAPPED_INDIRECT;
    ic.input_polarity_select = is_inverted ? TMR_INPUT_RISING_EDGE : TMR_INPUT_FALLING_EDGE;
    ic.input_filter_value = 0;
    tmr_input_channel_init(tmr_x, &ic, TMR_CHANNEL_INPUT_DIV_1);
  
	/* Setup complimentary channel */
	return 0;
}

static int pwm_at32_configure_capture(const struct device *dev,
				       uint32_t channel, pwm_flags_t flags,
				       pwm_capture_callback_handler_t cb,
				       void *user_data)
{
	/*
	 * Capture is implemented in two different ways, depending on the
	 * four-channel-capture-support setting in the node.
	 *  - Two Channel Support:
	 *	Only two channels (1 and 2) are available for capture. It uses
	 *	the slave mode controller to reset the counter on each edge.
	 *  - Four Channel Support:
	 *	All four channels are available for capture. Instead of the
	 *	slave mode controller it uses the ISR to reset the counter.
	 *	This is slightly less accurate, but still within acceptable
	 *	bounds.
	 */

	const struct pwm_at32_config *cfg = dev->config;
	struct pwm_at32_data *data = dev->data;
	struct pwm_at32_capture_data *cpt = &data->capture;
    tmr_type *tmr_x = (tmr_type *)cfg->reg;
	int ret;

	if (!cfg->four_channel_capture_support) {
		if ((channel != 1u) && (channel != 2u)) {
			LOG_ERR("PWM capture only supported on first two channels");
			return -ENOTSUP;
		}
	} else {
		if ((channel < 1u) || (channel > 4u)) {
			LOG_ERR("PWM capture only exists on channels 1, 2, 3 and 4.");
			return -ENOTSUP;
		}
	}
  
  if(tmr_x->iden_bit.c1ien || tmr_x->iden_bit.c2ien ||
    tmr_x->iden_bit.c3ien || tmr_x->iden_bit.c4ien)
  {
      LOG_ERR("PWM capture already active");
		return -EBUSY;
  }

	if (!(flags & PWM_CAPTURE_TYPE_MASK)) {
		LOG_ERR("No PWM capture type specified");
		return -EINVAL;
	}

	if (!cfg->four_channel_capture_support && !IS_TMR_SLAVE(tmr_x)) {
		/* slave mode is only used when not in four channel mode */
		LOG_ERR("Timer does not support slave mode for PWM capture");
		return -ENOTSUP;
	}
  
	cpt->callback = cb; /* even if the cb is reset, this is not an error */
	cpt->user_data = user_data;
	cpt->capture_period = (flags & PWM_CAPTURE_TYPE_PERIOD) ? true : false;
	cpt->capture_pulse = (flags & PWM_CAPTURE_TYPE_PULSE) ? true : false;
	cpt->continuous = (flags & PWM_CAPTURE_MODE_CONTINUOUS) ? true : false;

	/* Prevents faulty behavior while making changes */
    tmr_sub_mode_select(tmr_x, TMR_SUB_MODE_DIABLE);

	ret = init_capture_channels(dev, channel, flags);
	if (ret < 0) {
		return ret;
	}

	if (!cfg->four_channel_capture_support) {
		if (channel == 1u) {
            tmr_trigger_input_select(tmr_x, TMR_SUB_INPUT_SEL_C1DF1);
		} else {
            tmr_trigger_input_select(tmr_x, TMR_SUB_INPUT_SEL_C2DF2);
		}
        tmr_sub_mode_select(tmr_x, TMR_SUB_RESET_MODE);
	}

    tmr_period_buffer_enable(tmr_x, TRUE);
	if (!cfg->is_32bit) {
        tmr_period_value_set(tmr_x, 0xffffu);
	} else {
        tmr_period_value_set(tmr_x, 0xffffffffu);
	}
    tmr_overflow_event_disable(tmr_x, FALSE);
	return 0;
}

static int pwm_at32_enable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_at32_config *cfg = dev->config;
	struct pwm_at32_data *data = dev->data;
	struct pwm_at32_capture_data *cpt = &data->capture;
    tmr_type *tmr_x = (tmr_type *)cfg->reg;

	if (!cfg->four_channel_capture_support) {
		if ((channel != 1u) && (channel != 2u)) {
			LOG_ERR("PWM capture only supported on first two channels");
			return -ENOTSUP;
		}
	} else {
		if ((channel < 1u) || (channel > 4u)) {
			LOG_ERR("PWM capture only exists on channels 1, 2, 3 and 4.");
			return -ENOTSUP;
		}
	}
  
  if(tmr_x->iden_bit.c1ien || tmr_x->iden_bit.c2ien ||
    tmr_x->iden_bit.c3ien || tmr_x->iden_bit.c4ien)
  {
    LOG_ERR("PWM capture already active");
		return -EBUSY;
  }

	if (!data->capture.callback) {
		LOG_ERR("PWM capture not configured");
		return -EINVAL;
	}

	cpt->channel = channel;
	cpt->state = CAPTURE_STATE_WAIT_FOR_PULSE_START;
	data->capture.skip_irq = cfg->four_channel_capture_support ?  0 : SKIPPED_PWM_CAPTURES;
	data->capture.overflows = 0u;

    clear_capture_interrupt(tmr_x, channel -1);
    tmr_flag_clear(tmr_x, TMR_OVF_FLAG);

    tmr_overflow_request_source_set(tmr_x, TRUE);

	enable_capture_interrupt(tmr_x, channel - 1);

    tmr_channel_enable(tmr_x, ch2ll[channel - 1], TRUE);
    tmr_channel_enable(tmr_x, ch2ll[complimentary_channel[channel] - 1], TRUE);
    tmr_interrupt_enable(tmr_x, TMR_OVF_INT, TRUE);
    tmr_event_sw_trigger(tmr_x, TMR_OVERFLOW_SWTRIG);

	return 0;
}

static int pwm_at32_disable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_at32_config *cfg = dev->config;
    tmr_type *tmr_x = (tmr_type *)cfg->reg;

	if (!cfg->four_channel_capture_support) {
		if ((channel != 1u) && (channel != 2u)) {
			LOG_ERR("PWM capture only supported on first two channels");
			return -ENOTSUP;
		}
	} else {
		if ((channel < 1u) || (channel > 4u)) {
			LOG_ERR("PWM capture only exists on channels 1, 2, 3 and 4.");
			return -ENOTSUP;
		}
	}

    tmr_overflow_request_source_set(tmr_x, FALSE);

	disable_capture_interrupt(tmr_x, channel - 1 );
    tmr_interrupt_enable(tmr_x, TMR_OVF_INT, FALSE);
    tmr_channel_enable(tmr_x, ch2ll[channel - 1], FALSE);
    tmr_channel_enable(tmr_x, ch2ll[complimentary_channel[channel] - 1], FALSE);

	return 0;
}

static void pwm_at32_isr(const struct device *dev)
{
	const struct pwm_at32_config *cfg = dev->config;
	struct pwm_at32_data *data = dev->data;
	struct pwm_at32_capture_data *cpt = &data->capture;
    tmr_type *tmr_x = (tmr_type *)cfg->reg;
	int status = 0;

	if (cpt->skip_irq != 0u) {
		if (tmr_flag_get(tmr_x, TMR_OVF_FLAG)) {
            tmr_flag_clear(tmr_x, TMR_OVF_FLAG);
		}

        if(tmr_flag_get(tmr_x, TMR_C1_FLAG | TMR_C2_FLAG | 
                           TMR_C3_FLAG | TMR_C4_FLAG)) {
                             
            tmr_flag_clear(tmr_x, TMR_C1_FLAG | TMR_C2_FLAG | 
                           TMR_C3_FLAG | TMR_C4_FLAG);
            cpt->skip_irq--;
        }
		return;
	}

	if (tmr_flag_get(tmr_x, TMR_OVF_FLAG)) {
		tmr_flag_clear(tmr_x, TMR_OVF_FLAG);
		if (cfg->four_channel_capture_support &&
				cpt->state == CAPTURE_STATE_WAIT_FOR_UPDATE_EVENT) {
			/* Special handling of UPDATE event in case it's triggered */
			cpt->state = CAPTURE_STATE_WAIT_FOR_PERIOD_END;
		} else {
			cpt->overflows++;
		}
	}

	if (!cfg->four_channel_capture_support) {
		if (is_capture_active(tmr_x, cpt->channel - 1) ||
		    is_capture_active(tmr_x, complimentary_channel[cpt->channel] - 1)) {
            clear_capture_interrupt(tmr_x, cpt->channel -1);
            clear_capture_interrupt(tmr_x, complimentary_channel[cpt->channel] - 1);

			cpt->period = get_channel_capture(tmr_x, cpt->channel - 1);//[cpt->channel - 1](cfg->timer);
			cpt->pulse = get_channel_capture(tmr_x, complimentary_channel[cpt->channel] - 1);//[complimentary_channel[cpt->channel] - 1](cfg->timer);
		}
	} else {
		if (cpt->state == CAPTURE_STATE_WAIT_FOR_PULSE_START &&
		    is_capture_active(tmr_x, cpt->channel - 1)) {
			/* Reset the counter manually instead of automatically by HW
			 * This sets the pulse-start at 0 and makes the pulse-end
			 * and period related to that number. Sure we loose some
			 * accuracy but it's within acceptable range.
			 *
			 * This is done through an UPDATE event to also reset
			 * the prescalar. This could look like an overflow event
			 * and might therefore require special handling.
			 */
			cpt->state = CAPTURE_STATE_WAIT_FOR_UPDATE_EVENT;
            tmr_event_sw_trigger(tmr_x, TMR_OVERFLOW_SWTRIG);

		} else if ((cpt->state == CAPTURE_STATE_WAIT_FOR_UPDATE_EVENT ||
				cpt->state == CAPTURE_STATE_WAIT_FOR_PERIOD_END) &&
			    is_capture_active(tmr_x, cpt->channel - 1)) {
			    cpt->state = CAPTURE_STATE_IDLE;
			/* The end of the period. Both capture channels should now contain
			 * the timer value when the pulse and period ended respectively.
			 */
			  cpt->pulse = get_channel_capture(tmr_x, complimentary_channel[cpt->channel] - 1);//[complimentary_channel[cpt->channel] - 1](cfg->timer);
			  cpt->period = get_channel_capture(tmr_x, cpt->channel - 1);//[cpt->channel - 1](cfg->timer);
		}

        clear_capture_interrupt(tmr_x, cpt->channel -1);

		if (cpt->state != CAPTURE_STATE_IDLE) {
			/* Still waiting for a complete capture */
			return;
		}
	}

	if (cpt->overflows) {
		LOG_ERR("counter overflow during PWM capture");
		status = -ERANGE;
	}

	if (!cpt->continuous) {
		pwm_at32_disable_capture(dev, cpt->channel);
	} else {
		cpt->overflows = 0u;
		cpt->state = CAPTURE_STATE_WAIT_FOR_PULSE_START;
	}

	if (cpt->callback != NULL) {
		cpt->callback(dev, cpt->channel, cpt->capture_period ? cpt->period : 0u,
				cpt->capture_pulse ? cpt->pulse : 0u, status, cpt->user_data);
	}
}

#endif /* CONFIG_PWM_CAPTURE */

static int pwm_at32_get_cycles_per_sec(const struct device *dev,
					uint32_t channel, uint64_t *cycles)
{
	struct pwm_at32_data *data = dev->data;
	const struct pwm_at32_config *cfg = dev->config;

	*cycles = (uint64_t)(data->tim_clk / (cfg->prescaler + 1));

	return 0;
}

static DEVICE_API(pwm, pwm_at32_driver_api) = {
	.set_cycles = pwm_at32_set_cycles,
	.get_cycles_per_sec = pwm_at32_get_cycles_per_sec,
#ifdef CONFIG_PWM_CAPTURE
	.configure_capture = pwm_at32_configure_capture,
	.enable_capture = pwm_at32_enable_capture,
	.disable_capture = pwm_at32_disable_capture,
#endif /* CONFIG_PWM_CAPTURE */
};

static int pwm_at32_init(const struct device *dev)
{
	struct pwm_at32_data *data = dev->data;
	const struct pwm_at32_config *cfg = dev->config;
    tmr_type *tmr_x = (tmr_type *)cfg->reg;

	int r;

    r = clock_control_on(AT32_CLOCK_CONTROLLER,(clock_control_subsys_t)&cfg->clkid);
	if (r < 0) {
		LOG_ERR("Could not initialize clock (%d)", r);
		return r;
	}

	r =  pwm_at32_get_tim_clk(dev, &data->tim_clk);
	if (r < 0) {
		LOG_ERR("Could not obtain timer clock (%d)", r);
		return r;
	}

	/* Reset timer to default state using RCC */
	(void)reset_line_toggle_dt(&cfg->reset);

	/* configure pinmux */
	r = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (r < 0) {
		LOG_ERR("PWM pinctrl setup failed (%d)", r);
		return r;
	}
  
    tmr_base_init(tmr_x, 0, cfg->prescaler);
    tmr_clock_source_div_set(tmr_x, TMR_CLOCK_DIV1);
    tmr_cnt_dir_set(tmr_x, cfg->countermode);
    if (cfg->is_32bit)
    {
        tmr_32_bit_function_enable(tmr_x, TRUE);
    }
	/* enable outputs and counter */
	if (IS_TMR_BREAK(tmr_x)) {
        tmr_output_enable(tmr_x, TRUE);
	}

    tmr_counter_enable(tmr_x, TRUE);

#ifdef CONFIG_PWM_CAPTURE
	cfg->irq_config_func(dev);
#endif /* CONFIG_PWM_CAPTURE */

	return 0;
}



#define PWM(index) DT_INST_PARENT(index)

#ifdef CONFIG_PWM_CAPTURE
#define IRQ_CONNECT_AND_ENABLE_BY_NAME(index, name)				\
{										\
	IRQ_CONNECT(DT_IRQ_BY_NAME(PWM(index), name, irq),			\
			DT_IRQ_BY_NAME(PWM(index), name, priority),		\
			pwm_at32_isr, DEVICE_DT_INST_GET(index), 0);		\
	irq_enable(DT_IRQ_BY_NAME(PWM(index), name, irq));			\
}

#define IRQ_CONNECT_AND_ENABLE_DEFAULT(index)					\
{										\
	IRQ_CONNECT(DT_IRQN(PWM(index)),					\
			DT_IRQ(PWM(index), priority),				\
			pwm_at32_isr, DEVICE_DT_INST_GET(index), 0);		\
	irq_enable(DT_IRQN(PWM(index)));					\
}

#define IRQ_CONFIG_FUNC(index)                                                  \
static void pwm_at32_irq_config_func_##index(const struct device *dev)		\
{										\
	COND_CODE_1(DT_IRQ_HAS_NAME(PWM(index), cc),				\
		(IRQ_CONNECT_AND_ENABLE_BY_NAME(index, cc)),			\
		(IRQ_CONNECT_AND_ENABLE_DEFAULT(index))				\
	);									\
}
#define CAPTURE_INIT(index)                                                                        \
	.irq_config_func = pwm_at32_irq_config_func_##index,                                      \
	.four_channel_capture_support = DT_INST_PROP(index, four_channel_capture_support)
#else
#define IRQ_CONFIG_FUNC(index)
#define CAPTURE_INIT(index)
#endif /* CONFIG_PWM_CAPTURE */

#define DT_INST_CLK(index, inst)                                               \
	{                                                                      \
		.bus = DT_CLOCKS_CELL(PWM(index), bus),				\
		.enr = DT_CLOCKS_CELL(PWM(index), bits)				\
	}


#define PWM_DEVICE_INIT(index)                                                 \
	static struct pwm_at32_data pwm_at32_data_##index = {		       \
		.reset = RESET_DT_SPEC_GET(PWM(index)),			       \
	};								       \
									       \
	IRQ_CONFIG_FUNC(index)						       \
									       \
	PINCTRL_DT_INST_DEFINE(index);					       \
									       \
	static const struct pwm_at32_config pwm_at32_config_##index = {      \
		.reg = DT_REG_ADDR(DT_INST_PARENT(index)),	       \
    .clkid = DT_CLOCKS_CELL(DT_INST_PARENT(index), id),		       \
    .reset = RESET_DT_SPEC_GET(DT_INST_PARENT(index)),		       \
		.prescaler = DT_PROP(PWM(index), at_prescaler),		       \
		.countermode = DT_PROP(PWM(index), at_countermode),	       \
    .channels = DT_PROP(DT_INST_PARENT(index), channels),	       \
    .is_32bit = DT_PROP(DT_INST_PARENT(index), is_32bit),	       \
		.is_advanced = DT_PROP(DT_INST_PARENT(index), is_advanced),	       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),		       \
		CAPTURE_INIT(index)					       \
	};                                                                     \
									       \
	DEVICE_DT_INST_DEFINE(index, &pwm_at32_init, NULL,                    \
			    &pwm_at32_data_##index,                           \
			    &pwm_at32_config_##index, POST_KERNEL,            \
			    CONFIG_PWM_INIT_PRIORITY,                          \
			    &pwm_at32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT)
