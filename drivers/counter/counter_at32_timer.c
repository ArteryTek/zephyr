/*
 * Copyright (c) 2021 Kent Hall.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT at_at32_timer

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>

#include <at32_regs.h>
#include <at32_tmr.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_timer_at32, CONFIG_COUNTER_LOG_LEVEL);

#ifndef IS_TIM_32B_COUNTER_INSTANCE
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)
#endif

/** Maximum number of timer channels. */
#define TIMER_MAX_CH 4U

static void at32_set_timer_compare(tmr_type *tmr_x, int id, uint32_t value)
{
    switch(id)
    {
      case 0:
            tmr_channel_value_set(tmr_x, TMR_SELECT_CHANNEL_1, value);
            break;
        case 1:
            tmr_channel_value_set(tmr_x, TMR_SELECT_CHANNEL_2, value);
            break;
        case 2:
            tmr_channel_value_set(tmr_x, TMR_SELECT_CHANNEL_3, value);
            break;
        case 3:
            tmr_channel_value_set(tmr_x, TMR_SELECT_CHANNEL_4, value);
            break;
        default:
            break;
    }
}

static uint32_t at32_get_timer_compare(tmr_type *tmr_x, int id)
{
    uint32_t value = 0;
    switch(id)
    {
        case 0:
            value = tmr_channel_value_get(tmr_x, TMR_SELECT_CHANNEL_1);
            break;
        case 1:
            value = tmr_channel_value_get(tmr_x, TMR_SELECT_CHANNEL_2);
            break;
        case 2:
            value = tmr_channel_value_get(tmr_x, TMR_SELECT_CHANNEL_3);
            break;
        case 3:
            value = tmr_channel_value_get(tmr_x, TMR_SELECT_CHANNEL_4);
            break;
        default:
            break;
    }
    return value;
}


static void at32_enable_it(tmr_type *tmr_x, int id)
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

static void at32_disable_it(tmr_type *tmr_x, int id)
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

static uint32_t at32_check_it_enabled(tmr_type *tmr_x, int id)
{
    uint32_t enabled = 0;
    switch(id)
    {
        case 0:
            if(tmr_x->iden & TMR_C1_INT)
                enabled = 1;
            break;
        case 1:
            if(tmr_x->iden & TMR_C2_INT)
                enabled = 1;
            break;
        case 2:
            if(tmr_x->iden & TMR_C3_INT)
               enabled = 1;
            break;
        case 3:
            if(tmr_x->iden & TMR_C3_INT)
                enabled = 1;
            break;
        default:
            break;
    }
    return enabled;
}


static void at32_clear_it_flag(tmr_type *tmr_x, int id)
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
#define AT32_CLOCK_ID_OFFSET(id) (((id) >> 6U) & 0xFFU)
struct counter_at32_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	uint32_t guard_period;
	atomic_t cc_int_pending;
	uint32_t freq;
};

struct counter_at32_ch_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

struct counter_at32_config {
	struct counter_config_info info;
	struct counter_at32_ch_data *ch_data;
    uint32_t reg;
	uint16_t clkid;
	uint32_t prescaler;
	void (*irq_config_func)(const struct device *dev);
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
    void (*set_irq_pending)(void);
	uint32_t (*get_irq_pending)(void);
	LOG_INSTANCE_PTR_DECLARE(log);
};

static int counter_at32_start(const struct device *dev)
{
	const struct counter_at32_config *config = dev->config;
    tmr_type *tmr_x = (tmr_type *)config->reg;

	/* enable counter */
    tmr_counter_enable(tmr_x, TRUE);

	return 0;
}

static int counter_at32_stop(const struct device *dev)
{
	const struct counter_at32_config *config = dev->config;
    tmr_type *tmr_x = (tmr_type *)config->reg;

	/* disable counter */
	tmr_counter_enable(tmr_x, FALSE);

	return 0;
}

static uint32_t counter_at32_get_top_value(const struct device *dev)
{
	const struct counter_at32_config *config = dev->config;
    tmr_type *tmr_x = (tmr_type *)config->reg;

	return tmr_period_value_get(tmr_x);
}

static uint32_t counter_at32_read(const struct device *dev)
{
	const struct counter_at32_config *config = dev->config;
    tmr_type *tmr_x = (tmr_type *)config->reg;

	return tmr_counter_value_get(tmr_x);
}

static int counter_at32_get_value(const struct device *dev, uint32_t *ticks)
{
	*ticks = counter_at32_read(dev);
	return 0;
}

static uint32_t counter_at32_ticks_add(uint32_t val1, uint32_t val2, uint32_t top)
{
	uint32_t to_top;

	if (likely(IS_BIT_MASK(top))) {
		return (val1 + val2) & top;
	}

	to_top = top - val1;

	return (val2 <= to_top) ? val1 + val2 : val2 - to_top - 1U;
}

static uint32_t counter_at32_ticks_sub(uint32_t val, uint32_t old, uint32_t top)
{
	if (likely(IS_BIT_MASK(top))) {
		return (val - old) & top;
	}

	/* if top is not 2^n-1 */
	return (val >= old) ? (val - old) : val + top + 1U - old;
}

static void counter_at32_counter_at32_set_cc_int_pending(const struct device *dev, uint8_t chan)
{
	const struct counter_at32_config *config = dev->config;
	struct counter_at32_data *data = dev->data;

	atomic_or(&data->cc_int_pending, BIT(chan));

	config->set_irq_pending();
}

static int counter_at32_set_cc(const struct device *dev, uint8_t id,
				const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_at32_config *config = dev->config;
	struct counter_at32_data *data = dev->data;
    tmr_type *tmr_x = (tmr_type *)config->reg;

	__ASSERT_NO_MSG(data->guard_period < counter_at32_get_top_value(dev));
	uint32_t val = alarm_cfg->ticks;
	uint32_t flags = alarm_cfg->flags;
	bool absolute = flags & COUNTER_ALARM_CFG_ABSOLUTE;
	bool irq_on_late;
	uint32_t top = counter_at32_get_top_value(dev);
	int err = 0;
	uint32_t prev_val;
	uint32_t now;
	uint32_t diff;
	uint32_t max_rel_val;

	__ASSERT(!at32_check_it_enabled(tmr_x, id),
		 "Expected that CC interrupt is disabled.");

	/* First take care of a risk of an event coming from CC being set to
	 * next tick. Reconfigure CC to future (now tick is the furthest
	 * future).
	 */
	now = counter_at32_read(dev);
	prev_val = at32_get_timer_compare(tmr_x, id);//get_timer_compare[id](timer);
	at32_set_timer_compare(tmr_x, id, now);//[id](timer, now);
	at32_clear_it_flag(tmr_x, id);//clear_it_flag[id](timer);

	if (absolute) {
		max_rel_val = top - data->guard_period;
		irq_on_late = flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
	} else {
		/* If relative value is smaller than half of the counter range
		 * it is assumed that there is a risk of setting value too late
		 * and late detection algorithm must be applied. When late
		 * setting is detected, interrupt shall be triggered for
		 * immediate expiration of the timer. Detection is performed
		 * by limiting relative distance between CC and counter.
		 *
		 * Note that half of counter range is an arbitrary value.
		 */
		irq_on_late = val < (top / 2U);
		/* limit max to detect short relative being set too late. */
		max_rel_val = irq_on_late ? top / 2U : top;
		val = counter_at32_ticks_add(now, val, top);
	}

    at32_set_timer_compare(tmr_x, id, val);

	/* decrement value to detect also case when val == counter_stm32_read(dev). Otherwise,
	 * condition would need to include comparing diff against 0.
	 */
	diff = counter_at32_ticks_sub(val - 1U, counter_at32_read(dev), top);
	if (diff > max_rel_val) {
		if (absolute) {
			err = -ETIME;
		}

		/* Interrupt is triggered always for relative alarm and
		 * for absolute depending on the flag.
		 */
		if (irq_on_late) {
			counter_at32_counter_at32_set_cc_int_pending(dev, id);
		} else {
			config->ch_data[id].callback = NULL;
		}
	} else {
        at32_enable_it(tmr_x, id);
	}
	return err;
}

static int counter_at32_set_alarm(const struct device *dev, uint8_t chan,
				   const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_at32_config *config = dev->config;
	struct counter_at32_ch_data *chdata = &config->ch_data[chan];
  
	if (alarm_cfg->ticks >  counter_at32_get_top_value(dev)) {
		return -EINVAL;
	}
	if (chdata->callback) {
		return -EBUSY;
	}
	chdata->callback = alarm_cfg->callback;
	chdata->user_data = alarm_cfg->user_data;

	return counter_at32_set_cc(dev, chan, alarm_cfg);
}

static int counter_at32_cancel_alarm(const struct device *dev, uint8_t chan)
{
	const struct counter_at32_config *config = dev->config;
    tmr_type *tmr_x = (tmr_type *)config->reg;

    at32_disable_it(tmr_x, chan);
	config->ch_data[chan].callback = NULL;

	return 0;
}

static int counter_at32_set_top_value(const struct device *dev,
				       const struct counter_top_cfg *cfg)
{
	const struct counter_at32_config *config = dev->config;
    tmr_type *tmr_x = (tmr_type *)config->reg;
	struct counter_at32_data *data = dev->data;
	int err = 0;

	for (int i = 0; i < counter_get_num_of_channels(dev); i++) {
		/* Overflow can be changed only when all alarms are
		 * disabled.
		 */
		if (config->ch_data[i].callback) {
			return -EBUSY;
		}
	}
  
    tmr_interrupt_enable(tmr_x, TMR_OVF_INT, FALSE);
    tmr_period_value_set(tmr_x, cfg->ticks);
    tmr_flag_clear(tmr_x, TMR_OVF_FLAG);

	data->top_cb = cfg->callback;
	data->top_user_data = cfg->user_data;

	if (!(cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
    tmr_counter_value_set(tmr_x, 0);
	} else if (counter_at32_read(dev) >= cfg->ticks) {
		err = -ETIME;
		if (cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
			tmr_counter_value_set(tmr_x, 0);
		}
	}

	if (cfg->callback) {
		tmr_interrupt_enable(tmr_x, TMR_OVF_INT, TRUE);
	}

	return err;
}

static uint32_t counter_at32_get_pending_int(const struct device *dev)
{
	const struct counter_at32_config *cfg = dev->config;
    tmr_type *tmr_x = (tmr_type *)cfg->reg;
	uint32_t pending = 0;

	switch (counter_get_num_of_channels(dev)) {
	case 4U:
		pending |= tmr_flag_get(tmr_x, TMR_C4_FLAG);
		__fallthrough;
	case 3U:
		pending |= tmr_flag_get(tmr_x, TMR_C3_FLAG);
		__fallthrough;
	case 2U:
		pending |= tmr_flag_get(tmr_x, TMR_C2_FLAG);
		__fallthrough;
	case 1U:
		pending |= tmr_flag_get(tmr_x, TMR_C1_FLAG);
	}

	return !!pending;
}

/**
 * Obtain timer clock speed.
 *
 * @param pclken  Timer clock control subsystem.
 * @param tim_clk Where computed timer clock will be stored.
 *
 * @return 0 on success, error code otherwise.
 *
 * This function is ripped from the PWM driver; TODO handle code duplication.
 */
static int counter_at32_get_tim_clk(const struct device *dev, uint32_t *tim_clk)
{
    const struct counter_at32_config *cfg = dev->config;
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

static int counter_at32_init_timer(const struct device *dev)
{
	const struct counter_at32_config *cfg = dev->config;
	struct counter_at32_data *data = dev->data;
	tmr_type *tmr_x = (tmr_type *)cfg->reg;
	uint32_t tim_clk = 0;
	int r;
	/* initialize clock and check its speed  */
	r = clock_control_on(AT32_CLOCK_CONTROLLER,
			     (clock_control_subsys_t)&cfg->clkid);
	if (r < 0) {
		LOG_ERR("Could not initialize clock (%d)", r);
		return r;
	}
	r = counter_at32_get_tim_clk(dev, &tim_clk);
	if (r < 0) {
		LOG_ERR("Could not obtain timer clock (%d)", r);
		return r;
	}
	data->freq = tim_clk / (cfg->prescaler + 1U);

	if (!device_is_ready(cfg->reset.dev)) {
		LOG_ERR("reset controller not ready");
		return -ENODEV;
	}

	/* Reset timer to default state using RCC */
	(void)reset_line_toggle_dt(&cfg->reset);

	/* config/enable IRQ */
	cfg->irq_config_func(dev);
    tmr_base_init(tmr_x, counter_get_max_top_value(dev), cfg->prescaler);
    tmr_clock_source_div_set(tmr_x, TMR_CLOCK_DIV1);
    tmr_cnt_dir_set(tmr_x, TMR_COUNT_UP);
	if(counter_get_max_top_value(dev) == 0xFFFFFFFF)
	{
		tmr_32_bit_function_enable(tmr_x, TRUE);
	}
    tmr_period_value_set(tmr_x, 0xFFFFFFFF);
	return 0;
}

static uint32_t counter_at32_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct counter_at32_data *data = dev->data;

	ARG_UNUSED(flags);
	return data->guard_period;
}

static int counter_at32_set_guard_period(const struct device *dev, uint32_t guard,
					  uint32_t flags)
{
	struct counter_at32_data *data = dev->data;

	ARG_UNUSED(flags);
	__ASSERT_NO_MSG(guard < counter_at32_get_top_value(dev));

	data->guard_period = guard;
	return 0;
}

static uint32_t counter_at32_get_freq(const struct device *dev)
{
	struct counter_at32_data *data = dev->data;
	return data->freq;
}

static void counter_at32_top_irq_handle(const struct device *dev)
{
	struct counter_at32_data *data = dev->data;

	counter_top_callback_t cb = data->top_cb;

	__ASSERT(cb != NULL, "top event enabled - expecting callback");
	cb(dev, data->top_user_data);
}

static void counter_at32_alarm_irq_handle(const struct device *dev, uint32_t id)
{
	const struct counter_at32_config *config = dev->config;
	struct counter_at32_data *data = dev->data;
    tmr_type *tmr_x = (tmr_type *)config->reg;

	struct counter_at32_ch_data *chdata;
	counter_alarm_callback_t cb;

	atomic_and(&data->cc_int_pending, ~BIT(id));
    at32_disable_it(tmr_x, id);

	chdata = &config->ch_data[id];
	cb = chdata->callback;
	chdata->callback = NULL;

	if (cb) {
		uint32_t cc_val = at32_get_timer_compare(tmr_x, id);

		cb(dev, id, cc_val, chdata->user_data);
	}
}

static DEVICE_API(counter, counter_api) = {
	.start = counter_at32_start,
	.stop = counter_at32_stop,
	.get_value = counter_at32_get_value,
	.set_alarm = counter_at32_set_alarm,
	.cancel_alarm = counter_at32_cancel_alarm,
	.set_top_value = counter_at32_set_top_value,
	.get_pending_int = counter_at32_get_pending_int,
	.get_top_value = counter_at32_get_top_value,
	.get_guard_period = counter_at32_get_guard_period,
	.set_guard_period = counter_at32_set_guard_period,
	.get_freq = counter_at32_get_freq,
};


static void tmr_irq_cc_handle(const struct device *dev, int id, int flag, int cc_irq)
{
    const struct counter_at32_config *config = dev->config;
	struct counter_at32_data *data = dev->data;
    tmr_type *tmr_x = (tmr_type *)config->reg;
  
    if(cc_irq || (data->cc_int_pending & BIT(id - 1U)))
    {
        if(cc_irq)
        {
            tmr_flag_clear(tmr_x, flag);
        }
        counter_at32_alarm_irq_handle(dev, id - 1U);
    }
}
  
void counter_at32_irq_handler(const struct device *dev)
{
	const struct counter_at32_config *config = dev->config;
    tmr_type *tmr_x = (tmr_type *)config->reg;

	/* Capture compare events */
	switch (counter_get_num_of_channels(dev)) {
	case 4U:
    tmr_irq_cc_handle(dev, 4, TMR_C4_FLAG, tmr_interrupt_flag_get(tmr_x, TMR_C4_FLAG));
		__fallthrough;
	case 3U:
    tmr_irq_cc_handle(dev, 3, TMR_C3_FLAG, tmr_interrupt_flag_get(tmr_x, TMR_C3_FLAG));
		__fallthrough;
	case 2U:
    tmr_irq_cc_handle(dev, 2, TMR_C2_FLAG, tmr_interrupt_flag_get(tmr_x, TMR_C2_FLAG));
		__fallthrough;
	case 1U:
    tmr_irq_cc_handle(dev, 1, TMR_C1_FLAG, tmr_interrupt_flag_get(tmr_x, TMR_C1_FLAG));
    default:
    break;
	}

	/* Update event */
	if (tmr_interrupt_flag_get(tmr_x, TMR_OVF_FLAG)) {
		tmr_flag_clear(tmr_x, TMR_OVF_FLAG);
		counter_at32_top_irq_handle(dev);
	}
}

#define TIMER(idx)              DT_INST_PARENT(idx)

/** TIMx instance from DT */
#define TIM(idx) ((tmr_type *)DT_REG_ADDR(TIMER(idx)))

#define TIMER_IRQ_CONFIG(n)                                                    \
	static void irq_config_##n(const struct device *dev)                   \
	{                                                                      \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, global, irq),               \
			    DT_INST_IRQ_BY_NAME(n, global, priority),          \
			    counter_at32_irq_handler, DEVICE_DT_INST_GET(n), 0);            \
		irq_enable(DT_INST_IRQ_BY_NAME(n, global, irq));               \
	}                                                                      \
	static void set_irq_pending_##n(void)                                  \
	{                                                                      \
		(NVIC_SetPendingIRQ(DT_INST_IRQ_BY_NAME(n, global, irq)));     \
	}                                                                      \
	static uint32_t get_irq_pending_##n(void)                              \
	{                                                                      \
		return NVIC_GetPendingIRQ(                                     \
			DT_INST_IRQ_BY_NAME(n, global, irq));                  \
	}

#define TIMER_IRQ_CONFIG_ADVANCED(n)                                           \
	static void irq_config_##n(const struct device *dev)                   \
	{                                                                      \
		IRQ_CONNECT((DT_INST_IRQ_BY_NAME(n, up, irq)),                 \
			    (DT_INST_IRQ_BY_NAME(n, up, priority)),            \
			    counter_at32_irq_handler, (DEVICE_DT_INST_GET(n)), 0);          \
		irq_enable((DT_INST_IRQ_BY_NAME(n, up, irq)));                 \
		IRQ_CONNECT((DT_INST_IRQ_BY_NAME(n, cc, irq)),                 \
			    (DT_INST_IRQ_BY_NAME(n, cc, priority)),            \
			    counter_at32_irq_handler, (DEVICE_DT_INST_GET(n)), 0);          \
		irq_enable((DT_INST_IRQ_BY_NAME(n, cc, irq)));                 \
	}                                                                      \
	static void set_irq_pending_##n(void)                                  \
	{                                                                      \
		(NVIC_SetPendingIRQ(DT_INST_IRQ_BY_NAME(n, cc, irq)));         \
	}                                                                      \
	static uint32_t get_irq_pending_##n(void)                              \
	{                                                                      \
		return NVIC_GetPendingIRQ(DT_INST_IRQ_BY_NAME(n, cc, irq));    \
	}

#define AT32_TIMER_INIT(n)                                                     \
	COND_CODE_1(DT_INST_PROP(n, is_advanced),                              \
		    (TIMER_IRQ_CONFIG_ADVANCED(n)), (TIMER_IRQ_CONFIG(n)));         \
	static struct counter_at32_data counter##n##_data;			  \
	static struct counter_at32_ch_data counter##n##_ch_data[DT_INST_PROP(n, channels)]; \
	static const struct counter_at32_config counter##n##_config = {           \
		.info = {.max_top_value = COND_CODE_1(                 \
					 DT_INST_PROP(n, is_32bit),            \
					 (UINT32_MAX), (UINT16_MAX)),          \
				 .flags = COUNTER_CONFIG_INFO_COUNT_UP,        \
				 .freq = 0,                                    \
				 .channels = DT_INST_PROP(n, channels)},       \
		.reg = DT_INST_REG_ADDR(n),                                    \
		.ch_data = counter##n##_ch_data,	                             \
		.clkid = DT_INST_CLOCKS_CELL(n, id),                           \
		.reset = RESET_DT_SPEC_INST_GET(n),                            \
		.prescaler = DT_INST_PROP(n, at_prescaler),                       \
		.irq_config_func = irq_config_##n,                                  \
		.set_irq_pending = set_irq_pending_##n,                        \
		.get_irq_pending = get_irq_pending_##n,                        \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, counter_at32_init_timer, NULL,                \
			      &counter##n##_data,				  \
			      &counter##n##_config,				  \
			      PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,      \
			      &counter_api);

DT_INST_FOREACH_STATUS_OKAY(AT32_TIMER_INIT)
