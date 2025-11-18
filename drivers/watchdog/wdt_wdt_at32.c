/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2017 RnDity Sp. z o.o.
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT at_at32_watchdog

#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <soc.h>
#include <errno.h>
#include <at32_wdt.h>
#include <at32_debug.h>

#define WDT_PRESCALER_MIN	(4U)

#define WDT_PRESCALER_MAX (256U)

#define LICK_VALUE         40000

#define WDT_RELOAD_MIN		(0x0000U)
#define WDT_RELOAD_MAX		(0x0FFFU)

struct wdt_at32_config {
	/* IWDG peripheral instance. */
	wdt_type *instance;
};

struct wdt_at32_data {
	uint32_t prescaler;
	uint32_t reload;
};

/* Minimum timeout in microseconds. */
#define WDT_TIMEOUT_MIN	(WDT_PRESCALER_MIN * (WDT_RELOAD_MIN + 1U) \
				 * USEC_PER_SEC / LICK_VALUE)

/* Maximum timeout in microseconds. */
#define WDT_TIMEOUT_MAX	((uint64_t)WDT_PRESCALER_MAX * \
				 (WDT_RELOAD_MAX + 1U) * \
				 USEC_PER_SEC / LICK_VALUE)

#define IS_WDT_TIMEOUT(__TIMEOUT__)		\
	(((__TIMEOUT__) >= WDT_TIMEOUT_MIN) &&	\
	 ((__TIMEOUT__) <= WDT_TIMEOUT_MAX))

/*
 * Status register needs 5 LICK clock cycles divided by prescaler to be updated.
 * With highest prescaler and considering clock variation, we will wait
 * maximum 6 cycles (48 ms at 32 kHz) for register update.
 */
#define WDT_SR_UPDATE_TIMEOUT	(6U * WDT_PRESCALER_MAX * \
				 MSEC_PER_SEC / LICK_VALUE)

/**
 * @brief Calculates prescaler & reload values.
 *
 * @param timeout Timeout value in microseconds.
 * @param prescaler Pointer to prescaler value.
 * @param reload Pointer to reload value.
 */
static void wdt_at32_convert_timeout(uint32_t timeout,
				       uint32_t *prescaler,
				       uint32_t *reload)
{
	uint16_t divider = 4U;
	uint8_t shift = 0U;

	/* Convert timeout to LSI clock ticks. */
	uint32_t ticks = (uint64_t)timeout * 40000 / USEC_PER_SEC;

	while ((ticks / divider) > WDT_RELOAD_MAX) {
		shift++;
		divider = 4U << shift;
	}

	/*
	 * Value of the 'shift' variable corresponds to the
	 * defines of LL_IWDG_PRESCALER_XX type.
	 */
	*prescaler = shift;
	*reload = (uint32_t)(ticks / divider) - 1U;
}

static int wdt_at32_setup(const struct device *dev, uint8_t options)
{
	struct wdt_at32_data *data = dev->data;

	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		debug_apb1_periph_mode_set(DEBUG_WDT_PAUSE, TRUE);
	}

	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		return -ENOTSUP;
	}

	/* Enable the IWDG now and write IWDG registers at the same time */
	wdt_register_write_enable(TRUE);
	wdt_divider_set( data->prescaler);
	wdt_reload_value_set(data->reload);
	wdt_counter_reload();
	wdt_enable();

	return 0;
}

static int wdt_at32_disable(const struct device *dev)
{
	/* watchdog cannot be stopped once started */
	ARG_UNUSED(dev);

	return -EPERM;
}

static int wdt_at32_install_timeout(const struct device *dev,
				      const struct wdt_timeout_cfg *config)
{
	struct wdt_at32_data *data = dev->data;
	uint32_t timeout = config->window.max * USEC_PER_MSEC;
	uint32_t prescaler = 0U;
	uint32_t reload = 0U;

	if (config->callback != NULL) {
		return -ENOTSUP;
	}
	if (data->reload) {
		/* Timeout has already been configured */
		return -ENOMEM;
	}

	/* Calculating parameters to be applied later, on setup */
	wdt_at32_convert_timeout(timeout, &prescaler, &reload);
#if 0
	if (!(IS_WDT_TIMEOUT(timeout) && IS_WDT_PRESCALER(prescaler) &&
	    IS_WDT_RELOAD(reload))) {
		/* One of the parameters provided is invalid */
		return -EINVAL;
	}
#endif
	/* Store the calculated values to write in the iwdg registers */
	data->prescaler = prescaler;
	data->reload = reload;

	/* Do not enable and update the iwdg here but during wdt_setup() */
	return 0;
}

static int wdt_at32_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(channel_id);
	ARG_UNUSED(dev);
	wdt_counter_reload();

	return 0;
}

static DEVICE_API(wdt, wdt_at32_api) = {
	.setup = wdt_at32_setup,
	.disable = wdt_at32_disable,
	.install_timeout = wdt_at32_install_timeout,
	.feed = wdt_at32_feed,
};

static int wdt_at32_init(const struct device *dev)
{
#ifndef CONFIG_WDT_DISABLE_AT_BOOT
	struct wdt_timeout_cfg config = {
		.window.max = CONFIG_WDT_AT32_INITIAL_TIMEOUT
	};
	/* Watchdog should be configured and started by `wdt_setup`*/
	wdt_at32_install_timeout(dev, &config);
	wdt_at32_setup(dev, 0); /* no option specified */
#endif
	return 0;
}

static const struct wdt_at32_config wdt_at32_dev_cfg = {
	.instance = (wdt_type *)DT_INST_REG_ADDR(0),
};
static struct wdt_at32_data wdt_at32_dev_data;

DEVICE_DT_INST_DEFINE(0, wdt_at32_init, NULL,
		    &wdt_at32_dev_data, &wdt_at32_dev_cfg,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &wdt_at32_api);
