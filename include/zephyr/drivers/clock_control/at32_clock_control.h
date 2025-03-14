/*
 * Copyright (c) 2022 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_AT32_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_AT32_H_

#include <zephyr/device.h>

/**
 * @brief Obtain a reference to the GD32 clock controller.
 *
 * There is a single clock controller in the AT32: ctrl. The device can be
 * used without checking for it to be ready since it has no initialization
 * code subject to failures.
 */
#define AT32_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(cctl))

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_AT32_H_ */
