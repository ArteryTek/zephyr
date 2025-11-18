/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_AT32_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_AT32_H_

#define AT32_DMA_CONFIG_DIRECTION(config)	     ((config >> 6) & 0x3)
#define AT32_DMA_CONFIG_PERIPH_ADDR_INC(config)	     ((config >> 9) & 0x1)
#define AT32_DMA_CONFIG_MEMORY_ADDR_INC(config)	     ((config >> 10) & 0x1)
#define AT32_DMA_CONFIG_PERIPH_WIDTH(config)	     (1 << ((config >> 11) & 0x3))
#define AT32_DMA_CONFIG_MEMORY_WIDTH(config)	     (1 << ((config >> 13) & 0x3))
#define AT32_DMA_CONFIG_PERIPHERAL_INC_FIXED(config) ((config >> 15) & 0x1)
#define AT32_DMA_CONFIG_PRIORITY(config)	     ((config >> 16) & 0x3)

#define AT32_DMA_FEATURES_FIFO_THRESHOLD(threshold) (threshold & 0x3)

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_AT32_H_ */
