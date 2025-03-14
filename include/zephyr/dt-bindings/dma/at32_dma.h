/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_AT32_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_AT32_DMA_H_

/* macros for channel-cfg */

/* direction defined on bits 6-7 */
#define AT32_DMA_CH_CFG_DIRECTION(val) ((val & 0x3) << 6)
#define AT32_DMA_MEMORY_TO_MEMORY      AT32_DMA_CH_CFG_DIRECTION(0)
#define AT32_DMA_MEMORY_TO_PERIPH      AT32_DMA_CH_CFG_DIRECTION(1)
#define AT32_DMA_PERIPH_TO_MEMORY      AT32_DMA_CH_CFG_DIRECTION(2)

/* periph increase defined on bit 9 as true/false */
#define AT32_DMA_CH_CFG_PERIPH_ADDR_INC(val) ((val & 0x1) << 9)
#define AT32_DMA_NO_PERIPH_ADDR_INC	     AT32_DMA_CH_CFG_PERIPH_ADDR_INC(0)
#define AT32_DMA_PERIPH_ADDR_INC	    AT32_DMA_CH_CFG_PERIPH_ADDR_INC(1)

/* memory increase defined on bit 10 as true/false */
#define AT32_DMA_CH_CFG_MEMORY_ADDR_INC(val) ((val & 0x1) << 10)
#define AT32_DMA_NO_MEMORY_ADDR_INC	     AT32_DMA_CH_CFG_MEMORY_ADDR_INC(0)
#define AT32_DMA_MEMORY_ADDR_INC	     AT32_DMA_CH_CFG_MEMORY_ADDR_INC(1)

/* periph data size defined on bits 11-12 */
#define AT32_DMA_CH_CFG_PERIPH_WIDTH(val) ((val & 0x3) << 11)
#define AT32_DMA_PERIPH_WIDTH_8BIT	  AT32_DMA_CH_CFG_PERIPH_WIDTH(0)
#define AT32_DMA_PERIPH_WIDTH_16BIT	  AT32_DMA_CH_CFG_PERIPH_WIDTH(1)
#define AT32_DMA_PERIPH_WIDTH_32BIT	  AT32_DMA_CH_CFG_PERIPH_WIDTH(2)

/* memory data size defined on bits 13-14 */
#define AT32_DMA_CH_CFG_MEMORY_WIDTH(val) ((val & 0x3) << 13)
#define AT32_DMA_MEMORY_WIDTH_8BIT	  AT32_DMA_CH_CFG_MEMORY_WIDTH(0)
#define AT32_DMA_MEMORY_WIDTH_16BIT	  AT32_DMA_CH_CFG_MEMORY_WIDTH(1)
#define AT32_DMA_MEMORY_WIDTH_32BIT	  AT32_DMA_CH_CFG_MEMORY_WIDTH(2)

/* priority increment offset defined on bit 15 */
#define AT32_DMA_CH_CFG_PERIPH_INC_FIXED(val) ((val & 0x1) << 15)

/* priority defined on bits 16-17 as 0, 1, 2, 3 */
#define AT32_DMA_CH_CFG_PRIORITY(val) ((val & 0x3) << 16)
#define AT32_DMA_PRIORITY_LOW	      AT32_DMA_CH_CFG_PRIORITY(0)
#define AT32_DMA_PRIORITY_MEDIUM      AT32_DMA_CH_CFG_PRIORITY(1)
#define AT32_DMA_PRIORITY_HIGH	      AT32_DMA_CH_CFG_PRIORITY(2)
#define AT32_DMA_PRIORITY_VERY_HIGH   AT32_DMA_CH_CFG_PRIORITY(3)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_AT32_DMA_H_ */
