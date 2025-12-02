/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2017 BayLibre, SAS.
 * Copyright (c) 2023 Google Inc
 * Copyright (c) 2025 Maxjta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_FLASH_FLASH_AT32_H_
#define ZEPHYR_DRIVERS_FLASH_FLASH_AT32_H_

#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>

#include "at32_flash.h"

/* Get the base address of the flash from the DTS node */
#define FLASH_AT32_BASE_ADDRESS DT_REG_ADDR(DT_INST(0, at_at32_nv_flash))

#if DT_PROP(DT_INST(0, at_at32_nv_flash), write_block_size)
#define FLASH_AT32_WRITE_BLOCK_SIZE \
	DT_PROP(DT_INST(0, at_at32_nv_flash), write_block_size)
#else
#error Flash write block size not available
	/* Flash Write block size is extracted from device tree */
	/* as flash node property 'write-block-size' */
#endif

#define AT32_FLASH_TIMEOUT	\
	(2 * DT_PROP(DT_INST(0, at_at32_nv_flash), max_erase_time))

#if DT_PROP(DT_INST(0, at_at32_nv_flash), page_size)
#define FLASH_PAGE_SIZE \
	DT_PROP(DT_INST(0, at_at32_nv_flash), page_size)
#else
#error Flash page size not available
#endif

#if DT_PROP(DT_INST(0, at_at32_nv_flash), bank2_flash_size)
#define FLASH_AT32_BANK2_SIZE \
	DT_PROP(DT_INST(0, at_at32_nv_flash), bank2_flash_size)
#else
#define FLASH_AT32_BANK2_SIZE 0
#endif

struct at32_flash_bank_type
{
	__IO uint32_t unlock;
	__IO uint32_t reserved;
	__IO uint32_t sts;
	__IO uint32_t ctrl;
	__IO uint32_t addr;
};

struct flash_at32_priv {
	flash_type *regs;
	struct k_sem sem;
};

#define FLASH_AT32_PRIV(dev) ((struct flash_at32_priv *)((dev)->data))
#define FLASH_AT32_REGS(dev) (FLASH_AT32_PRIV(dev)->regs)
#define FLASH_AT32_SIZE_BYTE (CONFIG_FLASH_SIZE * 1024)
#define FLASH_AT32_GET_SIZE (*((uint32_t*)0x1FFFF7E0))

#define FLASH_AT32_BANK1    1
#define FLASH_AT32_BANK2    1

#if defined(FLASH_OPTR_DBANK)
#define FLASH_AT32_DBANK FLASH_OPTR_DBANK
#endif /* FLASH_OPTR_DBANK */

#define  FLASH_STS_BSY                          ((uint8_t)0x01)               /*!< Busy */
#define  FLASH_STS_PRGMFLR                      ((uint8_t)0x04)               /*!< Programming Error */
#define  FLASH_STS_WRPRTFLR                     ((uint8_t)0x10)               /*!< Write Protection Error */
#define  FLASH_STS_PRCDN                        ((uint8_t)0x20)               /*!< End of operation */

#define  FLASH_CTRL_PRGM                        ((uint16_t)0x0001)            /*!< Programming */
#define  FLASH_CTRL_PGERS                       ((uint16_t)0x0002)            /*!< Page Erase */
#define  FLASH_CTRL_CHPERS                      ((uint16_t)0x0004)            /*!< Mass Erase */
#define  FLASH_CTRL_UOBPRGM                     ((uint16_t)0x0010)            /*!< Option Byte Programming */
#define  FLASH_CTRL_UOBERS                      ((uint16_t)0x0020)            /*!< Option Byte Erase */
#define  FLASH_CTRL_STRT                        ((uint16_t)0x0040)            /*!< Start */
#define  FLASH_CTRL_LCK                         ((uint16_t)0x0080)            /*!< Lock */
#define  FLASH_CTRL_UOBWE                       ((uint16_t)0x0200)            /*!< Option Bytes Write Enable */
#define  FLASH_CTRL_FLRIE                       ((uint16_t)0x0400)            /*!< Error Interrupt Enable */
#define  FLASH_CTRL_PRCDNIE                     ((uint16_t)0x1000)            /*!< End of operation interrupt enable */

#define FLASH_AT32_RDP0 0xAA
#define FLASH_at32_RDP2 0xCC
#define FLASH_AT32_RDP1 0xFF


#ifdef CONFIG_FLASH_PAGE_LAYOUT
static inline bool flash_at32_range_exists(const struct device *dev,
					    off_t offset,
					    uint32_t len)
{
	struct flash_pages_info info;

	return !(flash_get_page_info_by_offs(dev, offset, &info) ||
		 flash_get_page_info_by_offs(dev, offset + len - 1, &info));
}
#endif	/* CONFIG_FLASH_PAGE_LAYOUT */

#define flash_at32_sem_init(dev)
#define flash_at32_sem_take(dev)
#define flash_at32_sem_give(dev)

static inline bool flash_at32_valid_write(off_t offset, uint32_t len)
{
	return ((offset % FLASH_AT32_WRITE_BLOCK_SIZE == 0) &&
		(len % FLASH_AT32_WRITE_BLOCK_SIZE == 0U));
}

bool flash_at32_valid_range(const struct device *dev, off_t offset,
			     uint32_t len, bool write);

int flash_at32_write_range(const struct device *dev, unsigned int offset,
			    const void *data, unsigned int len);

int flash_at32_block_erase_loop(const struct device *dev,
				 unsigned int offset,
				 unsigned int len);

uint32_t flash_at32_option_bytes_read(const struct device *dev);

int flash_at32_option_bytes_write(const struct device *dev, uint32_t mask,
				   uint32_t value);

int flash_at32_cr_lock(const struct device *dev, bool enable);

#ifdef CONFIG_FLASH_PAGE_LAYOUT
void flash_at32_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size);
#endif

#endif /* ZEPHYR_DRIVERS_FLASH_FLASH_AT32_H_ */
