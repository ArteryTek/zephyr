/*
 * Copyright (c) 2017 BayLibre, SAS
 * Copyright (c) 2019 Linaro Limited
 * Copyright (c) 2020 Andreas Sandberg
 * Copyright (c) 2025 Maxjta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_at32generic, CONFIG_FLASH_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/sys/barrier.h>
#include <soc.h>

#include "flash_at32.h"

#if FLASH_AT32_WRITE_BLOCK_SIZE == 8
typedef uint64_t flash_prg_t;
#elif FLASH_AT32_WRITE_BLOCK_SIZE == 4
typedef uint32_t flash_prg_t;
#elif FLASH_AT32_WRITE_BLOCK_SIZE == 2
typedef uint16_t flash_prg_t;
#elif FLASH_AT32_WRITE_BLOCK_SIZE == 1
typedef uint8_t flash_prg_t;
#else
#error Unknown write block size
#endif

static struct at32_flash_bank_type *get_bank(flash_type *regs, off_t offset)
{
#if FLASH_AT32_BANK2_SIZE
if (offset < (FLASH_AT32_SIZE_BYTE - FLASH_AT32_BANK2_SIZE)) {
	return (struct at32_flash_bank_type *)&(regs->unlock);
} else {
	return (struct at32_flash_bank_type *)&(regs->unlock2);
}
#else
	ARG_UNUSED(offset);
	return (struct at32_flash_bank_type *)&(regs->unlock);
#endif
}

static unsigned int get_page(off_t offset)
{
	return offset / FLASH_PAGE_SIZE;
}

static int z_flash_unlock(struct at32_flash_bank_type *bank)
{
	bank->unlock = FLASH_UNLOCK_KEY1;
	bank->unlock = FLASH_UNLOCK_KEY2;
	return 0;
}

static int z_flash_lock(struct at32_flash_bank_type *bank)
{
	bank->ctrl |= FLASH_CTRL_LCK;
	return 0;
}

static int is_flash_locked(struct at32_flash_bank_type *bank)
{
	return (bank->ctrl & FLASH_CTRL_LCK);
}

static void write_enable(struct at32_flash_bank_type *bank)
{
	bank->ctrl |= FLASH_CTRL_PRGM;
}

static void write_disable(struct at32_flash_bank_type *bank)
{
	bank->ctrl &= ~FLASH_CTRL_PRGM;
}

static void erase_page_begin(struct at32_flash_bank_type *bank, unsigned int page)
{
	/* Set the PER bit and select the page you wish to erase */
	bank->ctrl |= FLASH_CTRL_PGERS;
	bank->addr = FLASH_AT32_BASE_ADDRESS + page * FLASH_PAGE_SIZE;

	barrier_dsync_fence_full();

	/* Set the STRT bit */
	bank->ctrl |= FLASH_CTRL_STRT;
}

static void erase_page_end(struct at32_flash_bank_type *bank)
{
	bank->ctrl &= ~FLASH_CTRL_PGERS;
}

static int flash_at32_check_status(const struct device *dev, struct at32_flash_bank_type *bank)
{
	if (bank->sts & (FLASH_STS_PRGMFLR |
			FLASH_STS_WRPRTFLR)) {
		LOG_DBG("Flash status: error");
		bank->sts = FLASH_STS_PRGMFLR|FLASH_STS_WRPRTFLR|FLASH_STS_PRCDN;
		return -EIO;
	}
	bank->sts = FLASH_STS_PRCDN;
	return 0;
}
static int flash_at32_wait_flash_idle(const struct device *dev, struct at32_flash_bank_type *bank)
{
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(AT32_FLASH_TIMEOUT));
	bool expired = false;
	int rc;

	rc = flash_at32_check_status(dev, bank);
	if (rc < 0) {
		return -EIO;
	}

	while ((bank->sts &  FLASH_STS_BSY) == FLASH_STS_BSY) {
		if (expired) {
			LOG_ERR(" Bank1 Timeout! val: %d ms", AT32_FLASH_TIMEOUT);
			return -EIO;
		}
		/* Check if expired, but always read status register one more time.
		 * If the calling thread is pre-emptive we may have been scheduled out after reading
		 * the status register, and scheduled back after timeout has expired.
		 */
		 expired = sys_timepoint_expired(timeout);
	}

	return 0;
}

static int write_value(const struct device *dev, off_t offset,
		       flash_prg_t val)
{
	volatile flash_prg_t *flash = (flash_prg_t *)(
		offset + FLASH_AT32_BASE_ADDRESS);
	flash_type *regs = FLASH_AT32_REGS(dev);
	struct at32_flash_bank_type *bank = get_bank(regs, offset);
	int rc;
	
	z_flash_unlock(bank);

	/* if the control register is locked, do not fail silently */
	if (is_flash_locked(bank)) {
		LOG_ERR("Flash is locked");
		return -EIO;
	}

	/* Check that no Flash main memory operation is ongoing */
	rc = flash_at32_wait_flash_idle(dev, bank);
	if (rc < 0) {
		return rc;
	}

	/* Enable writing */
	write_enable(bank);

	/* Make sure the register write has taken effect */
	barrier_dsync_fence_full();

	/* Perform the data write operation at the desired memory address */
	*flash = val;

	/* Wait until the BSY bit is cleared */
	rc = flash_at32_wait_flash_idle(dev, bank);

	/* Disable writing */
	write_disable(bank);

	z_flash_lock(bank);

	return rc;
}

int flash_at32_block_erase_loop(const struct device *dev,
				 unsigned int offset,
				 unsigned int len)
{
	flash_type *regs = FLASH_AT32_REGS(dev);
	struct at32_flash_bank_type *bank = get_bank(regs, offset);
	int i, rc = 0;

	/* Check that no Flash memory operation is ongoing */
	rc = flash_at32_wait_flash_idle(dev, bank);
	if (rc == 0) {
		for (i = get_page(offset); i <= get_page(offset + len - 1); ++i) {

			bank = get_bank(regs, i * FLASH_PAGE_SIZE);
			if (is_flash_locked(bank)) {
				z_flash_unlock(bank);
			}
			erase_page_begin(bank, i);
			barrier_dsync_fence_full();
			rc = flash_at32_wait_flash_idle(dev, bank);
			erase_page_end(bank);

			z_flash_lock(bank);
			if (rc < 0) {
				break;
			}
		}
	}

	return rc;
}

int flash_at32_write_range(const struct device *dev, unsigned int offset,
			    const void *data, unsigned int len)
{
	int i, rc = 0;
	flash_prg_t value;

	/* unlock bank */

	for (i = 0; i < len / sizeof(flash_prg_t); i++) {
		memcpy(&value,
		       (const uint8_t *)data + i * sizeof(flash_prg_t),
		       sizeof(flash_prg_t));
		rc = write_value(dev, offset + i * sizeof(flash_prg_t), value);
		if (rc < 0) {
			return rc;
		}
	}

	return rc;
}

void flash_at32_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	static struct flash_pages_layout flash_layout = {
		.pages_count = 0,
		.pages_size = 0,
	};

	ARG_UNUSED(dev);

	if (flash_layout.pages_count == 0) {
		flash_layout.pages_count = (CONFIG_FLASH_SIZE * 1024) /
			FLASH_PAGE_SIZE;
		flash_layout.pages_size = FLASH_PAGE_SIZE;
	}

	*layout = &flash_layout;
	*layout_size = 1;
}
