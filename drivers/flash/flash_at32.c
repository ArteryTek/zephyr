/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2017 BayLibre, SAS.
 * Copyright (c) 2019 Centaur Analytics, Inc
 * Copyright (c) 2023 Google Inc
 * Copyright (c) 2025 Maxjta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define DT_DRV_COMPAT at_at32_flash_controller

#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/logging/log.h>

#include "flash_at32.h"
#include "at32_crm.h"

LOG_MODULE_REGISTER(flash_at32, CONFIG_FLASH_LOG_LEVEL);

/* Let's wait for double the max erase time to be sure that the operation is
 * completed.
 */

static const struct flash_parameters flash_at32_parameters = {
	.write_block_size = FLASH_AT32_WRITE_BLOCK_SIZE,
	.erase_value = 0xff,
};

bool __weak flash_at32_valid_range(const struct device *dev, off_t offset,
				    uint32_t len, bool write)
{
	if (write && !flash_at32_valid_write(offset, len)) {
		return false;
	}
	return flash_at32_range_exists(dev, offset, len);
}

int __weak flash_at32_check_configuration(void)
{
	return 0;
}

static int flash_at32_read(const struct device *dev, off_t offset,
			    void *data,
			    size_t len)
{
	if (!flash_at32_valid_range(dev, offset, len, false)) {
		LOG_ERR("Read range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	LOG_DBG("Read offset: %ld, len: %zu", (long int) offset, len);

	memcpy(data, (uint8_t *) FLASH_AT32_BASE_ADDRESS + offset, len);

	return 0;
}

static int flash_at32_erase(const struct device *dev, off_t offset,
			     size_t len)
{
	int rc;

	if (!flash_at32_valid_range(dev, offset, len, true)) {
		LOG_ERR("Erase range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	flash_at32_sem_take(dev);

	LOG_DBG("Erase offset: %ld, len: %zu", (long int) offset, len);

	rc = flash_at32_block_erase_loop(dev, offset, len);

	flash_at32_sem_give(dev);

	return rc;
}

static int flash_at32_write(const struct device *dev, off_t offset,
			     const void *data, size_t len)
{
	int rc;

	if (!flash_at32_valid_range(dev, offset, len, true)) {
		LOG_ERR("Write range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	flash_at32_sem_take(dev);

	LOG_DBG("Write offset: %ld, len: %zu", (long int) offset, len);

	rc = flash_at32_write_range(dev, offset, data, len);

	flash_at32_sem_give(dev);

	return rc;
}

#if defined(CONFIG_FLASH_EX_OP_ENABLED) && defined(CONFIG_FLASH_AT32_BLOCK_REGISTERS)
int flash_at32_control_register_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

	flash_lock();

	return 0;
}

int flash_at32_option_bytes_disable(const struct device *dev)
{
	flash_type *regs = FLASH_AT32_REGS(dev);

	ARG_UNUSED(regs);

	return -ENOTSUP;
}
#endif /* CONFIG_FLASH_AT32_BLOCK_REGISTERS */

static const struct flash_parameters *
flash_at32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_at32_parameters;
}

/* Gives the total logical device size in bytes and return 0. */
static int flash_at32_get_size(const struct device *dev, uint64_t *size)
{
	ARG_UNUSED(dev);

	*size = (uint64_t)FLASH_AT32_GET_SIZE * 1024U;

	return 0;
}

static struct flash_at32_priv flash_data = {
	.regs = (flash_type *) DT_INST_REG_ADDR(0),
};

static DEVICE_API(flash, flash_at32_api) = {
	.erase = flash_at32_erase,
	.write = flash_at32_write,
	.read = flash_at32_read,
	.get_parameters = flash_at32_get_parameters,
	.get_size = flash_at32_get_size,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_at32_page_layout,
#endif
};

static int at32_flash_init(const struct device *dev)
{
	int rc;

	flash_at32_sem_init(dev);

	LOG_DBG("Flash @0x%x initialized. BS: %zu",
		FLASH_AT32_BASE_ADDRESS,
		flash_at32_parameters.write_block_size);

	/* Check Flash configuration */
	rc = flash_at32_check_configuration();
	if (rc < 0) {
		return rc;
	}

#if ((CONFIG_FLASH_LOG_LEVEL >= LOG_LEVEL_DBG) && CONFIG_FLASH_PAGE_LAYOUT)
	const struct flash_pages_layout *layout;
	size_t layout_size;

	flash_at32_page_layout(dev, &layout, &layout_size);
	for (size_t i = 0; i < layout_size; i++) {
		LOG_DBG("Block %zu: bs: %zu count: %zu", i,
			layout[i].pages_size, layout[i].pages_count);
	}
#endif

	return 0;
}

DEVICE_DT_INST_DEFINE(0, at32_flash_init, NULL,
		    &flash_data, NULL, POST_KERNEL,
		    CONFIG_FLASH_INIT_PRIORITY, &flash_at32_api);
