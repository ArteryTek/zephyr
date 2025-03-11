/*
 * Copyright (c) 2017 Christer Weinigel.
 * Copyright (c) 2017, I-SENSE group of ICCS
 * Copyright (c) 2025, Maxjta
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB device controller shim driver for AT32 devices
 *
 * This driver uses the AT32 USB drivers to talk to the USB
 * device controller on the AT32 family of devices using the
 * STM32Cube HAL layer.
 */

#include <soc.h>
#include <at32_hal_udc.h>
#include <string.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>


#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(usb_dc_at32);

#if DT_HAS_COMPAT_STATUS_OKAY(at_at32_otgfs) && DT_HAS_COMPAT_STATUS_OKAY(at_at32_otghs) && DT_HAS_COMPAT_STATUS_OKAY(at_at32_usbfs)
#error "Only one interface should be enabled at a time, OTG FS or OTG HS"
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(at_at32_otghs)
#define DT_DRV_COMPAT    at_at32_otghs
#elif DT_HAS_COMPAT_STATUS_OKAY(at_at32_otgfs)
#define DT_DRV_COMPAT    at_at32_otgfs
#elif DT_HAS_COMPAT_STATUS_OKAY(at_at32_usbfs)
#define DT_DRV_COMPAT    at_at32_usbfs
#endif

#define USB_BASE_ADDRESS	      DT_INST_REG_ADDR(0)
#define USB_IRQ			            DT_INST_IRQN(0)
#define USB_IRQ_PRI		          DT_INST_IRQ(0, priority)
#define USB_NUM_BIDIR_ENDPOINTS	DT_INST_PROP(0, num_in_eps)
#define USB_RAM_SIZE		DT_INST_PROP(0, ram_size)

#if defined(USB)

#define EP0_MPS 64U
#define EP_MPS 64U

/*
 * USB BTABLE is stored in the PMA. The size of BTABLE is 4 bytes
 * per endpoint.
 *
 */
#define USB_BTABLE_SIZE  (8 * USB_NUM_BIDIR_ENDPOINTS)

#else /* USB_OTG_FS/HS */


#define EP0_MPS 64

#if DT_HAS_COMPAT_STATUS_OKAY(at_at32_otghs)
#define EP_MPS 512
#elif DT_HAS_COMPAT_STATUS_OKAY(at_at32_otgfs) || DT_HAS_COMPAT_STATUS_OKAY(at_at32_usbfs)
#define EP_MPS 64
#endif

/* We need n TX IN FIFOs */
#define TX_FIFO_NUM USB_NUM_BIDIR_ENDPOINTS

/* We need a minimum size for RX FIFO - exact number seemingly determined through trial and error */
#define RX_FIFO_EP_WORDS 160

/* Allocate FIFO memory evenly between the TX FIFOs */
/* except the first TX endpoint need only 64 bytes */
#define TX_FIFO_EP_0_WORDS 16
#define TX_FIFO_WORDS (USB_RAM_SIZE / 4 - RX_FIFO_EP_WORDS - TX_FIFO_EP_0_WORDS)
/* Number of words for each remaining TX endpoint FIFO */
#define TX_FIFO_EP_WORDS (TX_FIFO_WORDS / (TX_FIFO_NUM - 1))

#endif /* USB */

/* Size of a USB SETUP packet */
#define SETUP_SIZE 8

/* Helper macros to make it easier to work with endpoint numbers */
#define EP0_IDX 0
#define EP0_IN (EP0_IDX | USB_EP_DIR_IN)
#define EP0_OUT (EP0_IDX | USB_EP_DIR_OUT)

/* Endpoint state */
struct usb_dc_at32_ep_state {
	uint16_t ep_mps;		/** Endpoint max packet size */
	uint16_t ep_pma_buf_len;	/** Previously allocated buffer size */
	uint8_t ep_type;		/** Endpoint type */
	uint8_t ep_stalled;	/** Endpoint stall flag */
	usb_dc_ep_callback cb;	/** Endpoint callback function */
	uint32_t read_count;	/** Number of bytes in read buffer  */
	uint32_t read_offset;	/** Current offset in read buffer */
	struct k_sem write_sem;	/** Write boolean semaphore */
};

/* Driver state */
struct usb_dc_at32_state {
	hal_udc_handle pcd;	/* Storage for the HAL_PCD api */
	usb_dc_status_callback status_cb; /* Status callback */
	struct usb_dc_at32_ep_state out_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	struct usb_dc_at32_ep_state in_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	uint8_t ep_buf[USB_NUM_BIDIR_ENDPOINTS][EP_MPS];
	uint16_t clkid;

#if defined(USB)
	uint32_t pma_offset;
#endif /* USB */
};

static struct usb_dc_at32_state usb_dc_at32_state;

/* Internal functions */

static struct usb_dc_at32_ep_state *usb_dc_at32_get_ep_state(uint8_t ep)
{
	struct usb_dc_at32_ep_state *ep_state_base;

	if (USB_EP_GET_IDX(ep) >= USB_NUM_BIDIR_ENDPOINTS) {
		return NULL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_state_base = usb_dc_at32_state.out_ep_state;
	} else {
		ep_state_base = usb_dc_at32_state.in_ep_state;
	}

	return ep_state_base + USB_EP_GET_IDX(ep);
}

static void usb_dc_at32_isr(const void *arg)
{
	hal_udc_irq(&usb_dc_at32_state.pcd);
}

#ifdef CONFIG_USB_DEVICE_SOF
void hal_udc_sof_callback(hal_udc_handle *hpcd)
{
	usb_dc_at32_state.status_cb(USB_DC_SOF, NULL);
}
#endif

static int usb_dc_at32_clock_enable(void)
{
  usb_dc_at32_state.clkid = DT_INST_CLOCKS_CELL(0, id);
  (void)clock_control_on(AT32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&usb_dc_at32_state.clkid);
	return 0;
}

static int usb_dc_at32_clock_disable(void)
{
	usb_dc_at32_state.clkid = DT_INST_CLOCKS_CELL(0, id);
  (void)clock_control_off(AT32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&usb_dc_at32_state.clkid);
	return 0;
}


static int usb_dc_at32_init(void)
{
	uint8_t status;
	unsigned int i;

#if defined(USB)
	usb_dc_at32_state.pcd.usb_reg = (usb_reg_type *)DT_INST_REG_ADDR(0);
	usb_dc_at32_state.pcd.udc_config.speed = USB_FULL_SPEED;
	usb_dc_at32_state.pcd.udc_config.dev_endpoints = USB_NUM_BIDIR_ENDPOINTS;
	usb_dc_at32_state.pcd.udc_config.ep0_mps = USB_EPT0_MPS_64;
#else
#if DT_HAS_COMPAT_STATUS_OKAY(at_at32_otghs)
	usb_dc_at32_state.pcd.usb_reg = (usb_reg_type *)DT_INST_REG_ADDR(0);
	usb_dc_at32_state.pcd.udc_config.speed = USB_HIGH_SPEED;
#else
	usb_dc_at32_state.pcd.usb_reg = (usb_reg_type *)DT_INST_REG_ADDR(0);
	usb_dc_at32_state.pcd.udc_config.speed = USB_FULL_SPEED;
#endif

#endif /* USB */

#ifdef CONFIG_USB_DEVICE_SOF
	usb_dc_at32_state.pcd.udc_config.sof_en = 1;
#else
  usb_dc_at32_state.pcd.udc_config.sof_en = 0;
#endif /* CONFIG_USB_DEVICE_SOF */

	LOG_DBG("hal_udc_init");
	status = hal_udc_init(&usb_dc_at32_state.pcd);
	if (status != 0) {
		LOG_ERR("hal_udc init failed, %d", (int)status);
		return -EIO;
	}

	LOG_DBG("hal_udc_stop");
	status = hal_udc_stop(&usb_dc_at32_state.pcd);
	if (status != 0) {
		LOG_ERR("PCD_Stop failed, %d", (int)status);
		return -EIO;
	}

	LOG_DBG("hal_udc_stop");
	status = hal_udc_start(&usb_dc_at32_state.pcd);
	if (status != 0) {
		LOG_ERR("hal_udc_stop failed, %d", (int)status);
		return -EIO;
	}

	usb_dc_at32_state.out_ep_state[EP0_IDX].ep_mps = EP0_MPS;
	usb_dc_at32_state.out_ep_state[EP0_IDX].ep_type = EPT_CONTROL_TYPE;
	usb_dc_at32_state.in_ep_state[EP0_IDX].ep_mps = EP0_MPS;
	usb_dc_at32_state.in_ep_state[EP0_IDX].ep_type = EPT_CONTROL_TYPE;

#if defined(USB)
	/* Start PMA configuration for the endpoints after the BTABLE. */
	usb_dc_at32_state.pma_offset = USB_BTABLE_SIZE;

	for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		k_sem_init(&usb_dc_at32_state.in_ep_state[i].write_sem, 1, 1);
	}
#else

	usb_set_rx_fifo(usb_dc_at32_state.pcd.usb_reg, RX_FIFO_EP_WORDS);
	for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		if (i == 0) {
			/* first endpoint need only 64 byte for EP_TYPE_CTRL */
			usb_set_tx_fifo(usb_dc_at32_state.pcd.usb_reg, i,
					TX_FIFO_EP_0_WORDS);
		} else {
			usb_set_tx_fifo(usb_dc_at32_state.pcd.usb_reg, i,
					TX_FIFO_EP_WORDS);
		}
		k_sem_init(&usb_dc_at32_state.in_ep_state[i].write_sem, 1, 1);
	}
#endif /* USB */

	IRQ_CONNECT(USB_IRQ, USB_IRQ_PRI,
		    usb_dc_at32_isr, 0, 0);
	irq_enable(USB_IRQ);
	return 0;
}

/* Zephyr USB device controller API implementation */

int usb_dc_attach(void)
{
	int ret;

	LOG_DBG("usb_dc_attach");
  
	ret = usb_dc_at32_clock_enable();
	if (ret) {
		return ret;
	}

	ret = usb_dc_at32_init();
	if (ret) {
		return ret;
	}

	return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	ep_state->cb = cb;

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	LOG_DBG("");

	usb_dc_at32_state.status_cb = cb;
}

int usb_dc_set_address(const uint8_t addr)
{
	uint8_t status;

	LOG_DBG("address %u (0x%02x)", addr, addr);

	status = hal_udc_set_address(&usb_dc_at32_state.pcd, addr);
	if (status != 0) {
		LOG_ERR("hal_udc_set_address failed(0x%02x), %d", addr,
			(int)status);
		return -EIO;
	}

	return 0;
}

int usb_dc_ep_start_read(uint8_t ep, uint8_t *data, uint32_t max_data_len)
{
	uint8_t status;

	LOG_DBG("ep 0x%02x, len %u", ep, max_data_len);

	/* we flush EP0_IN by doing a 0 length receive on it */
	if (!USB_EP_DIR_IS_OUT(ep) && (ep != EP0_IN || max_data_len)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	if (max_data_len > EP_MPS) {
		max_data_len = EP_MPS;
	}

	status = hal_udc_start_read(&usb_dc_at32_state.pcd, ep,
				    usb_dc_at32_state.ep_buf[USB_EP_GET_IDX(ep)],
				    max_data_len);
	if (status != 0) {
		LOG_ERR("hal_udc_start_read failed(0x%02x), %d", ep,
			(int)status);
		return -EIO;
	}

	return 0;
}

int usb_dc_ep_get_read_count(uint8_t ep, uint32_t *read_bytes)
{
	if (!USB_EP_DIR_IS_OUT(ep) || !read_bytes) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	*read_bytes = hal_udc_get_read_count(&usb_dc_at32_state.pcd, ep);

	return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data * const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

	LOG_DBG("ep %x, mps %d, type %d", cfg->ep_addr, cfg->ep_mps,
		cfg->ep_type);

	if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
		LOG_ERR("invalid endpoint configuration");
		return -1;
	}

	if (ep_idx > (USB_NUM_BIDIR_ENDPOINTS - 1)) {
		LOG_ERR("endpoint index/address out of range");
		return -1;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data * const ep_cfg)
{
	uint8_t ep = ep_cfg->ep_addr;
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("ep 0x%02x, previous ep_mps %u, ep_mps %u, ep_type %u",
		ep_cfg->ep_addr, ep_state->ep_mps, ep_cfg->ep_mps,
		ep_cfg->ep_type);
#if defined(USB)
	if (ep_cfg->ep_mps > ep_state->ep_pma_buf_len) {
		if (ep_cfg->ep_type == USB_DC_EP_ISOCHRONOUS) {
			if (USB_RAM_SIZE <=
			   (usb_dc_at32_state.pma_offset + ep_cfg->ep_mps*2)) {
				return -EINVAL;
			}
		} else if (USB_RAM_SIZE <=
			  (usb_dc_at32_state.pma_offset + ep_cfg->ep_mps)) {
			return -EINVAL;
		}

		if (ep_cfg->ep_type == USB_DC_EP_ISOCHRONOUS) {
			hal_udc_ep_config(&usb_dc_at32_state.pcd, ep, PCD_DBL_BUF,
				usb_dc_at32_state.pma_offset +
				((usb_dc_at32_state.pma_offset + ep_cfg->ep_mps) << 16));
			ep_state->ep_pma_buf_len = ep_cfg->ep_mps*2;
			usb_dc_at32_state.pma_offset += ep_cfg->ep_mps*2;
		} else {
			hal_udc_ep_config(&usb_dc_at32_state.pcd, ep, PCD_SNG_BUF,
				usb_dc_at32_state.pma_offset);
			ep_state->ep_pma_buf_len = ep_cfg->ep_mps;
			usb_dc_at32_state.pma_offset += ep_cfg->ep_mps;
		}
	}
	if (ep_cfg->ep_type == USB_DC_EP_ISOCHRONOUS) {
		ep_state->ep_mps = ep_cfg->ep_mps*2;
	} else {
		ep_state->ep_mps = ep_cfg->ep_mps;
	}
#else
	ep_state->ep_mps = ep_cfg->ep_mps;
#endif

	switch (ep_cfg->ep_type) {
	case USB_DC_EP_CONTROL:
		ep_state->ep_type = EPT_CONTROL_TYPE;
		break;
	case USB_DC_EP_ISOCHRONOUS:
		ep_state->ep_type = EPT_ISO_TYPE;
		break;
	case USB_DC_EP_BULK:
		ep_state->ep_type = EPT_BULK_TYPE;
		break;
	case USB_DC_EP_INTERRUPT:
		ep_state->ep_type = EPT_INT_TYPE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);
	uint8_t status;

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	status = hal_udc_ep_set_stall(&usb_dc_at32_state.pcd, ep);
	if (status != 0) {
		LOG_ERR("hal_udc_ep_set_stall failed(0x%02x), %d", ep,
			(int)status);
		return -EIO;
	}

	ep_state->ep_stalled = 1U;

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);
	uint8_t status;

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	status = hal_udc_ep_clear_stall(&usb_dc_at32_state.pcd, ep);
	if (status != 0) {
		LOG_ERR("hal_udc_ep_clear_stall failed(0x%02x), %d", ep,
			(int)status);
		return -EIO;
	}

	ep_state->ep_stalled = 0U;
	ep_state->read_count = 0U;

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state || !stalled) {
		return -EINVAL;
	}

	*stalled = ep_state->ep_stalled;

	return 0;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);
	uint8_t status;

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("hal_udc_ep_open(0x%02x, %u, %u)", ep, ep_state->ep_mps,
		ep_state->ep_type);

	status = hal_udc_ep_open(&usb_dc_at32_state.pcd, ep,
				 ep_state->ep_mps, ep_state->ep_type);
	if (status != 0) {
		LOG_ERR("hal_udc_ep_open failed(0x%02x), %d", ep,
			(int)status);
		return -EIO;
	}

	if (USB_EP_DIR_IS_OUT(ep) && ep != EP0_OUT) {
		return usb_dc_ep_start_read(ep,
					  usb_dc_at32_state.ep_buf[USB_EP_GET_IDX(ep)],
					  ep_state->ep_mps);
	}

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);
	uint8_t status;

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	status = hal_udc_ep_close(&usb_dc_at32_state.pcd, ep);
	if (status != 0) {
		LOG_ERR("hal_udc_ep_close failed(0x%02x), %d", ep,
			(int)status);
		return -EIO;
	}

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data,
		    const uint32_t data_len, uint32_t * const ret_bytes)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);
	uint8_t status;
	uint32_t len = data_len;
	int ret = 0;

	LOG_DBG("ep 0x%02x, len %u", ep, data_len);

	if (!ep_state || !USB_EP_DIR_IS_IN(ep)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	ret = k_sem_take(&ep_state->write_sem, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Unable to get write lock (%d)", ret);
		return -EAGAIN;
	}

	if (!k_is_in_isr()) {
		irq_disable(USB_IRQ);
	}

	if (ep == EP0_IN && len > USB_MAX_CTRL_MPS) {
		len = USB_MAX_CTRL_MPS;
	}

	status = hal_udc_ep_write(&usb_dc_at32_state.pcd, ep,
				     (void *)data, len);
	if (status != 0) {
		LOG_ERR("HAL_PCD_EP_Transmit failed(0x%02x), %d", ep,
			(int)status);
		k_sem_give(&ep_state->write_sem);
		ret = -EIO;
	}

	if (!ret && ep == EP0_IN && len > 0) {
		usb_dc_ep_start_read(ep, NULL, 0);
	}

	if (!k_is_in_isr()) {
		irq_enable(USB_IRQ);
	}

	if (!ret && ret_bytes) {
		*ret_bytes = len;
	}

	return ret;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len,
			uint32_t *read_bytes)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);
	uint32_t read_count;

	if (!ep_state) {
		LOG_ERR("Invalid Endpoint %x", ep);
		return -EINVAL;
	}

	read_count = ep_state->read_count;

	LOG_DBG("ep 0x%02x, %u bytes, %u+%u, %p", ep, max_data_len,
		ep_state->read_offset, read_count, (void *)data);

	if (!USB_EP_DIR_IS_OUT(ep)) { /* check if OUT ep */
		LOG_ERR("Wrong endpoint direction: 0x%02x", ep);
		return -EINVAL;
	}
	if (data) {
		read_count = MIN(read_count, max_data_len);
		memcpy(data, usb_dc_at32_state.ep_buf[USB_EP_GET_IDX(ep)] +
		       ep_state->read_offset, read_count);
		ep_state->read_count -= read_count;
		ep_state->read_offset += read_count;
	} else if (max_data_len) {
		LOG_ERR("Wrong arguments");
	}

	if (read_bytes) {
		*read_bytes = read_count;
	}

	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);

	if (!ep_state || !USB_EP_DIR_IS_OUT(ep)) { /* Check if OUT ep */
		LOG_ERR("Not valid endpoint: %02x", ep);
		return -EINVAL;
	}

	if (!ep_state->read_count) {
		usb_dc_ep_start_read(ep, usb_dc_at32_state.ep_buf[USB_EP_GET_IDX(ep)],
				     ep_state->ep_mps);
	}

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t * const read_bytes)
{
	if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
		return -EINVAL;
	}

	if (usb_dc_ep_read_continue(ep) != 0) {
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_flush(const uint8_t ep)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_ERR("Not implemented");

	return 0;
}

int usb_dc_ep_mps(const uint8_t ep)
{
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	return ep_state->ep_mps;
}

int usb_dc_wakeup_request(void)
{
	uint8_t status;

	status = hal_udc_wakeup_enable(&usb_dc_at32_state.pcd);
	if (status != 0) {
		return -EAGAIN;
	}

	/* Must be active from 1ms to 15ms as per reference manual. */
	k_sleep(K_MSEC(2));

	status = hal_udc_wakeup_disable(&usb_dc_at32_state.pcd);
	if (status != 0) {
		return -EAGAIN;
	}

	return 0;
}

int usb_dc_detach(void)
{
	uint8_t status;
	int ret;

	LOG_DBG("hal_udc_deinit");
	status = hal_udc_deinit(&usb_dc_at32_state.pcd);
	if (status != 0) {
		LOG_ERR("hal_udc_deinit failed, %d", (int)status);
		return -EIO;
	}

	ret = usb_dc_at32_clock_disable();
	if (ret) {
		return ret;
	}

	if (irq_is_enabled(USB_IRQ)) {
		irq_disable(USB_IRQ);
	}

	return 0;
}

int usb_dc_reset(void)
{
	LOG_ERR("Not implemented");

	return 0;
}

void hal_udc_reset_callback(hal_udc_handle *pucd)
{
	int i;

	LOG_DBG("");

	hal_udc_ep_open(&usb_dc_at32_state.pcd, EP0_IN, EP0_MPS, EPT_CONTROL_TYPE);
	hal_udc_ep_open(&usb_dc_at32_state.pcd, EP0_OUT, EP0_MPS,
			EPT_CONTROL_TYPE);

	for (i = 0; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		k_sem_give(&usb_dc_at32_state.in_ep_state[i].write_sem);
	}

	if (usb_dc_at32_state.status_cb) {
		usb_dc_at32_state.status_cb(USB_DC_RESET, NULL);
	}
}

void hal_udc_connect_callback(hal_udc_handle *pucd)
{
	LOG_DBG("");

	if (usb_dc_at32_state.status_cb) {
		usb_dc_at32_state.status_cb(USB_DC_CONNECTED, NULL);
	}
}

void hal_udc_disconnect_callback(hal_udc_handle *pucd)
{
	LOG_DBG("");

	if (usb_dc_at32_state.status_cb) {
		usb_dc_at32_state.status_cb(USB_DC_DISCONNECTED, NULL);
	}
}

void hal_udc_suspend_callback(hal_udc_handle *pucd)
{
	LOG_DBG("");

	if (usb_dc_at32_state.status_cb) {
		usb_dc_at32_state.status_cb(USB_DC_SUSPEND, NULL);
	}
}

void hal_udc_resume_callback(hal_udc_handle *pucd)
{
	LOG_DBG("");

	if (usb_dc_at32_state.status_cb) {
		usb_dc_at32_state.status_cb(USB_DC_RESUME, NULL);
	}
}

void hal_udc_data_setup_callback(hal_udc_handle *pucd)
{
	struct usb_setup_packet *setup = (void *)usb_dc_at32_state.pcd.setup_buffer;
	struct usb_dc_at32_ep_state *ep_state;

	LOG_DBG("");

	ep_state = usb_dc_at32_get_ep_state(EP0_OUT); /* can't fail for ep0 */
	__ASSERT(ep_state, "No corresponding ep_state for EP0");

	ep_state->read_count = SETUP_SIZE;
	ep_state->read_offset = 0U;
	memcpy(&usb_dc_at32_state.ep_buf[EP0_IDX],
	       usb_dc_at32_state.pcd.setup_buffer, ep_state->read_count);

	if (ep_state->cb) {
		ep_state->cb(EP0_OUT, USB_DC_EP_SETUP);

		if (!(setup->wLength == 0U) &&
		    usb_reqtype_is_to_device(setup)) {
			usb_dc_ep_start_read(EP0_OUT,
					     usb_dc_at32_state.ep_buf[EP0_IDX],
					     setup->wLength);
		}
	}
}

void hal_udc_data_out_callback(hal_udc_handle *hpcd, uint8_t epnum)
{
	uint8_t ep_idx = USB_EP_GET_IDX(epnum);
	uint8_t ep = ep_idx | USB_EP_DIR_OUT;
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);

	LOG_DBG("epnum 0x%02x, rx_count %u", epnum,
		hal_udc_get_read_count(&usb_dc_at32_state.pcd, epnum));

	/* Transaction complete, data is now stored in the buffer and ready
	 * for the upper stack (usb_dc_ep_read to retrieve).
	 */
	usb_dc_ep_get_read_count(ep, &ep_state->read_count);
	ep_state->read_offset = 0U;

	if (ep_state->cb) {
		ep_state->cb(ep, USB_DC_EP_DATA_OUT);
	}
}

void hal_udc_data_in_callback(hal_udc_handle *hpcd, uint8_t epnum)
{
	uint8_t ep_idx = USB_EP_GET_IDX(epnum);
	uint8_t ep = ep_idx | USB_EP_DIR_IN;
	struct usb_dc_at32_ep_state *ep_state = usb_dc_at32_get_ep_state(ep);

	LOG_DBG("epnum 0x%02x", epnum);

	__ASSERT(ep_state, "No corresponding ep_state for ep");

	k_sem_give(&ep_state->write_sem);

	if (ep_state->cb) {
		ep_state->cb(ep, USB_DC_EP_DATA_IN);
	}
}

