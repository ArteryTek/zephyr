/*
 * Copyright (c) 2023 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file  udc_at32.c
 * @brief AT32 USB device controller (UDC) driver
 */

#include <soc.h>
#include <at32_hal_udc.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/sys/util.h>

#include "at32f403a_407_usb.h"
#include "udc_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_at32, CONFIG_UDC_DRIVER_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(at_at32_otghs)
#define UDC_PRIV_SPEED  USB_HIGH_SPEED
#define DT_DRV_COMPAT at_at32_otghs
#define UDC_AT32_IRQ_NAME     otghs
#elif DT_HAS_COMPAT_STATUS_OKAY(at_at32_otgfs)
#define UDC_PRIV_SPEED  USB_FULL_SPEED
#define DT_DRV_COMPAT at_at32_otgfs
#define UDC_AT32_IRQ_NAME     otgfs
#elif DT_HAS_COMPAT_STATUS_OKAY(at_at32_usb)
#define UDC_PRIV_SPEED  USB_FULL_SPEED
#define DT_DRV_COMPAT at_at32_usb
#define UDC_AT32_IRQ_NAME     usb
#endif

#define UDC_AT32_IRQ		DT_INST_IRQN(0)
#define UDC_AT32_IRQ_PRI	DT_INST_IRQ(0, priority)

/*
 * Hardcode EP0 max packet size (bMaxPacketSize0) to 64,
 * which is the maximum allowed by the USB Specification
 * and supported by all AT32 USB controllers.
 */
#define UDC_AT32_EP0_MAX_PACKET_SIZE	64U

struct udc_at32_data  {
	hal_udc_handle pcd;	/* Storage for the api */
	const struct device *dev;
	uint32_t occupied_mem;
	/* wLength of SETUP packet for s-out-status */
	uint32_t ep0_out_wlength;
	int (*clk_enable)(void);
	int (*clk_disable)(void);
	struct k_thread thread_data;
	struct k_msgq msgq_data;
};

struct udc_at32_config {
	/* Controller MMIO base address */
	void *base;
	/* # of bidirectional endpoints supported */
	uint32_t num_endpoints;
	/* USB SRAM size (in bytes) */
	uint32_t dram_size;
	/* Global USB interrupt IRQn */
	uint32_t irqn;
	/* PHY selected for use by instance */
	uint32_t selected_phy;
	/* Speed selected for use by instance */
	uint32_t selected_speed;
	/* Maximal packet size allowed for endpoints */
	uint16_t ep_mps;
};

enum udc_at32_msg_type {
	UDC_AT32_MSG_SETUP,
	UDC_AT32_MSG_DATA_OUT,
	UDC_AT32_MSG_DATA_IN,
};

struct udc_at32_msg {
	uint8_t type;
	uint8_t ep;
	uint16_t rx_count;
};

static void udc_at32_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_at32_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

#define hpcd2data(hpcd) CONTAINER_OF(hpcd, struct udc_at32_data, pcd);

void hal_udc_reset_callback(hal_udc_handle *hpcd)
{
	struct udc_at32_data *priv = hpcd2data(hpcd);
	const struct device *dev = priv->dev;
	struct udc_ep_config *ep_cfg;
	int __maybe_unused status;

	/* Re-Enable control endpoints */
	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	if (ep_cfg != NULL && ep_cfg->stat.enabled) {
		status = hal_udc_ep_open(&priv->pcd, USB_CONTROL_EP_OUT,
					 UDC_AT32_EP0_MAX_PACKET_SIZE,
					 EPT_CONTROL_TYPE);
		__ASSERT_NO_MSG(status == 0);
	}

	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	if (ep_cfg != NULL && ep_cfg->stat.enabled) {
		status = hal_udc_ep_open(&priv->pcd, USB_CONTROL_EP_IN,
					 UDC_AT32_EP0_MAX_PACKET_SIZE,
					 EPT_CONTROL_TYPE);
		__ASSERT_NO_MSG(status == 0);
	}

	udc_set_suspended(dev, false);
	udc_submit_event(priv->dev, UDC_EVT_RESET, 0);
}

void hal_udc_connect_callback(hal_udc_handle *hpcd)
{
	struct udc_at32_data *priv = hpcd2data(hpcd);

	udc_submit_event(priv->dev, UDC_EVT_VBUS_READY, 0);
}

void hal_udc_disconnect_callback(hal_udc_handle *hpcd)
{
	struct udc_at32_data *priv = hpcd2data(hpcd);

	udc_submit_event(priv->dev, UDC_EVT_VBUS_REMOVED, 0);
}

void hal_udc_suspend_callback(hal_udc_handle *hpcd)
{
	struct udc_at32_data *priv = hpcd2data(hpcd);

	udc_set_suspended(priv->dev, true);
	udc_submit_event(priv->dev, UDC_EVT_SUSPEND, 0);
}

void hal_udc_resume_callback(hal_udc_handle *hpcd)
{
	struct udc_at32_data *priv = hpcd2data(hpcd);

	udc_set_suspended(priv->dev, false);
	udc_submit_event(priv->dev, UDC_EVT_RESUME, 0);
}

void hal_udc_data_setup_callback(hal_udc_handle *hpcd)
{
	struct udc_at32_data *priv = hpcd2data(hpcd);
	struct udc_at32_msg msg = {.type = UDC_AT32_MSG_SETUP};
	int err;

	err = k_msgq_put(&priv->msgq_data, &msg, K_NO_WAIT);

	if (err < 0) {
		LOG_ERR("UDC Message queue overrun");
	}
}

void hal_udc_sof_callback(hal_udc_handle *hpcd)
{
	struct udc_at32_data *priv = hpcd2data(hpcd);

	udc_submit_sof_event(priv->dev);
}

/*
 * Prepare OUT EP0 for reception.
 *
 * @param dev		USB controller
 * @param length	wLength from SETUP packet for s-out-status
 *                      0 for s-in-status ZLP
 */
static int udc_at32_prep_out_ep0_rx(const struct device *dev, const size_t length)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;
	uint32_t buf_size;

	udc_ep_set_busy(ep_cfg, true);

	/*
	 * Make sure OUT EP0 can receive bMaxPacketSize0 bytes
	 * from each Data packet by rounding up allocation size
	 * even if "device behaviour is undefined if the host
	 * should send more data than specified in wLength"
	 * according to the USB Specification.
	 *
	 * Note that ROUND_UP() will return 0 for ZLP.
	 */
	buf_size = ROUND_UP(length, UDC_AT32_EP0_MAX_PACKET_SIZE);

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, buf_size);
	if (buf == NULL) {
		return -ENOMEM;
	}

	k_fifo_put(&ep_cfg->fifo, buf);

	/*
	 * Keep track of how much data we're expecting from
	 * host so we know when the transfer is complete.
	 * Unlike other endpoints, this bookkeeping isn't
	 * done by the HAL for OUT EP0.
	 */
	priv->ep0_out_wlength = length;

	/* Don't try to receive more than bMaxPacketSize0 */
	if (hal_udc_start_read(&priv->pcd, ep_cfg->addr, net_buf_tail(buf),
			       UDC_AT32_EP0_MAX_PACKET_SIZE) != 0) {
		return -EIO;
	}

	return 0;
}

static void udc_at32_flush_tx_fifo(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	int __maybe_unused status;

	status = hal_udc_start_read(&priv->pcd, ep_cfg->addr, NULL, 0);
	__ASSERT_NO_MSG(status == 0);
}

static int udc_at32_tx(const struct device *dev, struct udc_ep_config *ep_cfg,
			struct net_buf *buf)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	int status;
	uint8_t *data;
	uint32_t len;

	LOG_DBG("TX ep 0x%02x len %u", ep_cfg->addr, buf->len);

	if (udc_ep_is_busy(ep_cfg)) {
		return 0;
	}

	data = buf->data;
	len = buf->len;

	if (ep_cfg->addr == USB_CONTROL_EP_IN) {
		len = MIN(UDC_AT32_EP0_MAX_PACKET_SIZE, buf->len);
	}

	buf->data += len;
	buf->len -= len;

	status = hal_udc_ep_write(&priv->pcd, ep_cfg->addr, data, len);
	if (status != 0) {
		LOG_ERR("hal_udc_ep_write failed(0x%02x), %d", ep_cfg->addr, (int)status);
		return -EIO;
	}

	udc_ep_set_busy(ep_cfg, true);

	if (ep_cfg->addr == USB_CONTROL_EP_IN && len > 0U) {
		/* Wait for an empty package from the host.
		 * This also flushes the TX FIFO to the host.
		 */
		if (DT_HAS_COMPAT_STATUS_OKAY(at_at32_usb)) {
			udc_at32_flush_tx_fifo(dev);
		} else {
			udc_at32_prep_out_ep0_rx(dev, 0);
		}
	}

	return 0;
}

static int udc_at32_rx(const struct device *dev, struct udc_ep_config *ep_cfg,
			struct net_buf *buf)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	int status;

	/* OUT EP0 requires special logic! */
	__ASSERT_NO_MSG(ep_cfg->addr != USB_CONTROL_EP_OUT);

	LOG_DBG("RX ep 0x%02x len %u", ep_cfg->addr, buf->size);

	if (udc_ep_is_busy(ep_cfg)) {
		return 0;
	}

	status = hal_udc_start_read(&priv->pcd, ep_cfg->addr, buf->data, buf->size);
	if (status != 0) {
		LOG_ERR("hal_udc_start_read failed(0x%02x), %d", ep_cfg->addr, (int)status);
		return -EIO;
	}

	udc_ep_set_busy(ep_cfg, true);

	return 0;
}

void hal_udc_data_out_callback(hal_udc_handle *hpcd, uint8_t epnum)
{
	uint32_t rx_count = hal_udc_get_read_count(hpcd, epnum);
	struct udc_at32_data *priv = hpcd2data(hpcd);
	struct udc_at32_msg msg = {
		.type = UDC_AT32_MSG_DATA_OUT,
		.ep = epnum,
		.rx_count = rx_count,
	};
	int err;

	err = k_msgq_put(&priv->msgq_data, &msg, K_NO_WAIT);
	if (err != 0) {
		LOG_ERR("UDC Message queue overrun");
	}
}

void hal_udc_data_in_callback(hal_udc_handle *hpcd, uint8_t epnum)
{
	struct udc_at32_data *priv = hpcd2data(hpcd);
	struct udc_at32_msg msg = {
		.type = UDC_AT32_MSG_DATA_IN,
		.ep = epnum,
	};
	int err;

	err = k_msgq_put(&priv->msgq_data, &msg, K_NO_WAIT);
	if (err != 0) {
		LOG_ERR("UDC Message queue overrun");
	}
}

static void handle_msg_data_out(struct udc_at32_data *priv, uint8_t epnum, uint16_t rx_count)
{
	const struct device *dev = priv->dev;
	struct udc_ep_config *ep_cfg;
	uint8_t ep = epnum | USB_EP_DIR_OUT;
	struct net_buf *buf;

	LOG_DBG("DataOut ep 0x%02x",  ep);

	ep_cfg = udc_get_ep_cfg(dev, ep);

	buf = udc_buf_peek(ep_cfg);
	if (unlikely(buf == NULL)) {
		LOG_ERR("ep 0x%02x queue is empty", ep);
		udc_ep_set_busy(ep_cfg, false);
		return;
	}

	/* HAL copies data - we just need to update bookkeeping */
	net_buf_add(buf, rx_count);

	if (ep == USB_CONTROL_EP_OUT) {
		/*
		 * OUT EP0 is used for two purposes:
		 *  - receive 'out' Data packets during s-(out)-status
		 *  - receive Status OUT ZLP during s-in-(status)
		 */
		if (udc_ctrl_stage_is_status_out(dev)) {
			/* s-in-status completed */
			__ASSERT_NO_MSG(rx_count == 0);
			udc_ctrl_update_stage(dev, buf);
			udc_ctrl_submit_status(dev, buf);
		} else {
			/* Verify that host did not send more data than it promised */
			__ASSERT(buf->len <= priv->ep0_out_wlength,
				 "Received more data from Host than expected!");

			/* Check if the data stage is complete */
			if (buf->len < priv->ep0_out_wlength) {
				int __maybe_unused status;

				/* Not yet - prepare to receive more data and wait */
				status = hal_udc_start_read(&priv->pcd, ep_cfg->addr,
							    net_buf_tail(buf),
							    UDC_AT32_EP0_MAX_PACKET_SIZE);
				__ASSERT_NO_MSG(status == HAL_OK);
				return;
			} /* else: buf->len == priv->ep0_out_wlength */

			/*
			 * Data stage is complete: update to next step
			 * which should be Status IN, then submit the
			 * Setup+Data phase buffers to UDC stack and
			 * let it handle the next stage.
			 */
			udc_ctrl_update_stage(dev, buf);
			__ASSERT_NO_MSG(udc_ctrl_stage_is_status_in(dev));
			udc_ctrl_submit_s_out_status(dev, buf);
		}
	} else {
		udc_submit_ep_event(dev, buf, 0);
	}

	/* Buffer was filled and submitted - remove it from queue */
	(void)udc_buf_get(ep_cfg);

	/* Endpoint is no longer busy */
	udc_ep_set_busy(ep_cfg, false);

	/* Prepare next transfer for EP if its queue is not empty */
	buf = udc_buf_peek(ep_cfg);
	if (buf != NULL) {
		/*
		 * Only the driver is allowed to queue transfers on OUT EP0,
		 * and it should only be doing so once per Control transfer.
		 * If it has a queued transfer, something must be wrong.
		 */
		__ASSERT(ep_cfg->addr != USB_CONTROL_EP_OUT,
			 "OUT EP0 should never have pending transfers!");

		udc_at32_rx(dev, ep_cfg, buf);
	}
}

static void handle_msg_data_in(struct udc_at32_data *priv, uint8_t epnum)
{
	const struct device *dev = priv->dev;
	struct udc_ep_config *ep_cfg;
	uint8_t ep = epnum | USB_EP_DIR_IN;
	struct net_buf *buf;
	int status;

	LOG_DBG("DataIn ep 0x%02x",  ep);

	ep_cfg = udc_get_ep_cfg(dev, ep);
	udc_ep_set_busy(ep_cfg, false);

	buf = udc_buf_peek(ep_cfg);
	if (unlikely(buf == NULL)) {
		return;
	}

	if (ep == USB_CONTROL_EP_IN && buf->len > 0U) {
		uint32_t len = MIN(UDC_AT32_EP0_MAX_PACKET_SIZE, buf->len);

		status = hal_udc_ep_write(&priv->pcd, ep, buf->data, len);
		if (status != 0) {
			LOG_ERR("hal_udc_ep_write failed: %d", status);
			__ASSERT_NO_MSG(0);
			return;
		}

		buf->len -= len;
		buf->data += len;

		return;
	}

	if (udc_ep_buf_has_zlp(buf)) {
		udc_ep_buf_clear_zlp(buf);
		status = hal_udc_ep_write(&priv->pcd, ep, buf->data, 0);
		if (status != 0) {
			LOG_ERR("hal_udc_ep_write failed: %d", status);
			__ASSERT_NO_MSG(0);
		}

		return;
	}

	udc_buf_get(ep_cfg);

	if (ep == USB_CONTROL_EP_IN) {
		if (udc_ctrl_stage_is_status_in(dev) ||
		    udc_ctrl_stage_is_no_data(dev)) {
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_out(dev)) {
			/*
			 * IN transfer finished, release buffer,
			 * control OUT buffer should be already fed.
			 */
			net_buf_unref(buf);
		}

		return;
	}

	udc_submit_ep_event(dev, buf, 0);

	buf = udc_buf_peek(ep_cfg);
	if (buf != NULL) {
		udc_at32_tx(dev, ep_cfg, buf);
	}
}

static void handle_msg_setup(struct udc_at32_data *priv)
{
	struct usb_setup_packet *setup = (void *)priv->pcd.setup_buffer;
	const struct device *dev = priv->dev;
	struct net_buf *buf;
	int err;

	/* Drop all transfers in control endpoints queue upon new SETUP */
	buf = udc_buf_get_all(udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
	if (buf != NULL) {
		net_buf_unref(buf);
	}

	buf = udc_buf_get_all(udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
	if (buf != NULL) {
		net_buf_unref(buf);
	}

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, sizeof(struct usb_setup_packet));
	if (buf == NULL) {
		LOG_ERR("Failed to allocate for setup");
		return;
	}

	udc_ep_buf_set_setup(buf);
	net_buf_add_mem(buf, setup, sizeof(struct usb_setup_packet));

	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_data_out(dev)) {
		/*  Allocate and feed buffer for data OUT stage */
		err = udc_at32_prep_out_ep0_rx(dev, udc_data_stage_length(buf));
		if (err == -ENOMEM) {
			udc_submit_ep_event(dev, buf, err);
		}
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		udc_ctrl_submit_s_in_status(dev);
	} else {
		udc_ctrl_submit_s_status(dev);
	}
}

static void udc_at32_thread_handler(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct udc_at32_data *priv = udc_get_private(dev);
	struct udc_at32_msg msg;

	while (true) {
		k_msgq_get(&priv->msgq_data, &msg, K_FOREVER);
		switch (msg.type) {
		case UDC_AT32_MSG_SETUP:
			handle_msg_setup(priv);
			break;
		case UDC_AT32_MSG_DATA_IN:
			handle_msg_data_in(priv, msg.ep);
			break;
		case UDC_AT32_MSG_DATA_OUT:
			handle_msg_data_out(priv, msg.ep, msg.rx_count);
			break;
		}
	}
}

static void udc_at32_irq(const struct device *dev)
{
	const struct udc_at32_data *priv =  udc_get_private(dev);

	/* HAL irq handler will call the related above callback */
	hal_udc_irq((hal_udc_handle *)&priv->pcd);
}

int udc_at32_init(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	const struct udc_at32_config *cfg = dev->config;
	int status;

	if (priv->clk_enable != NULL && priv->clk_enable() != 0) {
		LOG_ERR("Error enabling clock(s)");
		return -EIO;
	}

	/* Wipe and (re)initialize HAL context */
	memset(&priv->pcd, 0, sizeof(priv->pcd));

	priv->pcd.usb_reg = cfg->base;
	priv->pcd.udc_config.speed = cfg->selected_speed;

	status = hal_udc_init(&priv->pcd);
	if (status != 0) {
		LOG_ERR("hal_udc_init failed, %d", (int)status);
		return -EIO;
	}

	if (hal_udc_stop(&priv->pcd) != 0) {
		return -EIO;
	}

	return 0;
}

#if defined(USB)
static inline void udc_at32_mem_init(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	const struct udc_at32_config *cfg = dev->config;

	/**
	 * Endpoint configuration table is placed at the
	 * beginning of Private Memory Area and consumes
	 * 8 bytes for each endpoint.
	 */
	priv->occupied_mem = 8 * cfg->num_endpoints;
}

static int udc_at32_ep_mem_config(const struct device *dev,
				   struct udc_ep_config *ep_cfg,
				   bool enable)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	const struct udc_at32_config *cfg = dev->config;
	uint32_t size;

	size = MIN(udc_mps_ep_size(ep_cfg), cfg->ep_mps);

	if (!enable) {
		priv->occupied_mem -= size;
		return 0;
	}

	if (priv->occupied_mem + size >= cfg->dram_size) {
		LOG_ERR("Unable to allocate FIFO for 0x%02x", ep_cfg->addr);
		return -ENOMEM;
	}

	/* Configure PMA offset for the endpoint */
	hal_udc_ep_config(&priv->pcd, ep_cfg->addr, 0,
				priv->occupied_mem);

	priv->occupied_mem += size;

	return 0;
}
#else
static void udc_at32_mem_init(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	const struct udc_at32_config *cfg = dev->config;
	uint32_t rxfifo_size; /* in words */
	int __maybe_unused status;

	LOG_DBG("DRAM size: %uB", cfg->dram_size);

	/*
	 * In addition to the user-provided baseline, RxFIFO should fit:
	 *	- Global OUT NAK (1 word)
	 *	- Received packet information (1 word)
	 *	- Transfer complete status information (2 words per OUT endpoint)
	 *
	 * Align user-provided baseline up to 32-bit word size then
	 * add this "fixed" overhead to obtain the final RxFIFO size.
	 */
	rxfifo_size = DIV_ROUND_UP(CONFIG_UDC_AT32_OTG_RXFIFO_BASELINE_SIZE, 4U);
	rxfifo_size += 2U; /* Global OUT NAK and Rx packet info */
	rxfifo_size += 2U * cfg->num_endpoints;

	LOG_DBG("RxFIFO size: %uB", rxfifo_size * 4U);

	usb_set_rx_fifo(&priv->pcd.usb_reg, rxfifo_size);

	priv->occupied_mem = rxfifo_size * 4U;

	/* For EP0 TX, reserve only one MPS */
	usb_set_tx_fifo(&priv->pcd.usb_reg, 0,
				     DIV_ROUND_UP(UDC_AT32_EP0_MAX_PACKET_SIZE, 4U));

	priv->occupied_mem += UDC_AT32_EP0_MAX_PACKET_SIZE;

	/* Reset TX allocs */
	for (unsigned int i = 1U; i < cfg->num_endpoints; i++) {
		usb_set_tx_fifo(&priv->pcd.usb_reg, i, 0);
	}
}

static int udc_at32_ep_mem_config(const struct device *dev,
				   struct udc_ep_config *ep_cfg,
				   bool enable)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	const struct udc_at32_config *cfg = dev->config;
	unsigned int words;

	if (!USB_EP_DIR_IS_IN(ep_cfg->addr) || USB_EP_GET_IDX(ep_cfg->addr) == 0U) {
		return 0;
	}

	words = DIV_ROUND_UP(MIN(udc_mps_ep_size(ep_cfg), cfg->ep_mps), 4U);
	words = (words <= 64) ? words * 2 : words;

	if (!enable) {
		if (priv->occupied_mem >= (words * 4)) {
			priv->occupied_mem -= (words * 4);
		}
		usb_set_tx_fifo(&priv->pcd.usb_reg, USB_EP_GET_IDX(ep_cfg->addr), 0);

		return 0;
	}

	if (cfg->dram_size - priv->occupied_mem < words * 4) {
		LOG_ERR("Unable to allocate FIFO for 0x%02x", ep_cfg->addr);
		return -ENOMEM;
	}

	usb_set_tx_fifo(&priv->pcd.usb_reg, USB_EP_GET_IDX(ep_cfg->addr), words);

	priv->occupied_mem += words * 4;

	return 0;
}
#endif

static int udc_at32_enable(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	const struct udc_at32_config *cfg = dev->config;
	int status;
	int ret;

	LOG_DBG("Enable UDC");

	udc_at32_mem_init(dev);

	status = hal_udc_start(&priv->pcd);
	if (status != 0) {
		LOG_ERR("hal_udc_start failed, %d", (int)status);
		return -EIO;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				     USB_EP_TYPE_CONTROL,
				     UDC_AT32_EP0_MAX_PACKET_SIZE, 0);
	if (ret != 0) {
		LOG_ERR("Failed enabling ep 0x%02x", USB_CONTROL_EP_OUT);
		return ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				     USB_EP_TYPE_CONTROL,
				     UDC_AT32_EP0_MAX_PACKET_SIZE, 0);
	if (ret != 0) {
		LOG_ERR("Failed enabling ep 0x%02x", USB_CONTROL_EP_IN);
		return ret;
	}

	irq_enable(cfg->irqn);

	return 0;
}

static int udc_at32_disable(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	const struct udc_at32_config *cfg = dev->config;
	int status;

	irq_disable(cfg->irqn);

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT) != 0) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN) != 0) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	status = hal_udc_stop(&priv->pcd);
	if (status != 0) {
		LOG_ERR("hal_udc_stop failed, %d", (int)status);
		return -EIO;
	}

	return 0;
}

static int udc_at32_shutdown(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	const struct udc_at32_config *cfg = dev->config;
	int status;

	status = hal_udc_deinit(&priv->pcd);
	if (status != 0) {
		LOG_ERR("hal_udc_deinit failed, %d", (int)status);
		/* continue anyway */
	}

	if (priv->clk_disable != NULL && priv->clk_disable() != 0) {
		LOG_ERR("Error disabling clock(s)");
		/* continue anyway */
	}

	if (irq_is_enabled(cfg->irqn)) {
		irq_disable(cfg->irqn);
	}

	return 0;
}

static int udc_at32_set_address(const struct device *dev, const uint8_t addr)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	int status;

	LOG_DBG("Set Address %u", addr);

	status = hal_udc_set_address(&priv->pcd, addr);
	if (status != 0) {
		LOG_ERR("hal_udc_set_address failed(0x%02x), %d",
			addr, (int)status);
		return -EIO;
	}

	return 0;
}

static int udc_at32_host_wakeup(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);

	hal_udc_wakeup_enable(&priv->pcd);

	/* Must be active from 1ms to 15ms as per reference manual. */
	k_sleep(K_MSEC(2));

	hal_udc_wakeup_disable(&priv->pcd);

	udc_set_suspended(dev, false);
	udc_submit_event(dev, UDC_EVT_RESUME, 0);

	return 0;
}

static int udc_at32_ep_enable(const struct device *dev,
			       struct udc_ep_config *ep_cfg)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	int status;
	uint8_t ep_type;
	int ret;

	LOG_DBG("Enable ep 0x%02x", ep_cfg->addr);

	switch (ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_CONTROL:
		ep_type = EPT_CONTROL_TYPE;
		break;
	case USB_EP_TYPE_BULK:
		ep_type = EPT_BULK_TYPE;
		break;
	case USB_EP_TYPE_INTERRUPT:
		ep_type = EPT_INT_TYPE;
		break;
	case USB_EP_TYPE_ISO:
		ep_type = EPT_ISO_TYPE;
		break;
	default:
		return -EINVAL;
	}

	ret = udc_at32_ep_mem_config(dev, ep_cfg, true);
	if (ret != 0) {
		return ret;
	}

	status = hal_udc_ep_open(&priv->pcd, ep_cfg->addr,
				 udc_mps_ep_size(ep_cfg), ep_type);
	if (status != 0) {
		LOG_ERR("hal_udc_ep_open failed(0x%02x), %d",
			ep_cfg->addr, (int)status);
		return -EIO;
	}

	return 0;
}

static int udc_at32_ep_disable(const struct device *dev,
			      struct udc_ep_config *ep_cfg)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	int status;

	LOG_DBG("Disable ep 0x%02x", ep_cfg->addr);

	status = hal_udc_ep_close(&priv->pcd, ep_cfg->addr);
	if (status != 0) {
		LOG_ERR("hal_udc_ep_close failed(0x%02x), %d",
			ep_cfg->addr, (int)status);
		return -EIO;
	}

	return udc_at32_ep_mem_config(dev, ep_cfg, false);
}

static int udc_at32_ep_set_halt(const struct device *dev,
				 struct udc_ep_config *ep_cfg)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	int status;

	LOG_DBG("Halt ep 0x%02x", ep_cfg->addr);

	status = hal_udc_ep_set_stall(&priv->pcd, ep_cfg->addr);
	if (status != 0) {
		LOG_ERR("hal_udc_ep_set_stall failed(0x%02x), %d",
			ep_cfg->addr, (int)status);
		return -EIO;
	}

	/* Mark endpoint as halted if not control EP */
	if (USB_EP_GET_IDX(ep_cfg->addr) != 0U) {
		ep_cfg->stat.halted = true;
	}

	return 0;
}

static int udc_at32_ep_clear_halt(const struct device *dev,
				   struct udc_ep_config *ep_cfg)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	int status;
	struct net_buf *buf;

	LOG_DBG("Clear halt for ep 0x%02x", ep_cfg->addr);

	status = hal_udc_ep_clear_stall(&priv->pcd, ep_cfg->addr);
	if (status != 0) {
		LOG_ERR("hal_udc_ep_clear_stall failed(0x%02x), %d",
			ep_cfg->addr, (int)status);
		return -EIO;
	}

	/* Clear halt bit from endpoint status */
	ep_cfg->stat.halted = false;

	/* Check if there are transfers queued for EP */
	buf = udc_buf_peek(ep_cfg);
	if (buf != NULL) {
		/*
		 * There is at least one transfer pending.
		 * IN EP transfer can be started only if not busy;
		 * OUT EP transfer should be prepared only if busy.
		 */
		const bool busy = udc_ep_is_busy(ep_cfg);

		if (USB_EP_DIR_IS_IN(ep_cfg->addr) && !busy) {
			udc_at32_tx(dev, ep_cfg, buf);
		} else if (USB_EP_DIR_IS_OUT(ep_cfg->addr) && busy) {
			udc_at32_rx(dev, ep_cfg, buf);
		}
	}
	return 0;
}

static int udc_at32_ep_flush(const struct device *dev,
			      struct udc_ep_config *ep_cfg)
{
	struct udc_at32_data *priv = udc_get_private(dev);

	hal_udc_dp_flush(&priv->pcd, ep_cfg->addr);
	return 0;
}

static int udc_at32_ep_enqueue(const struct device *dev,
				struct udc_ep_config *ep_cfg,
				struct net_buf *buf)
{
	unsigned int lock_key;
	int ret = 0;

	udc_buf_put(ep_cfg, buf);

	lock_key = irq_lock();

	if (USB_EP_DIR_IS_IN(ep_cfg->addr)) {
		if (ep_cfg->stat.halted) {
			LOG_DBG("skip enqueue for halted ep 0x%02x", ep_cfg->addr);
		} else {
			ret = udc_at32_tx(dev, ep_cfg, buf);
		}
	} else {
		ret = udc_at32_rx(dev, ep_cfg, buf);
	}

	irq_unlock(lock_key);

	return ret;
}

static int udc_at32_ep_dequeue(const struct device *dev,
				struct udc_ep_config *ep_cfg)
{
	struct net_buf *buf;

	udc_at32_ep_flush(dev, ep_cfg);

	buf = udc_buf_get_all(ep_cfg);
	if (buf != NULL) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	udc_ep_set_busy(ep_cfg, false);

	return 0;
}

static enum udc_bus_speed udc_at32_device_speed(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);

	/*
	 * N.B.: pcd.Init.speed is used here on purpose instead
	 * of udc_at32_config::selected_speed because HAL updates
	 * this field after USB enumeration to reflect actual bus speed.
	 */

	if (priv->pcd.udc_config.speed == USB_HIGH_SPEED) {
		return UDC_BUS_SPEED_HS;
	}

	if (priv->pcd.udc_config.speed == USB_FULL_SPEED) {
		return UDC_BUS_SPEED_FS;
	}

	return UDC_BUS_UNKNOWN;
}

static const struct udc_api udc_at32_api = {
	.lock = udc_at32_lock,
	.unlock = udc_at32_unlock,
	.init = udc_at32_init,
	.enable = udc_at32_enable,
	.disable = udc_at32_disable,
	.shutdown = udc_at32_shutdown,
	.set_address = udc_at32_set_address,
	.host_wakeup = udc_at32_host_wakeup,
	.ep_try_config = NULL,
	.ep_enable = udc_at32_ep_enable,
	.ep_disable = udc_at32_ep_disable,
	.ep_set_halt = udc_at32_ep_set_halt,
	.ep_clear_halt = udc_at32_ep_clear_halt,
	.ep_enqueue = udc_at32_ep_enqueue,
	.ep_dequeue = udc_at32_ep_dequeue,
	.device_speed = udc_at32_device_speed,
};

/* ----------------- Instance/Device specific data ----------------- */
#define USB_NUM_BIDIR_ENDPOINTS	DT_INST_PROP(0, num_bidir_endpoints)
#define USB_RAM_SIZE		DT_INST_PROP(0, ram_size)

static struct udc_at32_data udc0_priv;

static struct udc_data udc0_data = {
	.mutex = Z_MUTEX_INITIALIZER(udc0_data.mutex),
	.priv = &udc0_priv,
};

static const struct udc_at32_config udc0_cfg  = {
	.base = (void *)DT_INST_REG_ADDR(0),
	.num_endpoints = USB_NUM_BIDIR_ENDPOINTS,
	.dram_size = USB_RAM_SIZE,
	.irqn = UDC_AT32_IRQ,
	.ep_mps = 1023,
	.selected_speed = UDC_PRIV_SPEED
};

static uint32_t clkid = DT_INST_CLOCKS_CELL(0, id);

static int priv_clock_enable(void)
{
	if (clock_control_on(AT32_CLOCK_CONTROLLER, &clkid) != 0) {
		LOG_ERR("Unable to enable USB clock");
		return -EIO;
	}

	return 0;
}

static int priv_clock_disable(void)
{
	(void)clock_control_off(AT32_CLOCK_CONTROLLER, &clkid);

	return 0;
}

static struct udc_ep_config ep_cfg_in[DT_INST_PROP(0, num_bidir_endpoints)];
static struct udc_ep_config ep_cfg_out[DT_INST_PROP(0, num_bidir_endpoints)];

static char udc_msgq_buf_0[CONFIG_UDC_AT32_MAX_QMESSAGES * sizeof(struct udc_at32_msg)];

K_THREAD_STACK_DEFINE(udc_at32_stack_0, CONFIG_UDC_AT32_STACK_SIZE);

static int udc_at32_driver_init0(const struct device *dev)
{
	struct udc_at32_data *priv = udc_get_private(dev);
	const struct udc_at32_config *cfg = dev->config;
	struct udc_data *data = dev->data;
	int err;

	for (unsigned int i = 0; i < ARRAY_SIZE(ep_cfg_out); i++) {
		ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			ep_cfg_out[i].caps.control = 1;
			ep_cfg_out[i].caps.mps = UDC_AT32_EP0_MAX_PACKET_SIZE;
		} else {
			ep_cfg_out[i].caps.bulk = 1;
			ep_cfg_out[i].caps.interrupt = 1;
			ep_cfg_out[i].caps.iso = 1;
			ep_cfg_out[i].caps.mps = cfg->ep_mps;
		}

		ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(ep_cfg_in); i++) {
		ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			ep_cfg_in[i].caps.control = 1;
			ep_cfg_in[i].caps.mps = UDC_AT32_EP0_MAX_PACKET_SIZE;
		} else {
			ep_cfg_in[i].caps.bulk = 1;
			ep_cfg_in[i].caps.interrupt = 1;
			ep_cfg_in[i].caps.iso = 1;
			ep_cfg_in[i].caps.mps = cfg->ep_mps;
		}

		ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	data->caps.rwup = true;
	data->caps.out_ack = false;
	data->caps.addr_before_status = true;
	data->caps.mps0 = UDC_MPS0_64;
	data->caps.hs = true;
	if (cfg->selected_speed == USB_FULL_SPEED) {
		data->caps.hs = false;
	}

	priv->dev = dev;
	priv->clk_enable = priv_clock_enable;
	priv->clk_disable = priv_clock_disable;

	k_msgq_init(&priv->msgq_data, udc_msgq_buf_0, sizeof(struct udc_at32_msg),
		    CONFIG_UDC_AT32_MAX_QMESSAGES);

	k_thread_create(&priv->thread_data, udc_at32_stack_0,
			K_THREAD_STACK_SIZEOF(udc_at32_stack_0), udc_at32_thread_handler,
			(void *)dev, NULL, NULL, K_PRIO_COOP(CONFIG_UDC_AT32_THREAD_PRIORITY),
			K_ESSENTIAL, K_NO_WAIT);
	k_thread_name_set(&priv->thread_data, dev->name);

	IRQ_CONNECT(UDC_AT32_IRQ, UDC_AT32_IRQ_PRI, udc_at32_irq,
		    DEVICE_DT_INST_GET(0), 0);

	return 0;
}

DEVICE_DT_INST_DEFINE(0, udc_at32_driver_init0, NULL, &udc0_data, &udc0_cfg,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &udc_at32_api);
