/*
 * Copyright (c) 2018 Alexander Wachter
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include soc.h prior to Zephyr CAN headers to pull in HAL fixups */
#include <soc.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/at32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <at32_can.h>

LOG_MODULE_REGISTER(can_at32, CONFIG_CAN_LOG_LEVEL);

#define CAN_INIT_TIMEOUT  (10 * (sys_clock_hw_cycles_per_sec() / MSEC_PER_SEC))

#define DT_DRV_COMPAT at_at32_bxcan

#define CAN_AT32_NUM_FILTER_BANKS (14)

#define CAN_AT32_MAX_FILTER_ID    CONFIG_CAN_MAX_STD_ID_FILTER /* (CONFIG_CAN_MAX_STD_ID_FILTER * 2) */

/*
#define CAN_AT32_MAX_FILTER_ID \
	(CONFIG_CAN_MAX_EXT_ID_FILTER + CONFIG_CAN_MAX_STD_ID_FILTER * 2)
*/

#if 0
#define CAN_AT32_FIRX_STD_IDE_POS   (3U)
#define CAN_AT32_FIRX_STD_RTR_POS   (4U)
#define CAN_AT32_FIRX_STD_ID_POS    (5U)
#endif

#define CAN_AT32_FIRX_STD_IDE_POS   (2U)
#define CAN_AT32_FIRX_STD_RTR_POS   (1U)
#define CAN_AT32_FIRX_STD_ID_POS    (21U)

#define CAN_AT32_FIRX_EXT_IDE_POS    (2U)
#define CAN_AT32_FIRX_EXT_RTR_POS    (1U)
#define CAN_AT32_FIRX_EXT_STD_ID_POS (21U)
#define CAN_AT32_FIRX_EXT_EXT_ID_POS (3U)

struct can_at32_mailbox {
	can_tx_callback_t tx_callback;
	void *callback_arg;
};

struct can_at32_data {
	struct can_driver_data common;
	struct k_mutex inst_mutex;
	struct k_sem tx_int_sem;
	struct can_at32_mailbox mb0;
	struct can_at32_mailbox mb1;
	struct can_at32_mailbox mb2;
#if 0
	can_rx_callback_t rx_cb_std[CONFIG_CAN_MAX_STD_ID_FILTER];
	can_rx_callback_t rx_cb_ext[CONFIG_CAN_MAX_EXT_ID_FILTER];
	void *cb_arg_std[CONFIG_CAN_MAX_STD_ID_FILTER];
	void *cb_arg_ext[CONFIG_CAN_MAX_EXT_ID_FILTER];
#endif
    can_rx_callback_t rx_cb[CAN_AT32_MAX_FILTER_ID];
	void *cb_arg[CAN_AT32_MAX_FILTER_ID];
	enum can_state state;
};

struct can_at32_config {
	const struct can_driver_config common;
	can_type *can;   /*!< CAN Registers*/
	can_type *master_can;   /*!< CAN Registers for shared filter */
	uint32_t clkid;
	void (*config_irq)(can_type *can);
	const struct pinctrl_dev_config *pcfg;
};

/*
 * Mutex to prevent simultaneous access to filter registers shared between CAN1
 * and CAN2.
 */
static struct k_mutex filter_mutex;

static void can_at32_signal_tx_complete(const struct device *dev, struct can_at32_mailbox *mb,
					 int status)
{
	can_tx_callback_t callback = mb->tx_callback;

	if (callback != NULL) {
		callback(dev, status, mb->callback_arg);
		mb->tx_callback = NULL;
	}
}

static void can_at32_rx_fifo_pop(can_fifo_mailbox_type *mbox, struct can_frame *frame)
{
	memset(frame, 0, sizeof(*frame));

	if (mbox->rfi_bit.rfidi) {
		frame->id = 0x1FFFFFFF & (mbox->rfi >> 3);
		frame->flags |= CAN_FRAME_IDE;
	} else {
		frame->id = mbox->rfi_bit.rfsid;
	}

	if (mbox->rfi_bit.rffri) {
		frame->flags |= CAN_FRAME_RTR;
	} else {
		frame->data_32[0] = mbox->rfdtl;
		frame->data_32[1] = mbox->rfdth;
	}

	frame->dlc = mbox->rfc_bit.rfdtl;
#ifdef CONFIG_CAN_RX_TIMESTAMP
	frame->timestamp = mbox->rfc_bit.rfts;
#endif
}

static inline void can_at32_rx_isr_handler(const struct device *dev)
{
	struct can_at32_data *data = dev->data;
	const struct can_at32_config *cfg = dev->config;
	can_type *can = cfg->can;
	can_fifo_mailbox_type *mbox;
	int filter_id;
	struct can_frame frame;
	can_rx_callback_t callback = NULL;
	void *cb_arg;

	while (can->rf0_bit.rf0mn) {
		mbox = &can->fifo_mailbox[0];
		filter_id = mbox->rfc_bit.rffmn;

		LOG_DBG("Message on filter_id %d", filter_id);

		can_at32_rx_fifo_pop(mbox, &frame);

		if (filter_id < CAN_AT32_MAX_FILTER_ID) {
			callback = data->rx_cb[filter_id];
			cb_arg = data->cb_arg[filter_id];
		}

		if (callback) {
			callback(dev, &frame, cb_arg);
		}

		/* Release message */
		can->rf0 = CAN_RF0_RF0R_VAL;
	}

	if (can->rf0_bit.rf0of) {
		LOG_ERR("RX FIFO Overflow");
		CAN_STATS_RX_OVERRUN_INC(dev);
	}
}

static int can_at32_get_state(const struct device *dev, enum can_state *state,
			       struct can_bus_err_cnt *err_cnt)
{
	const struct can_at32_config *cfg = dev->config;
	struct can_at32_data *data = dev->data;
	can_type *can = cfg->can;

	if (state != NULL) {
		if (!data->common.started) {
			*state = CAN_STATE_STOPPED;
		} else if (can->ests_bit.bof) {
			*state = CAN_STATE_BUS_OFF;
		} else if (can->ests_bit.epf) {
			*state = CAN_STATE_ERROR_PASSIVE;
		} else if (can->ests_bit.eaf) {
			*state = CAN_STATE_ERROR_WARNING;
		} else {
			*state = CAN_STATE_ERROR_ACTIVE;
		}
	}

	if (err_cnt != NULL) {
		err_cnt->tx_err_cnt = can->ests_bit.tec;
		err_cnt->rx_err_cnt = can->ests_bit.rec;
	}

	return 0;
}

static inline void can_at32_bus_state_change_isr(const struct device *dev)
{
	struct can_at32_data *data = dev->data;
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	const can_state_change_callback_t cb = data->common.state_change_cb;
	void *state_change_cb_data = data->common.state_change_cb_user_data;

#ifdef CONFIG_CAN_STATS
	const struct can_at32_config *cfg = dev->config;
	can_type *can = cfg->can;

	switch (can->ests_bit.etr) {
	case (CAN_ERRORRECORD_STUFFERR):
		CAN_STATS_STUFF_ERROR_INC(dev);
		break;
	case (CAN_ERRORRECORD_FORMERR):
		CAN_STATS_FORM_ERROR_INC(dev);
		break;
	case (CAN_ERRORRECORD_ACKERR):
		CAN_STATS_ACK_ERROR_INC(dev);
		break;
	case (CAN_ERRORRECORD_BITRECESSIVEERR):
		CAN_STATS_BIT1_ERROR_INC(dev);
		break;
	case (CAN_ERRORRECORD_BITDOMINANTERR):
		CAN_STATS_BIT0_ERROR_INC(dev);
		break;
	case (CAN_ERRORRECORD_CRCERR):
		CAN_STATS_CRC_ERROR_INC(dev);
		break;
	default:
		break;
	}

	/* Clear last error code flag */
	can->ests_bit.etr = 0;
#endif /* CONFIG_CAN_STATS */

	(void)can_at32_get_state(dev, &state, &err_cnt);

	if (state != data->state) {
		data->state = state;

		if (cb != NULL) {
			cb(dev, state, err_cnt, state_change_cb_data);
		}
	}
}

static inline void can_at32_tx_isr_handler(const struct device *dev)
{
	struct can_at32_data *data = dev->data;
	const struct can_at32_config *cfg = dev->config;
	can_type *can = cfg->can;
	uint32_t bus_off;
	int status;

	bus_off = can->ests_bit.bof;

	if (can->tsts_bit.tm0tcf | bus_off) {
		status = can->tsts_bit.tm0tsf ? 0  :
			 can->tsts_bit.tm0tef ? -EIO :
			 can->tsts_bit.tm0alf ? -EBUSY :
					  bus_off ? -ENETUNREACH :
						    -EIO;
		/* clear the request. */
		can->tsts_bit.tm0tcf = 1;
		can_at32_signal_tx_complete(dev, &data->mb0, status);
	}

	if (can->tsts_bit.tm1tcf | bus_off) {
		status = can->tsts_bit.tm1tsf ? 0  :
			 can->tsts_bit.tm1tef ? -EIO :
			 can->tsts_bit.tm1alf ? -EBUSY :
					  bus_off ? -ENETUNREACH :
						    -EIO;
		/* clear the request. */
		can->tsts_bit.tm1tcf = 1;
		can_at32_signal_tx_complete(dev, &data->mb1, status);
	}

	if (can->tsts_bit.tm2tcf | bus_off) {
		status = can->tsts_bit.tm2tsf ? 0  :
			 can->tsts_bit.tm2tef ? -EIO :
			 can->tsts_bit.tm2alf ? -EBUSY :
					  bus_off ? -ENETUNREACH :
						    -EIO;
		/* clear the request. */
		can->tsts_bit.tm2tcf = 1;
		can_at32_signal_tx_complete(dev, &data->mb2, status);
	}

	if (can->tsts_bit.tm0ef || can->tsts_bit.tm1ef || can->tsts_bit.tm2ef) {
		k_sem_give(&data->tx_int_sem);
	}
}

static void can_at32_rx_isr(const struct device *dev)
{
	can_at32_rx_isr_handler(dev);
}

static void can_at32_tx_isr(const struct device *dev)
{
	can_at32_tx_isr_handler(dev);
}

static void can_at32_state_change_isr(const struct device *dev)
{
	const struct can_at32_config *cfg = dev->config;
	can_type *can = cfg->can;

	/* Signal bus-off to waiting tx */
	if (can->msts_bit.eoif) {
		can_at32_tx_isr_handler(dev);
		can_at32_bus_state_change_isr(dev);
		can->msts_bit.eoif = TRUE;
	}
}


static int can_at32_enter_init_mode(can_type *can)
{
	uint32_t start_time;

	can->mctrl_bit.fzen = TRUE;
	start_time = k_cycle_get_32();

	while (can->msts_bit.fzc == 0U) {
		if (k_cycle_get_32() - start_time > CAN_INIT_TIMEOUT) {
			can->mctrl_bit.fzen = FALSE;
			return -EAGAIN;
		}
	}

	return 0;
}

static int can_at32_leave_init_mode(can_type *can)
{
	uint32_t start_time;

	can->mctrl_bit.fzen = FALSE;
	start_time = k_cycle_get_32();

	while (can->msts_bit.fzc != 0U) {
		if (k_cycle_get_32() - start_time > CAN_INIT_TIMEOUT) {
			return -EAGAIN;
		}
	}

	return 0;
}

static int can_at32_leave_sleep_mode(can_type *can)
{
	uint32_t start_time;

	can->mctrl_bit.dzen = FALSE;
	start_time = k_cycle_get_32();

	while (can->msts_bit.dzc != 0U) {
		if (k_cycle_get_32() - start_time > CAN_INIT_TIMEOUT) {
			return -EAGAIN;
		}
	}

	return 0;
}

static int can_at32_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_ONE_SHOT;

	if (IS_ENABLED(CONFIG_CAN_MANUAL_RECOVERY_MODE)) {
		*cap |= CAN_MODE_MANUAL_RECOVERY;
	}

	return 0;
}

static int can_at32_start(const struct device *dev)
{
	const struct can_at32_config *cfg = dev->config;
	struct can_at32_data *data = dev->data;
	can_type *can = cfg->can;
	int ret = 0;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (data->common.started) {
		ret = -EALREADY;
		goto unlock;
	}

	if (cfg->common.phy != NULL) {
		ret = can_transceiver_enable(cfg->common.phy, data->common.mode);
		if (ret != 0) {
			LOG_ERR("failed to enable CAN transceiver (err %d)", ret);
			goto unlock;
		}
	}

	CAN_STATS_RESET(dev);

	ret = can_at32_leave_init_mode(can);
	if (ret < 0) {
		LOG_ERR("Failed to leave init mode");

		if (cfg->common.phy != NULL) {
			/* Attempt to disable the CAN transceiver in case of error */
			(void)can_transceiver_disable(cfg->common.phy);
		}

		ret = -EIO;
		goto unlock;
	}

	data->common.started = true;

unlock:
	k_mutex_unlock(&data->inst_mutex);

	return ret;
}

static int can_at32_stop(const struct device *dev)
{
	const struct can_at32_config *cfg = dev->config;
	struct can_at32_data *data = dev->data;
	can_type *can = cfg->can;
	int ret = 0;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (!data->common.started) {
		ret = -EALREADY;
		goto unlock;
	}

	ret = can_at32_enter_init_mode(can);
	if (ret < 0) {
		LOG_ERR("Failed to enter init mode");
		ret = -EIO;
		goto unlock;
	}

	/* Abort any pending transmissions */
	can_at32_signal_tx_complete(dev, &data->mb0, -ENETDOWN);
	can_at32_signal_tx_complete(dev, &data->mb1, -ENETDOWN);
	can_at32_signal_tx_complete(dev, &data->mb2, -ENETDOWN);
	can->tsts = CAN_TSTS_TM0CT_VAL | CAN_TSTS_TM1CT_VAL | CAN_TSTS_TM2CT_VAL;


	if (cfg->common.phy != NULL) {
		ret = can_transceiver_disable(cfg->common.phy);
		if (ret != 0) {
			LOG_ERR("failed to enable CAN transceiver (err %d)", ret);
			goto unlock;
		}
	}

	data->common.started = false;

unlock:
	k_mutex_unlock(&data->inst_mutex);

	return ret;
}

static int can_at32_set_mode(const struct device *dev, can_mode_t mode)
{
	can_mode_t supported = CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_ONE_SHOT;
	const struct can_at32_config *cfg = dev->config;
	can_type *can = cfg->can;
	struct can_at32_data *data = dev->data;

	LOG_DBG("Set mode %d", mode);

	if (IS_ENABLED(CONFIG_CAN_MANUAL_RECOVERY_MODE)) {
		supported |= CAN_MODE_MANUAL_RECOVERY;
	}

	if ((mode & ~(supported)) != 0) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	if (data->common.started) {
		return -EBUSY;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if ((mode & CAN_MODE_LOOPBACK) != 0) {
		/* Loopback mode */
		can->btmg_bit.lben = TRUE;

	} else {
		can->btmg_bit.lben = FALSE;
	}

	if ((mode & CAN_MODE_LISTENONLY) != 0) {
		/* Silent mode */
		can->btmg_bit.loen = TRUE;
	} else {
		can->btmg_bit.loen = FALSE;
	}

	if ((mode & CAN_MODE_ONE_SHOT) != 0) {
		/* No automatic retransmission */
		can->mctrl_bit.prsfen = TRUE;

	} else {
		can->mctrl_bit.prsfen = FALSE;
	}

	if (IS_ENABLED(CONFIG_CAN_MANUAL_RECOVERY_MODE)) {
		if ((mode & CAN_MODE_MANUAL_RECOVERY) != 0) {
			/* No automatic recovery from bus-off */
			can->mctrl_bit.aeboen = FALSE; 
		} else {
			can->mctrl_bit.aeboen = TRUE; 
		}
	}

	data->common.mode = mode;

	k_mutex_unlock(&data->inst_mutex);

	return 0;
}

static int can_at32_set_timing(const struct device *dev,
				const struct can_timing *timing)
{
	const struct can_at32_config *cfg = dev->config;
	can_type *can = cfg->can;
	struct can_at32_data *data = dev->data;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (data->common.started) {
		k_mutex_unlock(&data->inst_mutex);
		return -EBUSY;
	}

	can->btmg_bit.brdiv = timing->prescaler - 1;
	can->btmg_bit.rsaw = timing->sjw - 1;
	can->btmg_bit.bts1 = timing->phase_seg1 - 1;
	can->btmg_bit.bts2 = timing->phase_seg2 - 1;

	k_mutex_unlock(&data->inst_mutex);

	return 0;
}

static int can_at32_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_at32_config *cfg = dev->config;
	int ret;

	ret = clock_control_get_rate(AT32_CLOCK_CONTROLLER,
				     (clock_control_subsys_t) &cfg->clkid,
				     rate);
	if (ret != 0) {
		LOG_ERR("Failed call clock_control_get_rate: return [%d]", ret);
		return -EIO;
	}

	return 0;
}

static int can_at32_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(dev);

	if (ide) {
		return CONFIG_CAN_MAX_EXT_ID_FILTER;
	} else {
		return CONFIG_CAN_MAX_STD_ID_FILTER;
	}
}

static int can_at32_init(const struct device *dev)
{
	const struct can_at32_config *cfg = dev->config;
	struct can_at32_data *data = dev->data;
	can_type *can = cfg->can;
	struct can_timing timing = { 0 };
	int ret;

	k_mutex_init(&filter_mutex);
	k_mutex_init(&data->inst_mutex);
	k_sem_init(&data->tx_int_sem, 0, 1);

	if (cfg->common.phy != NULL) {
		if (!device_is_ready(cfg->common.phy)) {
			LOG_ERR("CAN transceiver not ready");
			return -ENODEV;
		}
	}

	ret = clock_control_on(AT32_CLOCK_CONTROLLER, (clock_control_subsys_t) &cfg->clkid);
	if (ret != 0) {
		LOG_ERR("HAL_CAN_Init clock control on failed: %d", ret);
		return -EIO;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("CAN pinctrl setup failed (%d)", ret);
		return ret;
	}

	ret = can_at32_enter_init_mode(can);
	if (ret) {
		LOG_ERR("Failed to enter init mode");
		return ret;
	}

	ret = can_at32_leave_sleep_mode(can);
	if (ret) {
		LOG_ERR("Failed to exit sleep mode");
		return ret;
	}

	/* configure scale of filter banks < CONFIG_CAN_MAX_EXT_ID_FILTER for ext ids */
	cfg->master_can->fctrl_bit.fcs = TRUE;
	cfg->master_can->fbwcfg |= (1U << CONFIG_CAN_MAX_EXT_ID_FILTER) - 1;
	cfg->master_can->fctrl_bit.fcs = FALSE;

	can->mctrl &= ~(0xFC);
#ifdef CONFIG_CAN_RX_TIMESTAMP
	can_x->mctrl_bit.ttcen = TRUE;
#endif

	/* Enable automatic bus-off recovery */
	can->mctrl_bit.aeboen = TRUE;

	ret = can_calc_timing(dev, &timing, cfg->common.bitrate,
			      cfg->common.sample_point);
	if (ret == -EINVAL) {
		LOG_ERR("Can't find timing for given param");
		return -EIO;
	}
	LOG_DBG("Presc: %d, TS1: %d, TS2: %d",
		timing.prescaler, timing.phase_seg1, timing.phase_seg2);
	LOG_DBG("Sample-point err : %d", ret);

	ret = can_set_timing(dev, &timing);
	if (ret) {
		return ret;
	}

	ret = can_at32_set_mode(dev, CAN_MODE_NORMAL);
	if (ret) {
		return ret;
	}

	(void)can_at32_get_state(dev, &data->state, NULL);

	cfg->config_irq(can);
    can->inten_bit.tcien = TRUE;
	return 0;
}

static void can_at32_set_state_change_callback(const struct device *dev,
						can_state_change_callback_t cb,
						void *user_data)
{
	struct can_at32_data *data = dev->data;
	const struct can_at32_config *cfg = dev->config;
	can_type *can = cfg->can;

	data->common.state_change_cb = cb;
	data->common.state_change_cb_user_data = user_data;

	if (cb == NULL) {
		can->inten &= ~(CAN_BOIEN_INT | CAN_EPIEN_INT | CAN_EAIEN_INT);
	} else {
		can->inten |= CAN_BOIEN_INT | CAN_EPIEN_INT | CAN_EAIEN_INT;
	}
}

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
static int can_at32_recover(const struct device *dev, k_timeout_t timeout)
{
	const struct can_at32_config *cfg = dev->config;
	struct can_at32_data *data = dev->data;
	can_type *can = cfg->can;
	int ret = -EAGAIN;
	int64_t start_time;

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if ((data->common.mode & CAN_MODE_MANUAL_RECOVERY) == 0U) {
		return -ENOTSUP;
	}

	if (!(can->ests & CAN_BOF_FLAG)) {
		return 0;
	}

	if (k_mutex_lock(&data->inst_mutex, K_FOREVER)) {
		return -EAGAIN;
	}

	ret = can_at32_enter_init_mode(can);
	if (ret) {
		goto done;
	}

	can_at32_leave_init_mode(can);

	start_time = k_uptime_ticks();

	while (can->ests & CAN_BOF_FLAG) {
		if (!K_TIMEOUT_EQ(timeout, K_FOREVER) &&
		    k_uptime_ticks() - start_time >= timeout.ticks) {
			goto done;
		}
	}

	ret = 0;

done:
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */

static int can_at32_send(const struct device *dev, const struct can_frame *frame,
			  k_timeout_t timeout, can_tx_callback_t callback,
			  void *user_data)
{
	const struct can_at32_config *cfg = dev->config;
	struct can_at32_data *data = dev->data;
	can_type *can = cfg->can;
	uint32_t transmit_status_register = 0;
	can_tx_mailbox_type *mailbox = NULL;
	struct can_at32_mailbox *mb = NULL;

	LOG_DBG("Sending %d bytes on %s. "
		    "Id: 0x%x, "
		    "ID type: %s, "
		    "Remote Frame: %s"
		    , frame->dlc, dev->name
		    , frame->id
		    , (frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard"
		    , (frame->flags & CAN_FRAME_RTR) != 0 ? "yes" : "no");

	if (frame->dlc > CAN_MAX_DLC) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
		return -EINVAL;
	}

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if (can->ests & CAN_BOF_FLAG) {
		return -ENETUNREACH;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);
	transmit_status_register = can->tsts;
	while (!(transmit_status_register & 0x1C000000)) {
		k_mutex_unlock(&data->inst_mutex);
		LOG_DBG("Transmit buffer full");
		if (k_sem_take(&data->tx_int_sem, timeout)) {
			return -EAGAIN;
		}

		k_mutex_lock(&data->inst_mutex, K_FOREVER);
		transmit_status_register = can->tsts;
	}

	if (can->tsts_bit.tm0ef) {
		LOG_DBG("Using TX mailbox 0");
		mailbox = &can->tx_mailbox[0];
		mb = &(data->mb0);
	} else if (can->tsts_bit.tm1ef) {
		LOG_DBG("Using TX mailbox 1");
		mailbox = &can->tx_mailbox[1];
		mb = &data->mb1;
	} else if (can->tsts_bit.tm2ef) {
		LOG_DBG("Using TX mailbox 2");
		mailbox = &can->tx_mailbox[2];
		mb = &data->mb2;
	}

	mb->tx_callback = callback;
	mb->callback_arg = user_data;

	/* mailbox identifier register setup */
	mailbox->tmi &= 0x00000001;

	if ((frame->flags & CAN_FRAME_IDE) != 0) {
		mailbox->tmi |= (frame->id << 3);
		mailbox->tmi_bit.tmidsel = TRUE;
	} else {
		mailbox->tmi_bit.tmsid = frame->id;
	}

	if ((frame->flags & CAN_FRAME_RTR) != 0) {
		mailbox->tmi_bit.tmfrsel = TRUE;
	} else {
		mailbox->tmdtl = frame->data_32[0];
		mailbox->tmdth = frame->data_32[1];
	}

    mailbox->tmc_bit.tmdtbl = frame->dlc & 0xF;

	mailbox->tmi_bit.tmsr = TRUE;
	k_mutex_unlock(&data->inst_mutex);

	return 0;
}

static void can_at32_set_filter_bank(int filter_id, can_filter_register_type *filter_reg,
				      bool ide, uint32_t id, uint32_t mask)
{
	
	filter_reg->ffdb1 = id;
	filter_reg->ffdb2 = mask;
}

static inline uint32_t can_at32_filter_to_std_mask(const struct can_filter *filter)
{
	uint32_t rtr_mask = !IS_ENABLED(CONFIG_CAN_ACCEPT_RTR);

	return  (filter->mask << CAN_AT32_FIRX_STD_ID_POS) |
		(rtr_mask << CAN_AT32_FIRX_STD_RTR_POS) |
		(1U << CAN_AT32_FIRX_STD_IDE_POS);
}

static inline uint32_t can_at32_filter_to_ext_mask(const struct can_filter *filter)
{
	uint32_t rtr_mask = !IS_ENABLED(CONFIG_CAN_ACCEPT_RTR);

	return  (filter->mask << CAN_AT32_FIRX_EXT_EXT_ID_POS) |
		(rtr_mask << CAN_AT32_FIRX_EXT_RTR_POS) |
		(1U << CAN_AT32_FIRX_EXT_IDE_POS);
}

static inline uint32_t can_at32_filter_to_std_id(const struct can_filter *filter)
{
	return  (filter->id  << CAN_AT32_FIRX_STD_ID_POS);
}

static inline uint32_t can_at32_filter_to_ext_id(const struct can_filter *filter)
{
	return  (filter->id << CAN_AT32_FIRX_EXT_EXT_ID_POS) |
		(1U << CAN_AT32_FIRX_EXT_IDE_POS);
}

static inline int can_at32_set_filter(const struct device *dev, const struct can_filter *filter)
{
	const struct can_at32_config *cfg = dev->config;
	struct can_at32_data *data = dev->data;
	can_type *can = cfg->master_can;
	uint32_t mask = 0U;
	uint32_t id = 0U;
	int filter_id = -ENOSPC;

    if ((filter->flags & CAN_FILTER_IDE) != 0) {
	    for (int i = 0; i < CAN_AT32_MAX_FILTER_ID; i++) {
		    if (data->rx_cb[i] == NULL) {
			    id = can_at32_filter_to_ext_id(filter);
			    mask = can_at32_filter_to_ext_mask(filter);
			    filter_id = i;
			    break;
		    }
	    }
    } else {
	    for (int i = 0; i < CAN_AT32_MAX_FILTER_ID; i++) {
		    if (data->rx_cb[i] == NULL) {
			    id = can_at32_filter_to_std_id(filter);
			    mask = can_at32_filter_to_std_mask(filter);
			    filter_id =  i;
			    break;
		    }
	    }
    }
	if (filter_id != -ENOSPC) {
		LOG_DBG("Adding filter_id %d, CAN ID: 0x%x, mask: 0x%x",
			filter_id, filter->id, filter->mask);

		/* set the filter init mode */
		can->fctrl_bit.fcs = TRUE;

		can_at32_set_filter_bank(filter_id, &can->ffb[filter_id],
					  (filter->flags & CAN_FILTER_IDE) != 0,
					  id, mask);

		can->facfg |= 1U << filter_id;
		can->fctrl_bit.fcs = FALSE;
	} else {
		LOG_WRN("No free filter left");
	}

	return filter_id;
}


/*
 * This driver uses masked mode for all filters (CAN_FM1R left at reset value
 * 0x00) in order to simplify mapping between filter match index from the FIFOs
 * and array index for the callbacks. All ext ID filters are stored in the
 * banks below CONFIG_CAN_MAX_EXT_ID_FILTER, followed by the std ID filters,
 * which consume only 1/2 bank per filter.
 *
 * The more complicated list mode must be implemented if someone requires more
 * than 28 std ID or 14 ext ID filters.
 *
 * Currently, all filter banks are assigned to FIFO 0 and FIFO 1 is not used.
 */
static int can_at32_add_rx_filter(const struct device *dev, can_rx_callback_t cb,
				   void *cb_arg, const struct can_filter *filter)
{
	struct can_at32_data *data = dev->data;
	int filter_id;

	if ((filter->flags & ~(CAN_FILTER_IDE)) != 0) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&filter_mutex, K_FOREVER);
	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	filter_id = can_at32_set_filter(dev, filter);
	if (filter_id >= 0) {
	    data->rx_cb[filter_id] = cb;
		data->cb_arg[filter_id] = cb_arg;
	}
	k_mutex_unlock(&data->inst_mutex);
	k_mutex_unlock(&filter_mutex);

	return filter_id;
}

static void can_at32_remove_rx_filter(const struct device *dev, int filter_id)
{
	const struct can_at32_config *cfg = dev->config;
	struct can_at32_data *data = dev->data;
	can_type *can = cfg->master_can;
	bool ide;
	int bank_offset = 0;
	int bank_num;
	bool bank_unused;

	if (filter_id < 0 || filter_id >= CAN_AT32_MAX_FILTER_ID) {
		LOG_ERR("filter ID %d out of bounds", filter_id);
		return;
	}

	k_mutex_lock(&filter_mutex, K_FOREVER);
	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (cfg->can != cfg->master_can) {
		bank_offset = CAN_AT32_NUM_FILTER_BANKS;
	}

	if (filter_id < CAN_AT32_MAX_FILTER_ID) {
		ide = true;
		bank_num = bank_offset + filter_id;

		data->rx_cb[filter_id] = NULL;
		data->cb_arg[filter_id] = NULL;

		bank_unused = true;
		LOG_DBG("Removing filter_id %d, ide %d", filter_id, ide);

		can->fctrl_bit.fcs = TRUE;
	
		can_at32_set_filter_bank(filter_id, &can->ffb[filter_id],
					  ide, 0, 0xFFFFFFFF);
	
		if (bank_unused) {
			can->facfg &= ~(1U << filter_id);
			LOG_DBG("Filter bank %d is unused -> deactivate", filter_id);
		}
		can->fctrl_bit.fcs = FALSE;
	}
	k_mutex_unlock(&data->inst_mutex);
	k_mutex_unlock(&filter_mutex);
}

static DEVICE_API(can, can_api_funcs) = {
	.get_capabilities = can_at32_get_capabilities,
	.start = can_at32_start,
	.stop = can_at32_stop,
	.set_mode = can_at32_set_mode,
	.set_timing = can_at32_set_timing,
	.send = can_at32_send,
	.add_rx_filter = can_at32_add_rx_filter,
	.remove_rx_filter = can_at32_remove_rx_filter,
	.get_state = can_at32_get_state,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = can_at32_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
	.set_state_change_callback = can_at32_set_state_change_callback,
	.get_core_clock = can_at32_get_core_clock,
	.get_max_filters = can_at32_get_max_filters,
	.timing_min = {
		.sjw = 0x1,
		.prop_seg = 0x00,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x01,
		.prescaler = 0x01
	},
	.timing_max = {
		.sjw = 0x04,
		.prop_seg = 0x00,
		.phase_seg1 = 0x10,
		.phase_seg2 = 0x08,
		.prescaler = 0x400
	}
};

#define CAN_AT32_IRQ_INST(inst)                                     \
static void config_can_##inst##_irq(can_type *can)                \
{                                                                    \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rx0, irq),             \
		    DT_INST_IRQ_BY_NAME(inst, rx0, priority),        \
		    can_at32_rx_isr, DEVICE_DT_INST_GET(inst), 0);  \
	irq_enable(DT_INST_IRQ_BY_NAME(inst, rx0, irq));             \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, tx, irq),              \
		    DT_INST_IRQ_BY_NAME(inst, tx, priority),         \
		    can_at32_tx_isr, DEVICE_DT_INST_GET(inst), 0);  \
	irq_enable(DT_INST_IRQ_BY_NAME(inst, tx, irq));              \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, sce, irq),             \
		    DT_INST_IRQ_BY_NAME(inst, sce, priority),        \
		    can_at32_state_change_isr,                      \
		    DEVICE_DT_INST_GET(inst), 0);                    \
	irq_enable(DT_INST_IRQ_BY_NAME(inst, sce, irq));             \
	can->inten |= (CAN_TCIEN_INT | CAN_EOIEN_INT | CAN_RF0MIEN_INT | CAN_RF1MIEN_INT | CAN_BOIEN_INT); \
	if (IS_ENABLED(CONFIG_CAN_STATS)) {                          \
		can->inten |= CAN_ETRIEN_INT;                           \
	}                                                            \
}

#define CAN_AT32_CONFIG_INST(inst)                                      \
PINCTRL_DT_INST_DEFINE(inst);                                            \
static const struct can_at32_config can_at32_cfg_##inst = {            \
	.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 1000000),       \
	.can = (can_type *)DT_INST_REG_ADDR(inst),                    \
	.master_can = (can_type *)DT_INST_PROP_OR(inst,               \
		master_can_reg, DT_INST_REG_ADDR(inst)),                 \
	.clkid = DT_INST_CLOCKS_CELL(inst, id),			              \
	.config_irq = config_can_##inst##_irq,                           \
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),	                 \
};

#define CAN_AT32_DATA_INST(inst) \
static struct can_at32_data can_at32_dev_data_##inst;

#define CAN_AT32_DEFINE_INST(inst)                                      \
CAN_DEVICE_DT_INST_DEFINE(inst, can_at32_init, NULL,                    \
			  &can_at32_dev_data_##inst, &can_at32_cfg_##inst, \
			  POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,         \
			  &can_api_funcs);

#define CAN_AT32_INST(inst)      \
CAN_AT32_IRQ_INST(inst)          \
CAN_AT32_CONFIG_INST(inst)       \
CAN_AT32_DATA_INST(inst)         \
CAN_AT32_DEFINE_INST(inst)

DT_INST_FOREACH_STATUS_OKAY(CAN_AT32_INST)
