/*
 * Copyright 2026 Nordic Semiconductor ASA
 * Copyright 2026 NXP (rw6xx sd_dev driver, zephyrproject-rtos/zephyr#111009)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NXP RW6xx SDIO device controller exposed through the role-neutral SDIO
 * device-controller class (@ref sdio_dc_interface). Derived from the sd_dev
 * RW6xx driver in zephyrproject-rtos/zephyr#111009, re-homed onto the
 * role-neutral subsystem so the RW6xx SDU HAL backs a single controller class
 * shared with the emulated controllers.
 *
 * Mapping (RW6xx SDU HAL -> role-neutral sdio_dc):
 *  - RX (host -> device): the SDU write callback delivers a whole frame, which
 *    the adapter forwards as a fixed-address (FIFO) write access into the
 *    subsystem xfer callback; the subsystem routes it to the addressed function
 *    (and, for a streaming function, into its RX FIFO).
 *  - TX (device -> host): SDU is a push model (SDU_Send). The current sdio_dc
 *    contract is pull based (host reads, served by the function FIFO handler)
 *    plus raise_interrupt(); a dedicated frame-send op on the controller class
 *    is the natural home for SDU_Send() and is the one item left to align with
 *    the streaming TX path.
 */

#define DT_DRV_COMPAT nxp_rw6xx_sdio_device

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/sdio_dc.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/logging/log.h>

#include "fsl_adapter_sdu.h"

LOG_MODULE_REGISTER(sdio_dc_rw6xx, CONFIG_SDIO_LOG_LEVEL);

struct rw6xx_sdio_dc_config {
	uint8_t num_funcs;
	uint16_t max_blk_size;
	uint32_t cccr_addr;
	uint32_t data_port_reg;
	void (*irq_config)(void);
};

struct rw6xx_sdio_dc_data {
	sdio_dc_xfer_cb_t xfer_cb;
	void *xfer_user;
	bool enabled;
};

/* The SDU HAL is a singleton, matching the upstream RW6xx driver. */
static const struct device *rw6xx_dc_dev;

/*
 * SDU delivered a frame written by the host. Present it to the subsystem as a
 * fixed-address (FIFO) write access so it is routed to the addressed function.
 */
static void rw6xx_sdio_dc_rx(void *tlv, size_t tlv_sz)
{
	const struct device *dev = rw6xx_dc_dev;
	struct rw6xx_sdio_dc_data *dd = dev->data;
	const struct rw6xx_sdio_dc_config *cfg = dev->config;
	struct sdio_dc_xfer xfer = {
		.func = SDIO_FUNC_NUM_1,
		.dir = SDIO_DC_DIR_WRITE,
		.reg = cfg->data_port_reg,
		.increment = false,
		.data = tlv,
		.len = (uint32_t)tlv_sz,
	};

	if (!dd->enabled || dd->xfer_cb == NULL) {
		return;
	}
	(void)dd->xfer_cb(dev, &xfer, dd->xfer_user);
}

static void rw6xx_sdio_dc_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	SDU_DriverIRQHandler();
}

static int rw6xx_sdio_dc_enable(const struct device *dev)
{
	struct rw6xx_sdio_dc_data *dd = dev->data;
	const struct rw6xx_sdio_dc_config *cfg = dev->config;

	if (SDU_InitPhase1() != kStatus_Success ||
	    SDU_InitPhase2() != kStatus_Success ||
	    SDU_InitPhase3() != kStatus_Success) {
		return -EIO;
	}
	SDU_GetDefaultCISTable(cfg->cccr_addr);
	if (SDU_InstallCallback(SDU_TYPE_FOR_WRITE_CMD, rw6xx_sdio_dc_rx) !=
		    kStatus_Success ||
	    SDU_InstallCallback(SDU_TYPE_FOR_WRITE_DATA, rw6xx_sdio_dc_rx) !=
		    kStatus_Success) {
		return -EIO;
	}
	dd->enabled = true;
	return (SDU_SetFwReady() == kStatus_Success) ? 0 : -EIO;
}

static int rw6xx_sdio_dc_disable(const struct device *dev)
{
	struct rw6xx_sdio_dc_data *dd = dev->data;

	dd->enabled = false;
	SDU_EnterSuspend();
	return 0;
}

static int rw6xx_sdio_dc_set_xfer_callback(const struct device *dev,
					   sdio_dc_xfer_cb_t cb, void *user)
{
	struct rw6xx_sdio_dc_data *dd = dev->data;

	dd->xfer_cb = cb;
	dd->xfer_user = user;
	return 0;
}

static int rw6xx_sdio_dc_raise_interrupt(const struct device *dev,
					 enum sdio_func_num func)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(func);
	/*
	 * SDU asserts the host interrupt as part of SDU_Send(); a standalone
	 * interrupt assert is not used. Device->host frames are delivered by a
	 * controller frame-send op (see file header); until that op exists on
	 * the class this remains a no-op.
	 */
	return -ENOSYS;
}

static int rw6xx_sdio_dc_get_caps(const struct device *dev,
				  struct sdio_dc_caps *caps)
{
	const struct rw6xx_sdio_dc_config *cfg = dev->config;

	caps->num_funcs = cfg->num_funcs;
	caps->max_blk_size = cfg->max_blk_size;
	caps->interrupt_supported = true;
	return 0;
}

static DEVICE_API(sdio_dc, rw6xx_sdio_dc_api) = {
	.enable = rw6xx_sdio_dc_enable,
	.disable = rw6xx_sdio_dc_disable,
	.set_xfer_callback = rw6xx_sdio_dc_set_xfer_callback,
	.raise_interrupt = rw6xx_sdio_dc_raise_interrupt,
	.get_caps = rw6xx_sdio_dc_get_caps,
};

static int rw6xx_sdio_dc_init(const struct device *dev)
{
	const struct rw6xx_sdio_dc_config *cfg = dev->config;

	rw6xx_dc_dev = dev;
	if (cfg->irq_config) {
		cfg->irq_config();
	}
	return 0;
}

#define RW6XX_SDIO_DC_INIT(inst)						\
	static void rw6xx_sdio_dc_irq_cfg_##inst(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),	\
			    rw6xx_sdio_dc_isr, DEVICE_DT_INST_GET(inst), 0);	\
		irq_enable(DT_INST_IRQN(inst));					\
	}									\
	static const struct rw6xx_sdio_dc_config rw6xx_sdio_dc_cfg_##inst = {	\
		.num_funcs = DT_INST_PROP_OR(inst, num_functions, 1),		\
		.max_blk_size = DT_INST_PROP_OR(inst, max_block_size, 512),	\
		.cccr_addr = DT_INST_REG_ADDR(inst),				\
		.data_port_reg = DT_INST_PROP_OR(inst, data_port_reg, 0),	\
		.irq_config = rw6xx_sdio_dc_irq_cfg_##inst,			\
	};									\
	static struct rw6xx_sdio_dc_data rw6xx_sdio_dc_data_##inst;		\
	DEVICE_DT_INST_DEFINE(inst, rw6xx_sdio_dc_init, NULL,			\
			      &rw6xx_sdio_dc_data_##inst,			\
			      &rw6xx_sdio_dc_cfg_##inst, POST_KERNEL,		\
			      CONFIG_SDIO_DEVICE_INIT_PRIORITY,			\
			      &rw6xx_sdio_dc_api);

DT_INST_FOREACH_STATUS_OKAY(RW6XX_SDIO_DC_INIT)
