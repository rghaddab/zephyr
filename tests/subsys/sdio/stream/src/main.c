/*
 * Copyright 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Exercises the optional device-side streaming (packet/poll) extension over
 * the role-neutral SDIO device subsystem, driven end-to-end through the virtual
 * loopback controller. A host endpoint writes/reads a streaming function's data
 * port; the device side consumes/produces packets via the streaming API.
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sdio/sdio_core.h>
#include <zephyr/sdio/sdio_device.h>
#include <zephyr/sdio/sdio_stream.h>
#include <zephyr/drivers/sdio_dc.h>
#include <zephyr/drivers/sdio_dc_virtual.h>

#define DC_NODE DT_NODELABEL(sdio_dc0)
#define STREAM_FIFO_REG 0x00

/* Device/slave side */
static struct sdio_device dev_endpoint;
static struct sdio_stream_function stream_func;

/* Host/master side */
static struct sdio_dev host;
static struct sdio_function host_func1;

static volatile uint32_t irq_count;

static void host_irq_cb(const struct device *dc, enum sdio_func_num func,
			void *user)
{
	ARG_UNUSED(dc);
	ARG_UNUSED(func);
	ARG_UNUSED(user);
	irq_count++;
}

static void *stream_setup(void)
{
	const struct device *dc = DEVICE_DT_GET(DC_NODE);

	zassert_true(device_is_ready(dc), "virtual controller not ready");

	/* Device side: a streaming function on function 1 */
	zassert_ok(sdio_device_init(&dev_endpoint, dc));
	zassert_ok(sdio_stream_function_init(&stream_func, SDIO_FUNC_NUM_1,
					     STREAM_FIFO_REG));
	zassert_ok(sdio_device_register_function(&dev_endpoint,
						 &stream_func.base));
	zassert_ok(sdio_device_enable(&dev_endpoint));

	/* Host side over the loopback transport */
	zassert_ok(sdio_dev_init(&host, SDIO_ROLE_HOST,
				 sdio_dc_virtual_loopback_api(),
				 sdio_dc_virtual_loopback_ctx(dc), 0));
	host.max_blk_size = 512;
	zassert_ok(sdio_func_bind(&host, &host_func1, SDIO_FUNC_NUM_1));

	sdio_dc_virtual_set_irq_cb(dc, host_irq_cb, NULL);
	return NULL;
}

/* Host writes a frame -> device receives it via the streaming RX path */
ZTEST(sdio_stream, test_host_to_device)
{
	uint8_t tx[48];
	uint8_t rx[64];
	int len;

	for (int i = 0; i < (int)sizeof(tx); i++) {
		tx[i] = (uint8_t)(i ^ 0x3C);
	}

	zassert_ok(sdio_func_write_fifo(&host_func1, STREAM_FIFO_REG, tx,
					sizeof(tx)));

	len = sdio_stream_read(&stream_func, rx, sizeof(rx), K_MSEC(100));
	zassert_equal(len, (int)sizeof(tx), "stream read len %d", len);
	zassert_mem_equal(rx, tx, sizeof(tx), "stream RX payload mismatch");
}

/* poll() reports POLLIN once a frame is queued */
ZTEST(sdio_stream, test_poll_pollin)
{
	uint8_t tx[16] = {1, 2, 3, 4, 5, 6, 7, 8};
	uint32_t revents = 0;
	struct sdio_pkt *pkt;

	zassert_ok(sdio_func_write_fifo(&host_func1, STREAM_FIFO_REG, tx,
					sizeof(tx)));

	zassert_ok(sdio_stream_poll(&stream_func, SDIO_STREAM_POLLIN, &revents,
				    K_MSEC(100)));
	zassert_true(revents & SDIO_STREAM_POLLIN, "POLLIN not reported");

	pkt = sdio_stream_read_pkt(&stream_func, K_NO_WAIT);
	zassert_not_null(pkt, "no packet after POLLIN");
	zassert_equal(pkt->len, sizeof(tx));
	sdio_pkt_free(pkt);
}

/* Device queues a frame (asserts IRQ) -> host reads it back */
ZTEST(sdio_stream, test_device_to_host)
{
	uint8_t tx[40];
	uint8_t rx[40];
	uint32_t before = irq_count;

	for (int i = 0; i < (int)sizeof(tx); i++) {
		tx[i] = (uint8_t)(0x80 + i);
	}

	zassert_ok(sdio_stream_write(&stream_func, tx, sizeof(tx)));
	zassert_equal(irq_count, before + 1, "device did not assert IRQ");

	memset(rx, 0, sizeof(rx));
	zassert_ok(sdio_func_read_fifo(&host_func1, STREAM_FIFO_REG, rx,
				       sizeof(rx)));
	zassert_mem_equal(rx, tx, sizeof(tx), "device->host payload mismatch");
}

/* Controller-submitted RX (packet-oriented controller path) */
ZTEST(sdio_stream, test_rx_submit)
{
	uint8_t frame[24];
	uint8_t rx[24];
	int len;

	for (int i = 0; i < (int)sizeof(frame); i++) {
		frame[i] = (uint8_t)(i + 100);
	}

	zassert_ok(sdio_stream_rx_submit(&stream_func, frame, sizeof(frame)));
	len = sdio_stream_read(&stream_func, rx, sizeof(rx), K_MSEC(100));
	zassert_equal(len, (int)sizeof(frame));
	zassert_mem_equal(rx, frame, sizeof(frame), "rx_submit payload mismatch");
}

ZTEST_SUITE(sdio_stream, NULL, stream_setup, NULL, NULL, NULL);
