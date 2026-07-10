# Copyright 2026 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SDIO_DC_VIRTUAL sdio_dc_virtual.c)
zephyr_library_sources_ifdef(CONFIG_SDIO_DC_RW6XX rw6xx_sdio_dc.c)
if(CONFIG_SDIO_DC_RW6XX)
  zephyr_include_directories(
    ${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk-ng/components/sdio_dev)
endif()
