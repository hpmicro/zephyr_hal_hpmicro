# Copyright 2022 hpmicro
# SPDX-License-Identifier: BSD-3-Clause
set(HPM_SDK_PROJECT_NAME ${ZEPHYR_CURRENT_LIBRARY})
set(HPM_SDK_INTERFACE_NAME zephyr_interface)
set(HPM_SDK_TARGET_SCOPE "private")
set(ADAPTED_ZEPHYR 1)
set(USE_CUSTOM_STARTUP 1)
include(hpm_sdk/cmake/cmake-ext.cmake)
add_subdirectory_ifdef(CONFIG_HAS_HPMICRO_HAL hpm_sdk/arch)
add_subdirectory_ifdef(CONFIG_HAS_HPMICRO_HAL hpm_sdk/boards)
add_subdirectory_ifdef(CONFIG_HAS_HPMICRO_HAL hpm_sdk/soc)
add_subdirectory_ifdef(CONFIG_HAS_HPMICRO_HAL hpm_sdk/drivers)