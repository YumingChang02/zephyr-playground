# SPDX-License-Identifier: Apache-2.0
# Sysbuild configuration for multi-core AMP on ESP32-S3

# Automatically derive remote board qualifier from primary target:
string(REPLACE "procpu" "appcpu" REMOTE_CPU "${BOARD_QUALIFIERS}")
string(CONFIGURE "${BOARD}/${REMOTE_CPU}" IPM_REMOTE_BOARD)

if(${REMOTE_CPU} STREQUAL ${BOARD_QUALIFIERS})
  message(FATAL_ERROR "BOARD_QUALIFIERS name error. Please ensure target board contains procpu qualifier.")
endif()

# Locate remote directory whether invoked from root or app subfolder
if(EXISTS "${APP_DIR}/../remote")
  set(REMOTE_DIR "${APP_DIR}/../remote")
elseif(EXISTS "${APP_DIR}/remote")
  set(REMOTE_DIR "${APP_DIR}/remote")
else()
  message(FATAL_ERROR "Could not find remote directory for APP_CPU firmware.")
endif()

# Add external project for APP_CPU
ExternalZephyrProject_Add(
    APPLICATION remote
    SOURCE_DIR ${REMOTE_DIR}
    BOARD ${IPM_REMOTE_BOARD}
)

# Ensure remote compiles alongside app
add_dependencies(app remote)
sysbuild_add_dependencies(CONFIGURE app remote)

if(SB_CONFIG_BOOTLOADER_MCUBOOT)
  sysbuild_add_dependencies(FLASH remote mcuboot)
endif()
