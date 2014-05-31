
# check the TARGET_CHIP and BOARD
TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_PCA10001


# check CFAGS remove extra -DNRF51 -DDEBUG_NRF_USER
CFLAGS += -DNRF51822_QFAA_CA
CFLAGS += -DBOARD_PCA10001
CFLAGS += -DNRF51 -DBOARD_PROJECT_W1 -DBLE_STACK_SUPPORT_REQD -DDEBUG_NRF_USER

SDK_PATH = ../../../../

C_SOURCE_FILES += main.c
C_SOURCE_FILES += ../ble_flash.c
C_SOURCE_FILES += ../ble_nus.c
C_SOURCE_FILES += ../data_transfer.c
C_SOURCE_FILES += ../flash.c
#C_SOURCE_FILES += ../../../../Release/History_W1FW.txt
C_SOURCE_FILES += ../rtc.c
C_SOURCE_FILES += ../sensor.c
C_SOURCE_FILES += $(SDK_PATH)Source/app_common/app_timer.c
C_SOURCE_FILES += $(SDK_PATH)Source/app_common/pstorage.c
C_SOURCE_FILES += $(SDK_PATH)Source/ble/ble_advdata.c
C_SOURCE_FILES += $(SDK_PATH)Source/ble/ble_conn_params.c
C_SOURCE_FILES += $(SDK_PATH)Source/ble/ble_debug_assert_handler.c
C_SOURCE_FILES += $(SDK_PATH)Source/ble/ble_error_log.c
C_SOURCE_FILES += $(SDK_PATH)Source/ble/ble_services/ble_bas.c
C_SOURCE_FILES += $(SDK_PATH)Source/ble/ble_services/ble_dis.c
C_SOURCE_FILES += $(SDK_PATH)Source/ble/ble_services/ble_srv_common.c
C_SOURCE_FILES += $(SDK_PATH)Source/sd_common/softdevice_handler.c
C_SOURCE_FILES += $(SDK_PATH)Source/simple_uart/simple_uart.c
C_SOURCE_FILES += ../step.c
C_SOURCE_FILES += ../twi_hw_master_SD_4_4.c

INCLUDEPATHS += -I"$(SDK_PATH)Include"
INCLUDEPATHS += -I"../Include"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/softdevice"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/boards"

OUTPUT_FILENAME := uart_example_gcc


DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := s210

include $(SDK_PATH)Source/templates/gcc/Makefile.common

