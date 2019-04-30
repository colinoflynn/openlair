WICED_SDK_MAKEFILES           += ./WICED/platform/MCU/MK24F12/peripherals/libraries/libraries.mk ./libraries/utilities/ring_buffer/ring_buffer.mk ./WICED/platform/MCU/MK24F12/peripherals/peripherals.mk ./WICED/platform/GCC/GCC.mk ././WICED/platform/MCU/MK24F12/MK24F12.mk ./libraries/drivers/spi_flash/spi_flash.mk ././WICED/RTOS/NoOS/WWD/WWD.mk ./libraries/utilities/crc/crc.mk ./libraries/filesystems/wicedfs/wicedfs.mk ././WICED/WICED.mk ./platforms/FRDMK24/FRDMK24.mk ./WICED/RTOS/NoOS/NoOS.mk ./apps/waf/bootloader/bootloader.mk
TOOLCHAIN_NAME                := GCC
JTAG                          := CYW9WCD1EVAL1
WICED_SDK_LDFLAGS             += -Wl,--gc-sections -Wl,-Og -Wl,--cref -Wl,--gc-sections -Wl,-Og -Wl,--cref -mthumb -mcpu=cortex-m4 -Wl,-A,thumb -Wl,-z,max-page-size=0x10 -Wl,-z,common-page-size=0x10 -mlittle-endian -nostartfiles -Wl,--defsym,__STACKSIZE__=8000 -L ./WICED/platform/MCU/MK24F12/GCC -L ./WICED/platform/MCU/MK24F12/GCC/MK24F12
RESOURCE_CFLAGS               += -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\" -mthumb -mcpu=cortex-m4 -mlittle-endian
RESOURCE_CXXFLAGS             += -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\" -mthumb -mcpu=cortex-m4 -mlittle-endian
WICED_SDK_LINK_SCRIPT         += ././WICED/platform/MCU/MK24F12/GCC/bootloader.ld
WICED_SDK_LINK_SCRIPT_CMD     += -Wl,-T ././WICED/platform/MCU/MK24F12/GCC/bootloader.ld
WICED_SDK_PREBUILT_LIBRARIES  += 
WICED_SDK_CERTIFICATES        += 
WICED_SDK_PRE_APP_BUILDS      += 
WICED_SDK_DCT_LINK_SCRIPT     += ././WICED/platform/MCU/MK24F12/GCC/MK24F12/dct.ld
WICED_SDK_DCT_LINK_CMD        += -Wl,-T ././WICED/platform/MCU/MK24F12/GCC/MK24F12/dct.ld
WICED_SDK_APPLICATION_DCT     += 
WICED_SDK_WIFI_CONFIG_DCT_H   += ./include/default_wifi_config_dct.h
WICED_SDK_BT_CONFIG_DCT_H     += ./include/default_bt_config_dct.h
WICED_SDK_LINK_FILES          +=                  $(OUTPUT_DIR)/Modules/./WICED/platform/MCU/MK24F12/../../ARM_CM4/crt0_GCC.o $(OUTPUT_DIR)/Modules/./WICED/platform/MCU/MK24F12/../../ARM_CM4/hardfault_handler.o $(OUTPUT_DIR)/Modules/./WICED/platform/MCU/MK24F12/platform_vector_table.o  $(OUTPUT_DIR)/Modules/WICED/platform/GCC/mem_newlib.o $(OUTPUT_DIR)/Modules/WICED/platform/GCC/time_newlib.o $(OUTPUT_DIR)/Modules/WICED/platform/GCC/stdio_newlib.o      
WICED_SDK_INCLUDES            +=                                      -I./WICED/platform/MCU/MK24F12/peripherals/libraries/. -I./WICED/platform/MCU/MK24F12/peripherals/libraries/inc -I./WICED/platform/MCU/MK24F12/peripherals/libraries/../../../ARM_CM4/CMSIS -I./libraries/utilities/ring_buffer/. -I./WICED/platform/MCU/MK24F12/peripherals/. -I./WICED/platform/GCC/. -I././WICED/platform/MCU/MK24F12/. -I././WICED/platform/MCU/MK24F12/.. -I././WICED/platform/MCU/MK24F12/../.. -I././WICED/platform/MCU/MK24F12/../../include -I././WICED/platform/MCU/MK24F12/../../ARM_CM4 -I././WICED/platform/MCU/MK24F12/../../ARM_CM4/CMSIS -I././WICED/platform/MCU/MK24F12/peripherals -I././WICED/platform/MCU/MK24F12/WAF -I./libraries/drivers/spi_flash/. -I././WICED/RTOS/NoOS/WWD/. -I././WICED/RTOS/NoOS/WWD/Cortex_M3_M4 -I./libraries/utilities/crc/. -I./libraries/filesystems/wicedfs/src -I././WICED/WWD/include -I././WICED/security/BESL/include -I././WICED/security/BESL/host/WICED -I././WICED/security/BESL/crypto_internal -I././WICED/security/PostgreSQL/include -I././WICED/security/BESL/mbedtls_open/include -I././WICED/. -I././WICED/platform/include -I./platforms/FRDMK24/. -I./platforms/FRDMK24/./libraries/inputs/gpio_button -I./WICED/RTOS/NoOS/. -I./apps/waf/bootloader/. -I./apps/waf/bootloader/../../../libraries/utilities/linked_list -I./WICED/WWD/internal/chips/43362 -I./libraries -I./include
WICED_SDK_DEFINES             +=                                        -DSFLASH_APPS_HEADER_LOC=0x0000 -DUSE_STDPERIPH_DRIVER -D_STM3x_ -D_STM32x_ -DCPU_MK24FN1M0VLL12 -DPLATFORM_SUPPORTS_LOW_POWER_MODES -DMAX_WATCHDOG_TIMEOUT_SECONDS=22 -DUSING_WICEDFS -DNO_WICED_API -DWWD_STARTUP_DELAY=10 -DBOOTLOADER_MAGIC_NUMBER=0x4d435242 -DWWD_DIRECT_RESOURCES -DHSE_VALUE=26000000 -DCRLF_STDIO_REPLACEMENT -DWICED_PLATFORM_DOESNT_USE_TEMP_DMA_BUFFER -DWICED_USE_WIFI_POWER_PIN_ACTIVE_HIGH -DRTOS_NoOS=1 -DWICED_NO_WIFI -DWICED_DISABLE_STDIO -DWICED_DISABLE_MCU_POWERSAVE -DWICED_DCACHE_WTHROUGH -DNO_WIFI_FIRMWARE -DBOOTLOADER -DDCT_CRC32_CALCULATION_SIZE_ON_STACK=1024 -DWICED_SDK_WIFI_CONFIG_DCT_H=\"./include/default_wifi_config_dct.h\" -DWICED_SDK_BT_CONFIG_DCT_H=\"./include/default_bt_config_dct.h\"
COMPONENTS                := App_WICED_Bootloader_FRDMK24 NoOS Platform_FRDMK24 WICED Lib_Wiced_RO_FS Lib_crc WWD_NoOS_Interface Lib_SPI_Flash_Library_FRDMK24 MK24F12 common_GCC MK24F12_Peripheral_Drivers Lib_Ring_Buffer MK24F12_Peripheral_Libraries
BUS                       := SDIO
IMAGE_TYPE                := ram
NETWORK_FULL              := 
RTOS_FULL                 := NoOS
PLATFORM_DIRECTORY        := FRDMK24
APP_FULL                  := waf/bootloader
NETWORK                   := 
RTOS                      := NoOS
PLATFORM                  := FRDMK24
APPS_CHIP_REVISION        := 
USB                       := 
APP                       := bootloader
HOST_OPENOCD              := stm32f4x
HOST_ARCH                 := ARM_CM4
WICED_SDK_CERTIFICATE         := 
WICED_SDK_PRIVATE_KEY         := 
NO_BUILD_BOOTLOADER           := 
NO_BOOTLOADER_REQUIRED        := 
COMPILER_SPECIFIC_SYSTEM_DIR  := -isystem ./tools/ARM_GNU/Win32/bin/../../include -isystem ./tools/ARM_GNU/Win32/bin/../../lib/include -isystem ./tools/ARM_GNU/Win32/bin/../../lib/include-fixed
BOARD_SPECIFIC_OPENOCD_SCRIPT := 
App_WICED_Bootloader_FRDMK24_LOCATION         := ./apps/waf/bootloader/
NoOS_LOCATION         := ./WICED/RTOS/NoOS/
Platform_FRDMK24_LOCATION         := ./platforms/FRDMK24/
WICED_LOCATION         := ././WICED/
Lib_Wiced_RO_FS_LOCATION         := ./libraries/filesystems/wicedfs/
Lib_crc_LOCATION         := ./libraries/utilities/crc/
WWD_NoOS_Interface_LOCATION         := ././WICED/RTOS/NoOS/WWD/
Lib_SPI_Flash_Library_FRDMK24_LOCATION         := ./libraries/drivers/spi_flash/
MK24F12_LOCATION         := ././WICED/platform/MCU/MK24F12/
common_GCC_LOCATION         := ./WICED/platform/GCC/
MK24F12_Peripheral_Drivers_LOCATION         := ./WICED/platform/MCU/MK24F12/peripherals/
Lib_Ring_Buffer_LOCATION         := ./libraries/utilities/ring_buffer/
MK24F12_Peripheral_Libraries_LOCATION         := ./WICED/platform/MCU/MK24F12/peripherals/libraries/
App_WICED_Bootloader_FRDMK24_SOURCES          += bootloader.c
NoOS_SOURCES          += 
Platform_FRDMK24_SOURCES          += platform.c
WICED_SOURCES          += internal/wiced_core.c internal/wiced_low_power.c
Lib_Wiced_RO_FS_SOURCES          += src/wicedfs.c wicedfs_drivers.c
Lib_crc_SOURCES          += crc.c
WWD_NoOS_Interface_SOURCES          += wwd_rtos.c Cortex_M3_M4/noos.c
Lib_SPI_Flash_Library_FRDMK24_SOURCES          += spi_flash.c spi_flash_wiced.c
MK24F12_SOURCES          += ../../ARM_CM4/crt0_GCC.c ../../ARM_CM4/hardfault_handler.c ../../ARM_CM4/host_cm4.c ../platform_resource.c ../platform_stdio.c ../wiced_platform_common.c ../wwd_platform_separate_mcu.c ../wwd_resources.c ../wiced_apps_common.c ../wiced_waf_common.c ../platform_nsclock.c platform_vector_table.c platform_init.c platform_unhandled_isr.c platform_filesystem.c WAF/waf_platform.c AUDIO/k24f_hal.c AUDIO/wm8940_i2c.c  ../wiced_dct_internal_common.c ../wiced_dct_update.c
common_GCC_SOURCES          +=  mem_newlib.c time_newlib.c math_newlib.c cxx_funcs.c stdio_newlib.c
MK24F12_Peripheral_Drivers_SOURCES          += platform_mcu_powersave.c platform_gpio.c libraries/system_MK24F12.c 
Lib_Ring_Buffer_SOURCES          += ring_buffer.c
MK24F12_Peripheral_Libraries_SOURCES          += src/fsl_adc16.c src/fsl_clock.c src/fsl_cmp.c src/fsl_cmt.c src/fsl_common.c src/fsl_crc.c src/fsl_dac.c src/fsl_debug_console.c src/fsl_dmamux.c src/fsl_dspi_edma.c src/fsl_dspi.c src/fsl_edma.c src/fsl_ewm.c src/fsl_flash.c src/fsl_flexbus.c src/fsl_flexcan.c src/fsl_ftm.c src/fsl_gpio.c src/fsl_i2c_edma.c src/fsl_i2c.c src/fsl_llwu.c src/fsl_lptmr.c src/fsl_pdb.c src/fsl_pit.c src/fsl_pmc.c src/fsl_rcm.c src/fsl_rnga.c src/fsl_rtc.c src/fsl_sai_edma.c src/fsl_sai.c src/fsl_sdhc.c src/fsl_sim.c src/fsl_smc.c src/fsl_sysmpu.c src/fsl_uart_edma.c src/fsl_uart.c src/fsl_vref.c src/fsl_wdog.c src/misc.c src/clock_config.c src/board.c src/pin_mux.c src/fsl_mmcau.c 
App_WICED_Bootloader_FRDMK24_CHECK_HEADERS    += 
NoOS_CHECK_HEADERS    += 
Platform_FRDMK24_CHECK_HEADERS    += 
WICED_CHECK_HEADERS    += internal/wiced_internal_api.h ../include/default_wifi_config_dct.h ../include/resource.h ../include/wiced.h ../include/wiced_defaults.h ../include/wiced_easy_setup.h ../include/wiced_framework.h ../include/wiced_management.h ../include/wiced_platform.h ../include/wiced_rtos.h ../include/wiced_tcpip.h ../include/wiced_time.h ../include/wiced_utilities.h ../include/wiced_crypto.h ../include/wiced_wifi.h ../include/wiced_wifi_deep_sleep.h
Lib_Wiced_RO_FS_CHECK_HEADERS    += 
Lib_crc_CHECK_HEADERS    += 
WWD_NoOS_Interface_CHECK_HEADERS    += wwd_rtos.h
Lib_SPI_Flash_Library_FRDMK24_CHECK_HEADERS    += 
MK24F12_CHECK_HEADERS    += 
common_GCC_CHECK_HEADERS    += 
MK24F12_Peripheral_Drivers_CHECK_HEADERS    += 
Lib_Ring_Buffer_CHECK_HEADERS    += 
MK24F12_Peripheral_Libraries_CHECK_HEADERS    += 
App_WICED_Bootloader_FRDMK24_INCLUDES         := 
NoOS_INCLUDES         := 
Platform_FRDMK24_INCLUDES         := 
WICED_INCLUDES         := 
Lib_Wiced_RO_FS_INCLUDES         := 
Lib_crc_INCLUDES         := 
WWD_NoOS_Interface_INCLUDES         := 
Lib_SPI_Flash_Library_FRDMK24_INCLUDES         := 
MK24F12_INCLUDES         := 
common_GCC_INCLUDES         := 
MK24F12_Peripheral_Drivers_INCLUDES         := 
Lib_Ring_Buffer_INCLUDES         := 
MK24F12_Peripheral_Libraries_INCLUDES         := 
App_WICED_Bootloader_FRDMK24_DEFINES          := 
NoOS_DEFINES          := 
Platform_FRDMK24_DEFINES          := 
WICED_DEFINES          := 
Lib_Wiced_RO_FS_DEFINES          := 
Lib_crc_DEFINES          := 
WWD_NoOS_Interface_DEFINES          := 
Lib_SPI_Flash_Library_FRDMK24_DEFINES          := -DSFLASH_SUPPORT_SST_PARTS -DSFLASH_SUPPORT_MACRONIX_PARTS -DSFLASH_SUPPORT_EON_PARTS -DSFLASH_SUPPORT_MICRON_PARTS
MK24F12_DEFINES          := 
common_GCC_DEFINES          := 
MK24F12_Peripheral_Drivers_DEFINES          := 
Lib_Ring_Buffer_DEFINES          := 
MK24F12_Peripheral_Libraries_DEFINES          := 
App_WICED_Bootloader_FRDMK24_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         
NoOS_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         
Platform_FRDMK24_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         
WICED_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         -Wall -fsigned-char -ffunction-sections -fdata-sections -fno-common -std=gnu11 -fdiagnostics-color  -Wstrict-prototypes  -W -Wshadow  -Wwrite-strings -pedantic -std=c11 -U__STRICT_ANSI__ -Wconversion -Wextra -Wdeclaration-after-statement -Wconversion -Waddress -Wlogical-op -Wstrict-prototypes -Wold-style-definition -Wmissing-prototypes -Wmissing-declarations -Wmissing-field-initializers -Wdouble-promotion -Wswitch-enum -Wswitch-default -Wuninitialized -Wunknown-pragmas -Wfloat-equal  -Wundef  -Wshadow 
Lib_Wiced_RO_FS_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         -Wall -fsigned-char -ffunction-sections -fdata-sections -fno-common -std=gnu11 -fdiagnostics-color  -Wstrict-prototypes  -W -Wshadow  -Wwrite-strings -pedantic -std=c11 -U__STRICT_ANSI__ -Wconversion -Wextra -Wdeclaration-after-statement -Wconversion -Waddress -Wlogical-op -Wstrict-prototypes -Wold-style-definition -Wmissing-prototypes -Wmissing-declarations -Wmissing-field-initializers -Wdouble-promotion -Wswitch-enum -Wswitch-default -Wuninitialized -Wunknown-pragmas -Wfloat-equal  -Wundef  -Wshadow 
Lib_crc_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         
WWD_NoOS_Interface_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         -Wall -fsigned-char -ffunction-sections -fdata-sections -fno-common -std=gnu11 -fdiagnostics-color  -Wstrict-prototypes  -W -Wshadow  -Wwrite-strings -pedantic -std=c11 -U__STRICT_ANSI__ -Wconversion -Wextra -Wdeclaration-after-statement -Wconversion -Waddress -Wlogical-op -Wstrict-prototypes -Wold-style-definition -Wmissing-prototypes -Wmissing-declarations -Wmissing-field-initializers -Wdouble-promotion -Wswitch-enum -Wswitch-default -Wuninitialized -Wunknown-pragmas -Wfloat-equal  -Wundef  -Wshadow 
Lib_SPI_Flash_Library_FRDMK24_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         -Wall -fsigned-char -ffunction-sections -fdata-sections -fno-common -std=gnu11 -fdiagnostics-color  -Wstrict-prototypes  -W -Wshadow  -Wwrite-strings -pedantic -std=c11 -U__STRICT_ANSI__ -Wconversion -Wextra -Wdeclaration-after-statement -Wconversion -Waddress -Wlogical-op -Wstrict-prototypes -Wold-style-definition -Wmissing-prototypes -Wmissing-declarations -Wmissing-field-initializers -Wdouble-promotion -Wswitch-enum -Wswitch-default -Wuninitialized -Wunknown-pragmas -Wfloat-equal  -Wundef  -Wshadow 
MK24F12_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         -Wall -fsigned-char -ffunction-sections -fdata-sections -fno-common -std=gnu11 -fdiagnostics-color  -Wstrict-prototypes  -W -Wshadow  -Wwrite-strings -pedantic -std=c11 -U__STRICT_ANSI__ -Wconversion -Wextra -Wdeclaration-after-statement -Wconversion -Waddress -Wlogical-op -Wstrict-prototypes -Wold-style-definition -Wmissing-prototypes -Wmissing-declarations -Wmissing-field-initializers -Wdouble-promotion -Wswitch-enum -Wswitch-default -Wuninitialized -Wunknown-pragmas -Wfloat-equal  -Wundef  -Wshadow 
common_GCC_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         
MK24F12_Peripheral_Drivers_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         
Lib_Ring_Buffer_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         
MK24F12_Peripheral_Libraries_CFLAGS           := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4    -mlittle-endian         
App_WICED_Bootloader_FRDMK24_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
NoOS_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
Platform_FRDMK24_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
WICED_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
Lib_Wiced_RO_FS_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
Lib_crc_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
WWD_NoOS_Interface_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
Lib_SPI_Flash_Library_FRDMK24_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
MK24F12_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
common_GCC_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
MK24F12_Peripheral_Drivers_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
Lib_Ring_Buffer_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
MK24F12_Peripheral_Libraries_CXXFLAGS         := -DWICED_VERSION=\"Wiced_006.002.001.0002\" -DBUS=\"$(BUS)\" -Ibuild/waf.bootloader-NoOS-FRDMK24/resources/ -DPLATFORM=\"$(PLATFORM)\" -DAPPS_CHIP_REVISION=\"$(APPS_CHIP_REVISION)\"     -mthumb -mcpu=cortex-m4  -mlittle-endian         
App_WICED_Bootloader_FRDMK24_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
NoOS_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
Platform_FRDMK24_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
WICED_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
Lib_Wiced_RO_FS_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
Lib_crc_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
WWD_NoOS_Interface_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
Lib_SPI_Flash_Library_FRDMK24_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
MK24F12_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
common_GCC_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
MK24F12_Peripheral_Drivers_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
Lib_Ring_Buffer_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
MK24F12_Peripheral_Libraries_ASMFLAGS         :=     -mcpu=cortex-m4 -mfpu=softvfp  -mlittle-endian         
App_WICED_Bootloader_FRDMK24_RESOURCES        := 
NoOS_RESOURCES        := 
Platform_FRDMK24_RESOURCES        := 
WICED_RESOURCES        := 
Lib_Wiced_RO_FS_RESOURCES        := 
Lib_crc_RESOURCES        := 
WWD_NoOS_Interface_RESOURCES        := 
Lib_SPI_Flash_Library_FRDMK24_RESOURCES        := 
MK24F12_RESOURCES        := 
common_GCC_RESOURCES        := 
MK24F12_Peripheral_Drivers_RESOURCES        := 
Lib_Ring_Buffer_RESOURCES        := 
MK24F12_Peripheral_Libraries_RESOURCES        := 
App_WICED_Bootloader_FRDMK24_MAKEFILE         := ./apps/waf/bootloader/bootloader.mk
NoOS_MAKEFILE         := ./WICED/RTOS/NoOS/NoOS.mk
Platform_FRDMK24_MAKEFILE         := ./platforms/FRDMK24/FRDMK24.mk
WICED_MAKEFILE         := ././WICED/WICED.mk
Lib_Wiced_RO_FS_MAKEFILE         := ./libraries/filesystems/wicedfs/wicedfs.mk
Lib_crc_MAKEFILE         := ./libraries/utilities/crc/crc.mk
WWD_NoOS_Interface_MAKEFILE         := ././WICED/RTOS/NoOS/WWD/WWD.mk
Lib_SPI_Flash_Library_FRDMK24_MAKEFILE         := ./libraries/drivers/spi_flash/spi_flash.mk
MK24F12_MAKEFILE         := ././WICED/platform/MCU/MK24F12/MK24F12.mk
common_GCC_MAKEFILE         := ./WICED/platform/GCC/GCC.mk
MK24F12_Peripheral_Drivers_MAKEFILE         := ./WICED/platform/MCU/MK24F12/peripherals/peripherals.mk
Lib_Ring_Buffer_MAKEFILE         := ./libraries/utilities/ring_buffer/ring_buffer.mk
MK24F12_Peripheral_Libraries_MAKEFILE         := ./WICED/platform/MCU/MK24F12/peripherals/libraries/libraries.mk
App_WICED_Bootloader_FRDMK24_PRE_BUILD_TARGETS:= 
NoOS_PRE_BUILD_TARGETS:= 
Platform_FRDMK24_PRE_BUILD_TARGETS:= 
WICED_PRE_BUILD_TARGETS:= 
Lib_Wiced_RO_FS_PRE_BUILD_TARGETS:= 
Lib_crc_PRE_BUILD_TARGETS:= 
WWD_NoOS_Interface_PRE_BUILD_TARGETS:= 
Lib_SPI_Flash_Library_FRDMK24_PRE_BUILD_TARGETS:= 
MK24F12_PRE_BUILD_TARGETS:= 
common_GCC_PRE_BUILD_TARGETS:= 
MK24F12_Peripheral_Drivers_PRE_BUILD_TARGETS:= 
Lib_Ring_Buffer_PRE_BUILD_TARGETS:= 
MK24F12_Peripheral_Libraries_PRE_BUILD_TARGETS:= 
App_WICED_Bootloader_FRDMK24_PREBUILT_LIBRARY := 
NoOS_PREBUILT_LIBRARY := 
Platform_FRDMK24_PREBUILT_LIBRARY := 
WICED_PREBUILT_LIBRARY := 
Lib_Wiced_RO_FS_PREBUILT_LIBRARY := 
Lib_crc_PREBUILT_LIBRARY := 
WWD_NoOS_Interface_PREBUILT_LIBRARY := 
Lib_SPI_Flash_Library_FRDMK24_PREBUILT_LIBRARY := 
MK24F12_PREBUILT_LIBRARY := 
common_GCC_PREBUILT_LIBRARY := 
MK24F12_Peripheral_Drivers_PREBUILT_LIBRARY := 
Lib_Ring_Buffer_PREBUILT_LIBRARY := 
MK24F12_Peripheral_Libraries_PREBUILT_LIBRARY := 
App_WICED_Bootloader_FRDMK24_BUILD_TYPE       := release
NoOS_BUILD_TYPE       := release
Platform_FRDMK24_BUILD_TYPE       := release
WICED_BUILD_TYPE       := release
Lib_Wiced_RO_FS_BUILD_TYPE       := release
Lib_crc_BUILD_TYPE       := release
WWD_NoOS_Interface_BUILD_TYPE       := release
Lib_SPI_Flash_Library_FRDMK24_BUILD_TYPE       := release
MK24F12_BUILD_TYPE       := release
common_GCC_BUILD_TYPE       := release
MK24F12_Peripheral_Drivers_BUILD_TYPE       := release
Lib_Ring_Buffer_BUILD_TYPE       := release
MK24F12_Peripheral_Libraries_BUILD_TYPE       := release
BOARD_REVISION   := 
WICED_SDK_UNIT_TEST_SOURCES   :=        ././WICED/internal/unit/wiced_unit.cpp  ./libraries/filesystems/wicedfs/src/unit/wicedfs_unit_images.c ./libraries/filesystems/wicedfs/src/unit/wicedfs_unit.cpp                
APP_WWD_ONLY              := TRUE
USES_BOOTLOADER_OTA       := 1
NODCT                     := 
ALL_RESOURCES             := 
RESOURCES_LOCATION        := RESOURCES_IN_DIRECT_RESOURCES
INTERNAL_MEMORY_RESOURCES := 
EXTRA_TARGET_MAKEFILES :=    ./tools/makefiles/standard_platform_targets.mk
EXTRA_PLATFORM_MAKEFILES := 
APPS_LUT_HEADER_LOC := 0x0000
APPS_START_SECTOR := 1 
FR_APP := 
OTA2_FAILSAFE_APP := 
OTA_APP := 
DCT_IMAGE := 
FILESYSTEM_IMAGE :=  
WIFI_FIRMWARE :=  
APP0 :=  
APP1 :=  
APP2 :=  
FR_APP_SECURE := 
OTA_APP_SECURE := 
FAILSAFE_APP_SECURE := 
WICED_ROM_SYMBOL_LIST_FILE := 
WICED_SDK_CHIP_SPECIFIC_SCRIPT :=             
WICED_SDK_CONVERTER_OUTPUT_FILE :=             
WICED_SDK_FINAL_OUTPUT_FILE :=             
WICED_RAM_STUB_LIST_FILE := 
DCT_IMAGE_SECURE := 
FILESYSTEM_IMAGE_SECURE := 
WIFI_FIRMWARE_SECURE := 
APP0_SECURE := 
APP1_SECURE := 
APP2_SECURE := 
