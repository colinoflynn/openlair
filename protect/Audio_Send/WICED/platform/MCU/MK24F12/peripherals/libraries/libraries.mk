#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#

NAME = MK24F12_Peripheral_Libraries

GLOBAL_INCLUDES :=  . \
                    inc \
                    ../../../$(HOST_ARCH)/CMSIS

$(NAME)_SOURCES := \
					src/fsl_adc16.c\
					src/fsl_clock.c\
					src/fsl_cmp.c\
					src/fsl_cmt.c\
					src/fsl_common.c\
					src/fsl_crc.c\
					src/fsl_dac.c\
					src/fsl_debug_console.c\
					src/fsl_dmamux.c\
					src/fsl_dspi_edma.c\
					src/fsl_dspi.c\
					src/fsl_edma.c\
					src/fsl_ewm.c\
					src/fsl_flash.c\
					src/fsl_flexbus.c\
					src/fsl_flexcan.c\
					src/fsl_ftm.c\
					src/fsl_gpio.c\
					src/fsl_i2c_edma.c\
					src/fsl_i2c.c\
					src/fsl_llwu.c\
					src/fsl_lptmr.c\
					src/fsl_pdb.c\
					src/fsl_pit.c\
					src/fsl_pmc.c\
					src/fsl_rcm.c\
					src/fsl_rnga.c\
					src/fsl_rtc.c\
					src/fsl_sai_edma.c\
					src/fsl_sai.c\
					src/fsl_sdhc.c\
					src/fsl_sim.c\
					src/fsl_smc.c\
					src/fsl_sysmpu.c\
					src/fsl_uart_edma.c\
					src/fsl_uart.c\
					src/fsl_vref.c\
					src/fsl_wdog.c\
					src/misc.c\
					src/clock_config.c\
					src/board.c\
					src/pin_mux.c\
					src/fsl_mmcau.c\
                   

