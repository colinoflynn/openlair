/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 * STM32F2xx MCU powersave implementation
 */
#include <stdint.h>
#include <string.h>
#include "platform_cmsis.h"
#include "core_cmFunc.h"
#include "platform_init.h"
#include "platform_constants.h"
#include "platform_assert.h"
#include "platform_peripheral.h"
#include "platform_isr_interface.h"
#include "platform_sleep.h"
#include "wwd_rtos_isr.h"
#include "wiced_defaults.h"
#include "wiced_low_power.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define NUMBER_OF_LSE_TICKS_PER_MS( scale_factor ) ( 32768 / 1000 / scale_factor )
#define CONVERT_FROM_TICKS_TO_MS( n, s )           ( n / NUMBER_OF_LSE_TICKS_PER_MS(s) )

/******************************************************
 *                    Constants
 ******************************************************/

#define RTC_INTERRUPT_EXTI_LINE       EXTI_Line22
#define WUT_COUNTER_MAX               0xffff
#define CK_SPRE_CLOCK_SOURCE_SELECTED 0xFFFF

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

#ifndef WICED_DISABLE_MCU_POWERSAVE
static unsigned long     stop_mode_power_down_hook( unsigned long sleep_ms );
static platform_result_t select_wut_prescaler_calculate_wakeup_time( unsigned long* wakeup_time, unsigned long sleep_ms, unsigned long* scale_factor );
#else
static unsigned long  idle_power_down_hook( unsigned long sleep_ms );
#endif

/******************************************************
 *               Variable Definitions
 ******************************************************/

#ifndef WICED_DISABLE_MCU_POWERSAVE
/* Default RTC time. Set to 12:20:30 08/04/2013 Monday */
static const platform_rtc_time_t default_rtc_time =
{
   .sec     = 30,
   .min     = 20,
   .hr      = 12,
   .weekday = 1,
   .date    = 8,
   .month   = 4,
   .year    = 13,
};

static unsigned long rtc_timeout_start_time       = 0;
static int32_t       stm32f2_clock_needed_counter = 0;
#endif /* #ifndef WICED_DISABLE_MCU_POWERSAVE */

/******************************************************
 *               Function Definitions
 ******************************************************/

platform_result_t platform_mcu_powersave_init( void )
{
   return PLATFORM_SUCCESS;
}

platform_result_t platform_mcu_powersave_disable( void )
{
   return PLATFORM_SUCCESS;
}

platform_result_t platform_mcu_powersave_enable( void )
{
#ifndef WICED_DISABLE_MCU_POWERSAVE
    return PLATFORM_SUCCESS;
#else
    return PLATFORM_FEATURE_DISABLED;
#endif
}

void platform_mcu_powersave_exit_notify( void )
{
}

#ifndef WICED_DISABLE_MCU_POWERSAVE
static platform_result_t select_wut_prescaler_calculate_wakeup_time( unsigned long* wakeup_time, unsigned long sleep_ms, unsigned long* scale_factor )
{

    return PLATFORM_SUCCESS;
}
#endif

/******************************************************
 *         IRQ Handlers Definition & Mapping
 ******************************************************/

#ifndef WICED_DISABLE_MCU_POWERSAVE
WWD_RTOS_DEFINE_ISR( rtc_wkup_irq )
{
    ;
}

WWD_RTOS_MAP_ISR( rtc_wkup_irq, RTC_WKUP_irq )
#endif

/******************************************************
 *               RTOS Powersave Hooks
 ******************************************************/

void platform_idle_hook( void )
{
    __asm("wfi");
}

uint32_t platform_power_down_hook( uint32_t sleep_ms )
{
#ifdef WICED_DISABLE_MCU_POWERSAVE
    /* If MCU powersave feature is disabled, enter idle mode when powerdown hook is called by the RTOS */
    return idle_power_down_hook( sleep_ms );

#else
    /* If MCU powersave feature is enabled, enter STOP mode when powerdown hook is called by the RTOS */
    return stop_mode_power_down_hook( sleep_ms );

#endif
}

#ifdef WICED_DISABLE_MCU_POWERSAVE
/* MCU Powersave is disabled */
static unsigned long idle_power_down_hook( unsigned long sleep_ms  )
{
    UNUSED_PARAMETER( sleep_ms );
    WICED_ENABLE_INTERRUPTS( );
    __asm("wfi");
    return 0;
}

#else
/* MCU Powersave is enabled */
static unsigned long stop_mode_power_down_hook( unsigned long sleep_ms )
{
    return 0;
}

#endif /* #ifdef WICED_DISABLE_MCU_POWERSAVE */

