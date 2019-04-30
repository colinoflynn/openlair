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
 * Defines board support package for BCM943362WCD6 board
 */
#include "platform.h"
#include "platform_config.h"
#include "platform_init.h"
#include "platform_isr.h"
#include "platform_peripheral.h"
#include "wwd_platform_common.h"
#include "wwd_rtos_isr.h"
#include "wiced_defaults.h"
#include "wiced_platform.h"
#include "MK24F12.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* GPIO pin table. Used by WICED/platform/MCU/wiced_platform_common.c*/
const platform_gpio_t platform_gpio_pins[] =
{
    [WICED_GPIO_1]  = { GPIOE,  0  },
    [WICED_GPIO_2]  = { GPIOE,  1  },
    [WICED_GPIO_3]  = { GPIOE,  2  },
    [WICED_GPIO_4]  = { GPIOE,  3  },
    [WICED_GPIO_5]  = { GPIOE,  4  },
    [WICED_GPIO_6]  = { GPIOE,  5  },
    [WICED_GPIO_7]  = { GPIOE,  6  },
    [WICED_GPIO_8]  = { GPIOE,  24 },
    [WICED_GPIO_9]  = { GPIOE,  25 },
    [WICED_GPIO_10] = { GPIOE,  26 },
    [WICED_GPIO_11] = { GPIOA,  0  },
    [WICED_GPIO_12] = { GPIOA,  1  },
    [WICED_GPIO_13] = { GPIOA,  2  },
    [WICED_GPIO_14] = { GPIOA,  3  },
    [WICED_GPIO_15] = { GPIOA,  4  },
    [WICED_GPIO_16] = { GPIOA,  5  },
    [WICED_GPIO_17] = { GPIOA,  12 },
    [WICED_GPIO_18] = { GPIOA,  13 },
    [WICED_GPIO_19] = { GPIOA,  14 },
    [WICED_GPIO_20] = { GPIOA,  15 },
    [WICED_GPIO_21] = { GPIOA,  16 },
    [WICED_GPIO_22] = { GPIOA,  17 },
    [WICED_GPIO_23] = { GPIOA,  18 },
    [WICED_GPIO_24] = { GPIOA,  19 },
    [WICED_GPIO_25] = { GPIOB,  0  },
    [WICED_GPIO_26] = { GPIOB,  1  },
    [WICED_GPIO_27] = { GPIOB,  3  },
    [WICED_GPIO_28] = { GPIOB,  9  },
    [WICED_GPIO_29] = { GPIOB,  10 },
    [WICED_GPIO_30] = { GPIOB,  11 },
    [WICED_GPIO_31] = { GPIOB,  16 },
    [WICED_GPIO_32] = { GPIOB,  17 },
    [WICED_GPIO_33] = { GPIOB,  18 },
    [WICED_GPIO_34] = { GPIOB,  19 },
    [WICED_GPIO_35] = { GPIOB,  20 },
    [WICED_GPIO_36] = { GPIOB,  21 },
    [WICED_GPIO_37] = { GPIOB,  22 },
    [WICED_GPIO_38] = { GPIOB,  23 },
    [WICED_GPIO_39] = { GPIOC,  0  },
    [WICED_GPIO_40] = { GPIOC,  1  },
    [WICED_GPIO_41] = { GPIOC,  2  },
    [WICED_GPIO_42] = { GPIOC,  3  },
    [WICED_GPIO_43] = { GPIOC,  4  },
    [WICED_GPIO_44] = { GPIOC,  5  },
    [WICED_GPIO_45] = { GPIOC,  6  },
    [WICED_GPIO_46] = { GPIOC,  7  },
    [WICED_GPIO_47] = { GPIOC,  8  },
    [WICED_GPIO_48] = { GPIOC,  9  },
    [WICED_GPIO_49] = { GPIOC,  10 },
    [WICED_GPIO_50] = { GPIOC,  11 },
    [WICED_GPIO_51] = { GPIOC,  12 },
    [WICED_GPIO_51] = { GPIOC,  13 },
    [WICED_GPIO_53] = { GPIOC,  14 },
    [WICED_GPIO_54] = { GPIOC,  15 },
    [WICED_GPIO_55] = { GPIOC,  16 },
    [WICED_GPIO_56] = { GPIOC,  18 },
    [WICED_GPIO_57] = { GPIOC,  17 },
    [WICED_GPIO_58] = { GPIOC,  18 },
    [WICED_GPIO_59] = { GPIOD,  0  },
    [WICED_GPIO_60] = { GPIOD,  1  },
    [WICED_GPIO_61] = { GPIOD,  2  },
    [WICED_GPIO_62] = { GPIOD,  3  },
    [WICED_GPIO_63] = { GPIOD,  4  },
    [WICED_GPIO_64] = { GPIOD,  5  },
    [WICED_GPIO_65] = { GPIOD,  6  },
    [WICED_GPIO_66] = { GPIOD,  7  },

};

/* UART peripherals and runtime drivers. Used by WICED/platform/MCU/wiced_platform_common.c*/
const platform_uart_t platform_uart_peripherals[] =
{
    [WICED_UART_0] =
    {
        .port               = UART0,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_19 ],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_20],
        .cts_pin            = &platform_gpio_pins[WICED_GPIO_21],
        .rts_pin            = &platform_gpio_pins[WICED_GPIO_22],
    },
    [WICED_UART_1] =
    {
        .port               = UART1,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_1],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_2],
        .cts_pin            = &platform_gpio_pins[WICED_GPIO_3],
        .rts_pin            = &platform_gpio_pins[WICED_GPIO_4],
    },
    [WICED_UART_2] =
    {
        .port               = UART2,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_62],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_61],
        .cts_pin            = &platform_gpio_pins[WICED_GPIO_60],
        .rts_pin            = &platform_gpio_pins[WICED_GPIO_59],
    },
    [WICED_UART_3] =
    {
        .port               = UART3,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_30],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_29],
        .cts_pin            = &platform_gpio_pins[WICED_GPIO_28],
        .rts_pin            = NULL,
    },
    [WICED_UART_4] =
    {
        .port               = UART4,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_8],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_9],
        .cts_pin            = &platform_gpio_pins[WICED_GPIO_10],
        .rts_pin            = NULL,
    },
};
platform_uart_driver_t platform_uart_drivers[WICED_UART_MAX];


/* UART standard I/O configuration */
#ifndef WICED_DISABLE_STDIO
static const platform_uart_config_t stdio_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_DISABLED,
};
#endif

/* Wi-Fi control pins. Used by WICED/platform/MCU/wwd_platform_common.c */
const platform_gpio_t wifi_control_pins[] =
{
    [WWD_PIN_POWER      ] = { GPIOC,  0 },
    [WWD_PIN_RESET      ] = { GPIOB,  1 },
#if defined ( WICED_USE_WIFI_32K_CLOCK_MCO )
    [WWD_PIN_32K_CLK    ] = { GPIOA,  8 },
#else
    [WWD_PIN_32K_CLK        ] = { GPIOA, 11 },
#endif
    [WWD_PIN_BOOTSTRAP_0] = { GPIOB,  0 },
    [WWD_PIN_BOOTSTRAP_1] = { GPIOB,  1 },
};

/* Wi-Fi SDIO bus pins. Used by WICED/platform/MK64F12/WWD/wwd_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
    [WWD_PIN_SDIO_OOB_IRQ] = { GPIOB,  0  },
    [WWD_PIN_SDIO_CLK    ] = { GPIOE,  2  },
    [WWD_PIN_SDIO_CMD    ] = { GPIOE,  3  },
    [WWD_PIN_SDIO_D0     ] = { GPIOE,  1  },
    [WWD_PIN_SDIO_D1     ] = { GPIOE,  0  },
    [WWD_PIN_SDIO_D2     ] = { GPIOE,  4  },
    [WWD_PIN_SDIO_D3     ] = { GPIOE,  5  },
};

const wiced_gpio_t platform_gpio_leds[PLATFORM_LED_COUNT] =
{
     [WICED_LED_INDEX_1] = WICED_LED1,
     [WICED_LED_INDEX_2] = WICED_LED2,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void platform_init_peripheral_irq_priorities( void )
{
    /* Interrupt priority setup. Called by WICED/platform/MCU/MK64F12/platform_init.c */


    // TFL : Deleted SPI, DMA, RTC and any extra unnecessary interrupts
    NVIC_SetPriority( SDHC_IRQn        ,  2 ); /* WLAN SDIO           */
    NVIC_SetPriority( PORTA_IRQn       , 10 ); /* GPIO                */
    NVIC_SetPriority( PORTB_IRQn       , 10 ); /* GPIO                */
    NVIC_SetPriority( PORTC_IRQn       , 10 ); /* GPIO                */
    NVIC_SetPriority( PORTD_IRQn       , 10 ); /* GPIO                */
    NVIC_SetPriority( PORTE_IRQn       , 10 ); /* GPIO                */
    NVIC_SetPriority( I2S0_Tx_IRQn     ,  3 ); /* AUDIO CONTROL       */
    NVIC_SetPriority( I2S0_Rx_IRQn     ,  3 ); /* AUDIO CONTROL       */
}

/* LEDs on this platform are active HIGH */
platform_result_t platform_led_set_state(int led_index, int off_on )
{
    if ((led_index >= 0) && (led_index < PLATFORM_LED_COUNT))
    {
        switch (off_on)
        {
            case WICED_LED_OFF:
                platform_gpio_output_low( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                break;
            case WICED_LED_ON:
                platform_gpio_output_high( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                break;
        }
        return PLATFORM_SUCCESS;
    }
    return PLATFORM_BADARG;
}

void platform_led_init( void )
{
    /* Initialise LEDs and turn off by default */
    //platform_gpio_init( &platform_gpio_pins[WICED_LED1], OUTPUT_PUSH_PULL );
    //platform_gpio_init( &platform_gpio_pins[WICED_LED2], OUTPUT_PUSH_PULL );
    //platform_led_set_state(WICED_LED_INDEX_1, WICED_LED_ON);
    //platform_led_set_state(WICED_LED_INDEX_2, WICED_LED_OFF);

    //platform_gpio_init( &platform_gpio_pins[WICED_GPIO_28], OUTPUT_PUSH_PULL );
    //platform_gpio_output_low( &platform_gpio_pins[platform_gpio_leds[WICED_GPIO_28]] );

 }


void platform_init_external_devices( void )
{
    /* Initialise LEDs and turn off by default */
    platform_led_init();

    /* Initialise buttons to input by default */
#ifndef WICED_DISABLE_STDIO
    /* Initialise UART standard I/O */
    platform_stdio_init( &platform_uart_drivers[STDIO_UART], &platform_uart_peripherals[STDIO_UART], &stdio_config );
#endif
}

uint32_t  platform_get_button_press_time ( int button_index, int led_index, uint32_t max_time )
{
    uint32_t        button_press_timer = 0;

    return button_press_timer;
}

uint32_t  platform_get_factory_reset_button_time ( uint32_t max_time )
{
    return platform_get_button_press_time ( PLATFORM_FACTORY_RESET_BUTTON_INDEX, PLATFORM_RED_LED_INDEX, max_time );
}


/******************************************************
 *           Interrupt Handler Definitions
 ******************************************************/


/******************************************************
 *            Interrupt Handlers Mapping
 ******************************************************/

