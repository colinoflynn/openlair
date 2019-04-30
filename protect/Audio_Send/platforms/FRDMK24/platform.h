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
 * Defines peripherals available for use on BCM943362WCD6 board
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
/*
BCM943362WCD4 platform pin definitions ...
+--------------------------------------------------------------------------------------------------------+
| Enum ID       |Pin |   Pin Name on    |    Module     | STM32| Peripheral  |    Board     | Peripheral  |
|               | #  |      Module      |  GPIO Alias   | Port | Available   |  Connection  |   Alias     |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_1  | 17 | MICRO_WKUP       | WICED_GPIO_1  | A  0 | GPIO        |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_2  | 18 | MICRO_ADC_IN1    | WICED_GPIO_2  | A  1 | GPIO        | BUTTON SW2   |             |
|               |    |                  |               |      | TIM2_CH2    |              |             |
|               |    |                  |               |      | TIM5_CH2    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_3  | 19 | MICRO_ADC_IN2    | WICED_GPIO_3  | A  2 | ADC123_IN2  |              |             |
|               |    |                  |               |      | GPIO        | BUTTON SW1   |             |
|               |    |                  |               |      | TIM2_CH3    |              |             |
|               |    |                  |               |      | TIM5_CH3    |              |             |
|               |    |                  |               |      | TIM9_CH1    |              |             |
|               |    |                  |               |      | USART2_TX   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_4  | 21 | MICRO_ADC_IN3    | WICED_GPIO_4  | A  3 | ADC123_IN3  |              | WICED_ADC_3 |
|               |    |                  |               |      | GPIO        | THERMISTOR   |             |
|               |    |                  |               |      | TIM2_CH4    |              |             |
|               |    |                  |               |      | TIM5_CH4    |              |             |
|               |    |                  |               |      | TIM9_CH2    |              |             |
|               |    |                  |               |      | UART2_RX    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_5  | 27 | MICRO_SPI_SSN    | WICED_GPIO_5  | A  4 | ADC12_IN4   |              |             |
|               |    |                  |               |      | DAC1_OUT    |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | I2S3_WS     |              |             |
|               |    |                  |               |      | SPI1_NSS    |              |             |
|               |    |                  |               |      | SPI3_NSS    |              |             |
|               |    |                  |               |      | USART2_CK   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_6  | 26 | MICRO_SPI_SCK    | WICED_GPIO_6  | A  5 | ADC12_IN5   |              |             |
|               |    |                  |               |      | DAC2_OUT    |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | SPI1_SCK    |              |             |
|               |    |                  |               |      | TIM2_CH1_ETR|              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_7  | 25 | MICRO_SPI_MISO   | WICED_GPIO_7  | A  6 | ADC12_IN6   |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | SPI1_MISO   |              |             |
|               |    |                  |               |      | TIM1_BKIN   |              |             |
|               |    |                  |               |      | TIM3_CH1    |              |             |
|               |    |                  |               |      | TIM8_BKIN   |              |             |
|               |    |                  |               |      | TIM13_CH1   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_8  | 24 | MICRO_SPI_MOSI   | WICED_GPIO_8  | A  7 | ADC12_IN7   |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | SPI1_MOSI   |              |             |
|               |    |                  |               |      | TIM1_CH1N   |              |             |
|               |    |                  |               |      | TIM3_CH2    |              |             |
|               |    |                  |               |      | TIM8_CH1N   |              |             |
|               |    |                  |               |      | TIM14_CH1   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_9  | 12 | MICRO_UART_TX    | WICED_GPIO_9  | A  9 | GPIO        |              |             |
|               |    |                  |               |      | I2C3_SMBA   |              |             |
|               |    |                  |               |      | TIM1_CH2    |              |             |
|               |    |                  |               |      | USART1_TX   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_10 | 11 | MICRO_UART_RX    | WICED_GPIO_10 | A 10 | GPIO        |              |             |
|               |    |                  |               |      | TIM1_CH3    |              |             |
|               |    |                  |               |      | USART1_RX   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_11 | 22 | MICRO_GPIO_0     | WICED_GPIO_11 | B  6 | GPIO        | LED D1       |             |
|               |    |                  |               |      | TIM4_CH1    |              | WICED_PWM_1 |
|               |    |                  |               |      | I2C1_SCL    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_12 | 23 | MICRO_GPIO_1     | WICED_GPIO_12 | B  7 | GPIO        | LED D2       |             |
|               |    |                  |               |      | TIM4_CH2    |              | WICED_PWM_2 |
|               |    |                  |               |      | I2C1_SDA    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_13 |  6 | MICRO_JTAG_TMS   | WICED_GPIO_13 | A 13 | GPIO        |              |             |
|               |    |                  |               |      | JTMS-SWDIO  |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_14 |  7 | MICRO_JTAG_TCK   | WICED_GPIO_14 | A 14 | GPIO        |              |             |
|               |    |                  |               |      | JTCK-SWCLK  |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_15 |  8 | MICRO_JTAG_TDI   | WICED_GPIO_15 | A 15 | GPIO        |              |             |
|               |    |                  |               |      | JTDI        |              |             |
|               |    |                  |               |      | I2S3_WS     |              |             |
|               |    |                  |               |      | SPI1_NSS    |              |             |
|               |    |                  |               |      | SPI3_NSS    |              |             |
|               |    |                  |               |      | TIM2_CH1_ETR|              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_16 |  9 | MICRO_JTAG_TDO   | WICED_GPIO_16 | B  3 | GPIO        |              |             |
|               |    |                  |               |      | JTDO        |              |             |
|               |    |                  |               |      | SPI1_SCK    |              |             |
|               |    |                  |               |      | SPI3_SCK    |              |             |
|               |    |                  |               |      | I2S3_SCK    |              |             |
|               |    |                  |               |      | TIM2_CH2    |              |             |
|               |    |                  |               |      | TRACESWO    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_17 | 10 | MICRO_JTAG_TRSTN | WICED_GPIO_17 | B  4 | GPIO        |              |             |
|               |    |                  |               |      | NJTRST      |              |             |
|               |    |                  |               |      | SPI1_MISO   |              |             |
|               |    |                  |               |      | SPI3_MISO   |              |             |
|               |    |                  |               |      | TIM3_CH1    |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
Notes
1. These mappings are defined in <WICED-SDK>/platforms/BCM943362WCD4/platform.c
2. STM32F2xx Datasheet  -> http://www.st.com/web/en/resource/technical/document/datasheet/CD00237391.pdf
3. STM32F2xx Ref Manual -> http://www.st.com/web/en/resource/technical/document/reference_manual/CD00225773.pdf
*/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    WICED_GPIO_1,
    WICED_GPIO_2,
    WICED_GPIO_3,
    WICED_GPIO_4,
    WICED_GPIO_5,
    WICED_GPIO_6,
    WICED_GPIO_7,
    WICED_GPIO_8,
    WICED_GPIO_9,
    WICED_GPIO_10,
    WICED_GPIO_11,
    WICED_GPIO_12,
    WICED_GPIO_13,
    WICED_GPIO_14,
    WICED_GPIO_15,
    WICED_GPIO_16,
    WICED_GPIO_17,
    WICED_GPIO_18,
    WICED_GPIO_19,
    WICED_GPIO_20,
    WICED_GPIO_21,
    WICED_GPIO_22,
    WICED_GPIO_23,
    WICED_GPIO_24,
    WICED_GPIO_25,
    WICED_GPIO_26,
    WICED_GPIO_27,
    WICED_GPIO_28,
    WICED_GPIO_29,
    WICED_GPIO_30,
    WICED_GPIO_31,
    WICED_GPIO_32,
    WICED_GPIO_33,
    WICED_GPIO_34,
    WICED_GPIO_35,
    WICED_GPIO_36,
    WICED_GPIO_37,
    WICED_GPIO_38,
    WICED_GPIO_39,
    WICED_GPIO_40,
    WICED_GPIO_41,
    WICED_GPIO_42,
    WICED_GPIO_43,
    WICED_GPIO_44,
    WICED_GPIO_45,
    WICED_GPIO_46,
    WICED_GPIO_47,
    WICED_GPIO_48,
    WICED_GPIO_49,
    WICED_GPIO_50,
    WICED_GPIO_51,
    WICED_GPIO_52,
    WICED_GPIO_53,
    WICED_GPIO_54,
    WICED_GPIO_55,
    WICED_GPIO_56,
    WICED_GPIO_57,
    WICED_GPIO_58,
    WICED_GPIO_59,
    WICED_GPIO_60,
    WICED_GPIO_61,
    WICED_GPIO_62,
    WICED_GPIO_63,
    WICED_GPIO_64,
    WICED_GPIO_65,
    WICED_GPIO_66,
    WICED_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    WICED_GPIO_32BIT = 0x7FFFFFFF,
} wiced_gpio_t;

//TFL: Deleted SPI, I2C & I2S Data Struct

//TFL: Deleted PWM Data Struct

//TFL: Deleted ADC Data Struct

typedef enum
{
    WICED_UART_0,
    WICED_UART_1,
    WICED_UART_2,
    WICED_UART_3,
    WICED_UART_4,
    WICED_UART_5,
    WICED_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
    WICED_UART_32BIT = 0x7FFFFFFF,
} wiced_uart_t;

typedef enum
{
    PLATFORM_BUTTON_1,
    PLATFORM_BUTTON_2,
    PLATFORM_BUTTON_MAX, /* Denotes the total number of Buttons on the board. Not a valid Button Alias */
} platform_button_t;


/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_PLATFORM_BUTTON_COUNT  ( 2 )

/* UART port used for standard I/O */
#define STDIO_UART ( WICED_UART_0 )

/* Components connected to external I/Os */
#define PLATFORM_LED_COUNT               ( 2 )
#define WICED_LED1         ( WICED_GPIO_37 )
#define WICED_LED2         ( WICED_GPIO_36 )
#define WICED_LED1_ON_STATE  ( WICED_ACTIVE_LOW )
#define WICED_LED2_ON_STATE  ( WICED_ACTIVE_LOW )

#define WICED_BUTTON1      ( WICED_GPIO_45 )
#define WICED_BUTTON2      ( WICED_GPIO_15 )


/* I/O connection <-> Peripheral Connections */
#define WICED_LED1_JOINS_PWM        ( WICED_PWM_1 )
#define WICED_LED2_JOINS_PWM        ( WICED_PWM_2 )
#define WICED_THERMISTOR_JOINS_ADC  ( WICED_ADC_3 )

/* Bootloader OTA and OTA2 factory reset during settings */
#define PLATFORM_FACTORY_RESET_BUTTON_INDEX     ( PLATFORM_BUTTON_1 )
#define PLATFORM_FACTORY_RESET_TIMEOUT          ( 10000 )

/* Generic button checking defines */
#define PLATFORM_BUTTON_PRESS_CHECK_PERIOD      ( 100 )
#define PLATFORM_BUTTON_PRESSED_STATE           (   0 )

#define PLATFORM_GREEN_LED_INDEX                ( WICED_LED_INDEX_1 )
#define PLATFORM_RED_LED_INDEX                  ( WICED_LED_INDEX_2 )

#ifdef __cplusplus
} /*extern "C" */
#endif
