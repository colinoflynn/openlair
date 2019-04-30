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
 * MK24F12 UART implementation
 */
#include <stdint.h>
#include <string.h>
#include "platform_config.h"
#include "platform_peripheral.h"
#include "platform_sleep.h"
#include "platform_assert.h"
#include "platform_gpio.c"
#include "wwd_assert.h"
#include "fsl_uart.h"
#include "clock_config.h"
#include "fsl_port.h"
#include "../../../../libraries/test/rtt/SEGGER_RTT.c"
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/


#define UART_MAX_TRANSMIT_WAIT_TIME_MS  (10)

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

static platform_result_t receive_bytes       ( platform_uart_driver_t* driver, void* data, uint32_t size, uint32_t timeout );


/******************************************************
 *               Variable Definitions
 ******************************************************/


enum usart_alternate { GPIO_AF_UART0,GPIO_AF_UART1,GPIO_AF_UART2,GPIO_AF_UART3,GPIO_AF_UART4,GPIO_AF_UART5 };
/* UART alternate functions */
static const uint8_t uart_alternate_functions[NUMBER_OF_UART_PORTS] =
{
    [0] = GPIO_AF_UART0,
    [1] = GPIO_AF_UART1,
    [2] = GPIO_AF_UART2,
    [3] = GPIO_AF_UART3,
    [4] = GPIO_AF_UART4,
    [5] = GPIO_AF_UART5,
};


/******************************************************
 *               Function Definitions
 ******************************************************/

platform_result_t platform_uart_init( platform_uart_driver_t* driver, const platform_uart_t* peripheral, const platform_uart_config_t* config, wiced_ring_buffer_t* optional_ring_buffer )
{

    SEGGER_RTT_Init();

    uart_config_t uartconfig = {0}; /*MK64 Config Struct */
    port_pin_config_t port_pin_config = {0}; /* MK64 Port Pin Config */
    uint32_t          uart_number;

    wiced_assert( "bad argument", ( driver != NULL ) && ( peripheral != NULL ) && ( config != NULL ) );
    wiced_assert( "Bad ring buffer", (optional_ring_buffer == NULL) || ((optional_ring_buffer->buffer != NULL ) && (optional_ring_buffer->size != 0)) );
    platform_mcu_powersave_disable();
    uart_number = platform_uart_get_port_number( peripheral->port ); /* Retriving port */

    /* We can send an 8 bit message max with parity */
    driver->rx_size              = 0x08;
    driver->tx_size              = 0x08;
    driver->last_transmit_result = PLATFORM_SUCCESS;
    driver->last_receive_result  = PLATFORM_SUCCESS;
    driver->peripheral           = (platform_uart_t*)peripheral;

    /* Semaphores for uart driver */
    host_rtos_init_semaphore( &driver->tx_complete );
    host_rtos_init_semaphore( &driver->rx_complete );

    /* Configure TX and RX pin configuration*/
    port_pin_config.openDrainEnable =  kPORT_OpenDrainDisable;
    port_pin_config.pullSelect = kPORT_PullDisable;

    /* Config for Port based on UART */
    switch(uart_number){
        case GPIO_AF_UART0:
            port_pin_config.mux = kPORT_MuxAlt3;
            break;
        case GPIO_AF_UART1:
            port_pin_config.mux = kPORT_MuxAlt3;
            break;
        case GPIO_AF_UART2:
            port_pin_config.mux = kPORT_MuxAlt3;
            break;
        case GPIO_AF_UART3:
            port_pin_config.mux = kPORT_MuxAlt3;
            break;
        case GPIO_AF_UART4:
            port_pin_config.mux = kPORT_MuxAlt3;
            break;
        case GPIO_AF_UART5:
            port_pin_config.mux = kPORT_MuxAlt3;
            break;
        default: break;
   }

    /* Setting the alt functions for tx and rx pins */
    platform_gpio_set_alternate_function( peripheral->tx_pin->port, peripheral->tx_pin->pin_number, &port_pin_config );
    platform_gpio_set_alternate_function( peripheral->rx_pin->port, peripheral->rx_pin->pin_number, &port_pin_config );

    if ( ( peripheral->cts_pin != NULL ) && ( config->flow_control == FLOW_CONTROL_CTS || config->flow_control == FLOW_CONTROL_CTS_RTS ) )
    {
        platform_gpio_set_alternate_function( peripheral->cts_pin->port, peripheral->cts_pin->pin_number, &port_pin_config );
    }

    if ( ( peripheral->rts_pin != NULL ) && ( config->flow_control == FLOW_CONTROL_RTS || config->flow_control == FLOW_CONTROL_CTS_RTS ) )
    {
        platform_gpio_set_alternate_function( peripheral->rts_pin->port, peripheral->rts_pin->pin_number, &port_pin_config);
    }

    /* Configuring Uart Config Structure */
    uartconfig.enableRx = 0x01;
    uartconfig.enableTx = 0x01;
    uartconfig.baudRate_Bps = config->baud_rate;
    uartconfig.txFifoWatermark = 8;
    uartconfig.rxFifoWatermark = 8;
    uartconfig.stopBitCount   = ( config->stop_bits == STOP_BITS_1 ) ? kUART_OneStopBit : kUART_TwoStopBit;

    /*Configuring parity*/
    switch ( config->parity )
    {
        case NO_PARITY:
            uartconfig.parityMode = kUART_ParityDisabled;
            break;

        case EVEN_PARITY:
            uartconfig.parityMode = kUART_ParityEven;
            break;

        case ODD_PARITY:
            uartconfig.parityMode = kUART_ParityOdd;
            break;

        default:
            return PLATFORM_BADARG;
    }


    /* Initialise USART peripheral */
    UART_Init(driver->peripheral->port,&uartconfig,SystemCoreClock);


    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

platform_result_t platform_uart_deinit( platform_uart_driver_t* driver )
{
    uint8_t          uart_number;

    wiced_assert( "bad argument", ( driver != NULL ) );

    platform_mcu_powersave_disable();

    uart_number = platform_uart_get_port_number( driver->peripheral->port );


    /* Deinitialise USART */
    UART_Deinit(driver->peripheral->port);

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

platform_result_t platform_uart_transmit_bytes( platform_uart_driver_t* driver, const uint8_t* data_out, uint32_t size )
{
    wiced_assert( "bad argument", ( driver != NULL ) && ( data_out != NULL ) && ( size != 0 ) );

    platform_mcu_powersave_disable();

    /* Wait for transmission complete */
    host_rtos_get_semaphore( &driver->tx_complete, NEVER_TIMEOUT, WICED_TRUE );
    /*Transmitting data over UART (Non interrupt) */
    UART_WriteBlocking(driver->peripheral->port, data_out,(size_t)size);

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

platform_result_t platform_uart_exception_transmit_bytes( platform_uart_driver_t* driver, const uint8_t* data_out, uint32_t size )
{

    return PLATFORM_SUCCESS;
}

platform_result_t platform_uart_receive_bytes( platform_uart_driver_t* driver, uint8_t* data_in, uint32_t* expected_data_size, uint32_t timeout_ms )
{
    platform_result_t result = PLATFORM_SUCCESS;
    uint32_t bytes_left      = 0;
    status_t mystatus;
    wiced_assert( "bad argument", ( driver != NULL ) && ( data_in != NULL ) && ( expected_data_size != NULL ) && ( *expected_data_size != 0 ) );

    /* Recieving the bytes expected*/
    bytes_left = *expected_data_size;

    /* Receving data over UART (non interrupt) */
    mystatus = UART_ReadBlocking(driver->peripheral->port, data_in,(size_t)expected_data_size);
    if(mystatus == kStatus_Success){
    return result;
    }else{
    result = PLATFORM_ERROR;
    return result;
    }
}

uint8_t platform_uart_get_port_number( UART_Type* uart )
{    if ( uart == UART0 )
    {
        return 0;
    }
    else if ( uart == UART1 )
    {
        return 1;
    }
    else if ( uart == UART2 )
    {
        return 2;
    }
    else if ( uart == UART3 )
    {
        return 3;
    }
    else if ( uart == UART4 )
    {
        return 4;
    }
    else if ( uart == UART5 )
    {
        return 5;
    }
    else
    {
        return INVALID_UART_PORT_NUMBER;
    }
}




/******************************************************
 *            IRQ Handlers Definition
 ******************************************************/

void platform_uart_irq( platform_uart_driver_t* driver )
{
   /* We arent using interrupts so we dont need IRQ's*/ ;
}

void platform_uart_tx_dma_irq( platform_uart_driver_t* driver )
{
    /* We arent using interrupts so we dont need IRQ's*/ ;
}

void platform_uart_rx_dma_irq( platform_uart_driver_t* driver )
{
    /* We arent using interrupts so we dont need IRQ's*/  ;
}

platform_result_t platform_uart_powersave_wakeup_handler ( const platform_uart_t* peripheral )
{
    /* We arent using interrupts so we dont need IRQ's*/
    return PLATFORM_SUCCESS;
}

platform_result_t platform_uart_powersave_sleep_handler ( const platform_uart_t* peripheral )
{
    /* We arent using interrupts so we dont need IRQ's*/
    return PLATFORM_SUCCESS;
}
