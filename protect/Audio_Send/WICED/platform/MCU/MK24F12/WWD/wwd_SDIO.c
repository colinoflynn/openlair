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
 * Defines WWD SDIO functions for STM32F4xx MCU
 */
#include <string.h> /* For memcpy */
#include "wwd_platform_common.h"
#include "wwd_bus_protocol.h"
#include "wwd_assert.h"
#include "platform/wwd_platform_interface.h"
#include "platform/wwd_sdio_interface.h"
#include "platform/wwd_bus_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "network/wwd_network_constants.h"
#include "platform_cmsis.h"
#include "platform_peripheral.h"
#include "platform_config.h"
#include "wwd_rtos_isr.h"
#include "fsl_sdhc.h"
#include "fsl_common.h"
#include "MK24F12.h"
#include "fsl_clock.h"
#include "fsl_sdhc.h"
#include "fsl_sysmpu.h"
#include "pin_mux.h"
#include "clock_config.h"

/******************************************************
 *             Constants
 ******************************************************/

#define COMMAND_FINISHED_CMD52_TIMEOUT_LOOPS (100000)
#define COMMAND_FINISHED_CMD53_TIMEOUT_LOOPS (100000)
#define SDIO_TX_RX_COMPLETE_TIMEOUT_LOOPS    (100000)
#define SDIO_DMA_TIMEOUT_LOOPS               (1000000)
#define MAX_TIMEOUTS                         (30)
#define SDIO_IRQ_CHANNEL                     ((u8)0x31)
#define DMA2_3_IRQ_CHANNEL                   ((u8)DMA2_Stream3_IRQn)
#define BUS_LEVEL_MAX_RETRIES                (5)
#define SDIO_ENUMERATION_TIMEOUT_MS          (500)

//#define SDIO_1_BIT

/******************************************************
 *             Structures
 ******************************************************/

typedef struct
{
    /*@shared@*/ /*@null@*/ uint8_t* data;
    uint16_t length;
} sdio_dma_segment_t;

/******************************************************
 *             Variables
 ******************************************************/

static const uint32_t bus_direction_mapping[] =
{
    [BUS_READ]  = 0,
    [BUS_WRITE] = 1
};

ALIGNED_PRE(4) static uint8_t       temp_dma_buffer[MAX(2*1024,WICED_LINK_MTU+ 64)] ALIGNED(4);
static uint8_t*                     user_data;
static uint32_t                     user_data_size;
static uint8_t*                     dma_data_source;
static uint32_t                     dma_transfer_size;
static host_semaphore_type_t        sdio_transfer_finished_semaphore;
static wiced_bool_t                 sdio_transfer_failed;
static wwd_bus_transfer_direction_t current_transfer_direction;
static uint32_t                     current_command;

/******************************************************
 *             Static Function Declarations
 ******************************************************/

static uint32_t          sdio_get_blocksize_dctrl   ( sdio_block_size_t block_size );
static sdio_block_size_t find_optimal_block_size    ( uint32_t data_size );
static void              sdio_prepare_data_transfer ( wwd_bus_transfer_direction_t direction, sdio_block_size_t block_size, /*@unique@*/ uint8_t* data, uint16_t data_size ) /*@modifies dma_data_source, user_data, user_data_size, dma_transfer_size@*/;

/******************************************************
 *             Function definitions
 ******************************************************/

#ifndef  WICED_DISABLE_MCU_POWERSAVE
static void sdio_oob_irq_handler( void* arg )
{
    //Fix this Var increment

    UNUSED_PARAMETER(arg);
    WWD_BUS_STATS_INCREMENT_VARIABLE(error_intrs );
    platform_mcu_powersave_exit_notify( );
    wwd_thread_notify_irq( );
}
#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */

static void sdio_enable_bus_irq( void )
{

    SDHC_EnableInterruptStatus(SDHC, kSDHC_CardInterruptFlag);
    SDHC_EnableInterruptSignal(SDHC, kSDHC_CardInterruptFlag);
}

static void sdio_disable_bus_irq( void )
{
    SDHC_DisableInterruptStatus(SDHC, kSDHC_CardInterruptFlag);
    SDHC_DisableInterruptSignal(SDHC, kSDHC_CardInterruptFlag);

}

#ifndef WICED_DISABLE_MCU_POWERSAVE
wwd_result_t host_enable_oob_interrupt( void )
{
    /* Set GPIO_B[1:0] to input. One of them will be re-purposed as OOB interrupt */
    platform_gpio_init( &wifi_sdio_pins[WWD_PIN_SDIO_OOB_IRQ], INPUT_HIGH_IMPEDANCE );
    platform_gpio_irq_enable( &wifi_sdio_pins[WWD_PIN_SDIO_OOB_IRQ], IRQ_TRIGGER_RISING_EDGE, sdio_oob_irq_handler, 0 );
    return WWD_SUCCESS;
}

uint8_t host_platform_get_oob_interrupt_pin( void )
{
    return WICED_WIFI_OOB_IRQ_GPIO_PIN;
}
#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */

wwd_result_t host_platform_bus_init( void )
{
    wwd_result_t     result;
    /* Initializing transfer sema */
    result = host_rtos_init_semaphore( &sdio_transfer_finished_semaphore );
    if ( result != WWD_SUCCESS )
    {
        return result;
    }
    /* Enabling Bus interrupts to clear sema */
    sdio_enable_bus_irq();
    NVIC_EnableIRQ( SDHC_IRQn );
    return WICED_SUCCESS;
}

wwd_result_t host_platform_sdio_enumerate( void )
{
    wwd_result_t result;
    uint32_t       loop_count;
    uint32_t       data = 0;

    loop_count = 0;
    do
    {
        /* Send CMD0 to set it to idle state */
        SDHC_SetCardActive(SDHC,1000);
        result = (wwd_result_t)host_platform_sdio_transfer( BUS_WRITE, SDIO_CMD_0, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, NO_RESPONSE, NULL );
        /* CMD5. */
        result = (wwd_result_t)host_platform_sdio_transfer( BUS_READ, SDIO_CMD_5, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, NO_RESPONSE, NULL );

        /* Send CMD3 to get RCA. */
        result = (wwd_result_t)host_platform_sdio_transfer( BUS_READ, SDIO_CMD_3, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, RESPONSE_NEEDED, &data );
        loop_count++;
        if ( loop_count >= (uint32_t) SDIO_ENUMERATION_TIMEOUT_MS )
        {
            return WWD_TIMEOUT;
        }
    } while ( ( result != WWD_SUCCESS ) && ( host_rtos_delay_milliseconds( (uint32_t) 1 ), ( 1 == 1 ) ) );
    /* If you're stuck here, check the platform matches your hardware */

    /* Send CMD7 with the returned RCA to select the card */
    host_platform_sdio_transfer( BUS_WRITE, SDIO_CMD_7, SDIO_BYTE_MODE, SDIO_1B_BLOCK, data, 0, 0, RESPONSE_NEEDED, NULL );

    return WWD_SUCCESS;
}

wwd_result_t host_platform_bus_deinit( void )
{
    wwd_result_t result;
    uint32_t     a;
    /* Deinitializing sema */

    result = host_rtos_deinit_semaphore( &sdio_transfer_finished_semaphore );

    platform_mcu_powersave_disable();

    /* Disabling SDIO and OOB IRQ */
    sdio_disable_bus_irq( );

    for ( a = 0; a < WWD_PIN_SDIO_MAX; a++ )
    {
        platform_gpio_deinit( &wifi_sdio_pins[ a ] );
    }

#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_0
    platform_gpio_deinit( &wifi_control_pins[WWD_PIN_BOOTSTRAP_0] );
#endif
#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_1
    platform_gpio_deinit( &wifi_control_pins[WWD_PIN_BOOTSTRAP_1] );
#endif



    platform_mcu_powersave_enable();

    return result;
}

//#define NO_TOUCHY ((1<<2) | (1<<3) | (1<<16) | (1<<23))

wwd_result_t host_platform_sdio_transfer( wwd_bus_transfer_direction_t direction, sdio_command_t command, sdio_transfer_mode_t mode, sdio_block_size_t block_size, uint32_t argument, /*@null@*/ uint32_t* data, uint16_t data_size, sdio_response_needed_t response_expected, /*@out@*/ /*@null@*/  uint32_t* response )
{
    uint32_t mask;
    wwd_result_t result; /* Wiced Status var */
    status_t k_status; /* MK64 Status var */

    /*Initializing to zero to prevent any unwanted options to be set */
    sdhc_command_t k_cmd  = {0}; /* MK64 command struct */
    sdhc_data_t k_data = {0}; /* MK64 data struct */
    sdhc_transfer_t k_tx = {0}; /* MK64 Transmisison struct */

    /* Setting as NULL for all CMD's except 53 */
    k_tx.data = NULL;

    /* Grabbing block count from count in argument */
    k_data.blockCount =  (uint32_t) argument & 0x1FF;
    //Choosing Response type based on command
    switch(command){
        case SDIO_CMD_0:
            k_cmd.responseType = kCARD_ResponseTypeNone; break;
        case SDIO_CMD_3:
            k_cmd.responseType = kCARD_ResponseTypeR6; break;
        case SDIO_CMD_5:
            k_cmd.responseType = kCARD_ResponseTypeR4; break;
        case SDIO_CMD_7:
            k_cmd.responseType = kCARD_ResponseTypeR1; break;
        case SDIO_CMD_52:
            k_cmd.responseType = kCARD_ResponseTypeR5; break;
        case SDIO_CMD_53:
            k_cmd.responseType = kCARD_ResponseTypeR5;

            /* For CMD53 we are either sending 64 Byte blocks as what has been set in the SDIO card
             * or we are in byte mode, this next bit of code is to deal with both cases
             */

            if(mode == SDIO_BLOCK_MODE){
                k_data.blockSize = 64;
                k_data.blockCount =  (uint32_t) argument & 0x1FF;
            }else{
                block_size = find_optimal_block_size( data_size );
                /* Hack to ensure all blocks are aligned */
                  if ( block_size < SDIO_512B_BLOCK )
                  {
                      argument = ( argument & (uint32_t) ( ~0x1FF ) ) | block_size;
                  }
                  else
                  {
                      argument = ( argument & (uint32_t) ( ~0x1FF ) );
                  }
                  k_data.blockSize =  (uint32_t) argument & 0x1FF;
                  k_data.blockCount = 1;
            }

            /* Setting data pointer for when reading and writing */
            if(direction == BUS_READ){

                  k_data.rxData = data;
                  k_data.txData = NULL;
            }else{
                k_data.txData = data;
                k_data.rxData = NULL;

            }

            /*Linking Data struct to Tx struct for DAT Line transmission if not in CMD53
             * should be null to prevent unwanted transfer */
            k_tx.data = &k_data;
            break;
        case __MAX_VAL:
            k_cmd.responseType =  kCARD_ResponseTypeNone; break;
        default: break;
   }
    k_cmd.index = command;
    k_cmd.argument = argument;
    k_cmd.type = kCARD_CommandTypeNormal;
    k_tx.command = &k_cmd;

    /* hacky way to get the sdhc transfer to work on Nest */
    //GPIOA->PDDR = 0x00000000;

   // GPIOC->PDDR = 0x00061401;
   // GPIOC->PSOR = 0x00061001;

   // GPIOD->PDDR = 0x95;
   // GPIOD->PDOR = 0x1;
    uint32_t NO_TOUCHY = ((1<<2) | (1<<3) | (1<<16) | (1<<23));
    GPIOB->PDDR = (0x00A50402 & ~NO_TOUCHY) | (GPIOB->PDDR & NO_TOUCHY);
    GPIOB->PSOR=  (0x00A00002 & ~NO_TOUCHY) | (GPIOB->PDOR & NO_TOUCHY);


    /* Sending command and receiving response */
    k_status = SDHC_TransferBlocking(SDHC,NULL,0x00, &k_tx);

/*
    GPIOA->PDDR = gpioa.PDDR;

    GPIOB->PDDR = gpiob.PDDR;
    GPIOB->PDOR=  gpiob.PDOR;

    GPIOC->PDDR = gpioc.PDDR;
    GPIOC->PDOR = gpioc.PDOR;

    GPIOD->PDDR = gpioe.PDDR;
    GPIOD->PDOR = gpioe.PDOR;*/
    /* End of hack method */


    //Re enabling card interrupts after command;

    SDHC_EnableInterruptSignal(SDHC, kSDHC_CardInterruptFlag);

    //Any additional INIT based on CMD type
    if((direction == BUS_READ) && (response != NULL)){
       *response = k_tx.command->response[0];
    }

    //Any additional requirements needed based on command

    switch(command){
        case SDIO_CMD_0:
            break;
        case SDIO_CMD_3:
            //Referencing response
             *response = *response & (0xFFFF<<15);
             break;
        case SDIO_CMD_5:
            //Masking Response and re-sending as working voltage
              mask = 0x00FFFF00 & k_tx.command->response[0];
              k_tx.command->argument = mask;
              k_status = SDHC_TransferBlocking(SDHC,NULL,0x00, &k_tx);
              break;
        case SDIO_CMD_7:
              break;
        case SDIO_CMD_52:
              break;
        case SDIO_CMD_53:
              break;
        case __MAX_VAL:
              break;
        default: break;
   }

    //Result for WICED Status

    switch(k_status){
        case kStatus_Success : result = 0x00; break;
        case kStatus_Fail : result = 10; break;
        case kStatus_ReadOnly : result = 1030; break;
        case kStatus_OutOfRange : result = 1012; break;
        case kStatus_InvalidArgument: result = 0x05; break;
        case kStatus_Timeout: result = 0x02; break;
        case kStatus_NoTransferInProgress: result = 1; break;
        default: result = 10; break;
    }

exit:
    platform_mcu_powersave_enable();

    return result;
}

static void sdio_prepare_data_transfer( wwd_bus_transfer_direction_t direction, sdio_block_size_t block_size, /*@unique@*/ uint8_t* data, uint16_t data_size ) /*@modifies dma_data_source, user_data, user_data_size, dma_transfer_size@*/
{
    /* Setup a single transfer using the temp buffer */
    user_data         = data;
    user_data_size    = data_size;
    dma_transfer_size = (uint32_t) ( ( ( data_size + (uint16_t) block_size - 1 ) / (uint16_t) block_size ) * (uint16_t) block_size );

    if ( direction == BUS_WRITE )
    {
        dma_data_source = data;
    }
    else
    {
        dma_data_source = temp_dma_buffer;
    }

}

wwd_result_t host_platform_enable_high_speed_sdio( void )
{
    uint32_t src_clk;
    //
    src_clk = CLOCK_GetCoreSysClkFreq();
    #ifndef SDIO_1_BIT
    SDHC_SetDataBusWidth(SDHC,kSDHC_DataBusWidth4Bit);
    #else
    SDHC_SetDataBusWidth(SDHC,kSDHC_DataBusWidth1Bit);
    #endif
    SDHC_SetSdClock(SDHC,src_clk,25000000);
    return WWD_SUCCESS;
}

static sdio_block_size_t find_optimal_block_size( uint32_t data_size )
{
    if ( data_size > (uint32_t) 256 )
        return SDIO_512B_BLOCK;
    if ( data_size > (uint32_t) 128 )
        return SDIO_256B_BLOCK;
    if ( data_size > (uint32_t) 64 )
        return SDIO_128B_BLOCK;
    if ( data_size > (uint32_t) 32 )
        return SDIO_64B_BLOCK;
    if ( data_size > (uint32_t) 16 )
        return SDIO_32B_BLOCK;
    if ( data_size > (uint32_t) 8 )
        return SDIO_16B_BLOCK;
    if ( data_size > (uint32_t) 4 )
        return SDIO_8B_BLOCK;
    if ( data_size > (uint32_t) 2 )
        return SDIO_4B_BLOCK;

    return SDIO_4B_BLOCK;
}

static uint32_t sdio_get_blocksize_dctrl(sdio_block_size_t block_size)
{
    return 0;
}

wwd_result_t host_platform_bus_enable_interrupt( void )
{
    sdio_enable_bus_irq();
    return  WWD_SUCCESS;
}

wwd_result_t host_platform_bus_disable_interrupt( void )
{
    return  WWD_SUCCESS;
}

void host_platform_bus_buffer_freed( wwd_buffer_dir_t direction )
{
    UNUSED_PARAMETER( direction );
}

/******************************************************
 *             IRQ Handler Definitions
 ******************************************************/

WWD_RTOS_DEFINE_ISR( sdio_irq )
{
    uint32_t intstatus = SDHC->IRQSTAT;

    /*  Incrementing Debug Stats setting transfer semaphore */
    WWD_BUS_STATS_INCREMENT_VARIABLE( sdio_intrs );
    if ( ( intstatus & ( kSDHC_DataErrorFlag |  kSDHC_CommandErrorFlag )) != 0 )
       {

           WWD_BUS_STATS_INCREMENT_VARIABLE( error_intrs );
           sdio_transfer_failed = WICED_TRUE;
           host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WICED_TRUE );
       }
       else
       {
           if ((intstatus & (kSDHC_CommandCompleteFlag|kSDHC_DataCompleteFlag)) != 0)
           {

               if ( ( SDHC->CMDRSP[0] & 0x800 ) != 0 )
               {
                   sdio_transfer_failed = WICED_TRUE;
                   host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WICED_TRUE );
               }
         }

        /* Clear all command/response interrupts */

    if ( ( intstatus & SDHC_IRQSIGEN_CINTIEN_MASK ) != 0 )
    {
        /* Clear the interrupt and then inform WICED thread */
        SDHC->IRQSIGEN = 0x00;
        SDHC_DisableInterruptSignal(SDHC, kSDHC_CardInterruptFlag);
        wwd_thread_notify_irq( );
        }
    }
}

/******************************************************
 *             IRQ Handler Mapping
 ******************************************************/

    WWD_RTOS_MAP_ISR( sdio_irq,     SDHC_IRQHandler     )
