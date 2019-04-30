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
 * STM32F4xx vector table
 */
#include <stdint.h>
#include "platform_cmsis.h"
#include "platform_assert.h"
#include "platform_constants.h"
#include "platform_isr.h"
#include "platform_isr_interface.h"
#include "wwd_rtos_isr.h"
#include "fsl_sai.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef SVC_irq
#error SVC_irq not defined - this will probably cause RTOS to fail to run
#endif

#ifndef PENDSV_irq
#error PENDSV_irq not defined - this will probably cause RTOS to fail to run
#endif

#ifndef SYSTICK_irq
#error SYSTICK_irq not defined - this will probably cause RTOS to fail to run
#endif

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
extern void UnhandledInterrupt( void );
extern void reset_handler     ( void );


/******************************************************
 *               Variable Definitions
 ******************************************************/

#if defined (DEBUG)
#pragma GCC push_options
#pragma GCC optimize ("Og")
#endif // (DEBUG)

#if defined (__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
extern "C" {
    extern void __libc_init_array(void);
}
#endif
#endif

#define WEAK_AV __attribute__ ((weak, section(".after_vectors")))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

/* Pointer to stack location */
extern void* link_stack_end;

PLATFORM_DEFINE_INTERRUPT_VECTOR_TABLE_ARRAY( interrupt_vector_table, PLATFORM_INTERRUPT_VECTOR_TABLE_HAS_VARIABLE_SIZE ) =
{
        (uint32_t)&link_stack_end       , // Initial stack location
        (uint32_t)reset_handler         , // Reset vector
        (uint32_t)NMIException          , // Non Maskable Interrupt
        (uint32_t)HardFaultException    , // Hard Fault interrupt
        (uint32_t)MemManageException    , // Memory Management Fault interrupt
        (uint32_t)BusFaultException     , // Bus Fault interrupt
        (uint32_t)UsageFaultException   , // Usage Fault interrupt
        (uint32_t)0                     , // Reserved
        (uint32_t)0                     , // Reserved
        (uint32_t)0                     , // Reserved
        (uint32_t)0                     , // Reserved
        (uint32_t)SVC_irq               , // SVC interrupt
        (uint32_t)DebugMonitor          , // Debug Monitor interrupt
        (uint32_t)0                     , // Reserved
        (uint32_t)PENDSV_irq            , // PendSV interrupt
        (uint32_t)SYSTICK_irq           , // Sys Tick Interrupt
        // Chip Level - MK24F12
        (uint32_t)DMA0_IRQHandler,                      // 16 : DMA Channel 0 Transfer Complete
        (uint32_t)DMA1_IRQHandler,                      // 17 : DMA Channel 1 Transfer Complete
        (uint32_t)DMA2_IRQHandler,                      // 18 : DMA Channel 2 Transfer Complete
        (uint32_t)DMA3_IRQHandler,                      // 19 : DMA Channel 3 Transfer Complete
        (uint32_t)DMA4_IRQHandler,                      // 20 : DMA Channel 4 Transfer Complete
        (uint32_t)DMA5_IRQHandler,                      // 21 : DMA Channel 5 Transfer Complete
        (uint32_t)DMA6_IRQHandler,                      // 22 : DMA Channel 6 Transfer Complete
        (uint32_t)DMA7_IRQHandler,                      // 23 : DMA Channel 7 Transfer Complete
        (uint32_t)DMA8_IRQHandler,                      // 24 : DMA Channel 8 Transfer Complete
        (uint32_t)DMA9_IRQHandler,                      // 25 : DMA Channel 9 Transfer Complete
        (uint32_t)DMA10_IRQHandler,                     // 26 : DMA Channel 10 Transfer Complete
        (uint32_t)DMA11_IRQHandler,                     // 27 : DMA Channel 11 Transfer Complete
        (uint32_t)DMA12_IRQHandler,                     // 28 : DMA Channel 12 Transfer Complete
        (uint32_t)DMA13_IRQHandler,                     // 29 : DMA Channel 13 Transfer Complete
        (uint32_t)DMA14_IRQHandler,                     // 30 : DMA Channel 14 Transfer Complete
        (uint32_t)DMA15_IRQHandler,                     // 31 : DMA Channel 15 Transfer Complete
        (uint32_t)DMA_Error_IRQHandler,                 // 32 : DMA Error Interrupt
        (uint32_t)MCM_IRQHandler,                       // 33 : Normal Interrupt
        (uint32_t)FTFE_IRQHandler,                      // 34 : FTFE Command complete interrupt
        (uint32_t)Read_Collision_IRQHandler,            // 35 : Read Collision Interrupt
        (uint32_t)LVD_LVW_IRQHandler,                   // 36 : Low Voltage Detect, Low Voltage Warning
        (uint32_t)LLWU_IRQHandler,                      // 37 : Low Leakage Wakeup Unit
        (uint32_t)WDOG_EWM_IRQHandler,                  // 38 : WDOG Interrupt
        (uint32_t)RNG_IRQHandler,                       // 39 : RNG Interrupt
        (uint32_t)I2C0_IRQHandler,                      // 40 : I2C0 interrupt
        (uint32_t)I2C1_IRQHandler,                      // 41 : I2C1 interrupt
        (uint32_t)SPI0_IRQHandler,                      // 42 : SPI0 Interrupt
        (uint32_t)SPI1_IRQHandler,                      // 43 : SPI1 Interrupt
        (uint32_t)I2S0_Tx_IRQHandler,                   // 44 : I2S0 transmit interrupt
        (uint32_t)I2S0_Rx_IRQHandler,                   // 45 : I2S0 receive interrupt
        (uint32_t)UART0_LON_IRQHandler,                 // 46 : UART0 LON interrupt
        (uint32_t)UART0_RX_TX_IRQHandler,               // 47 : UART0 Receive/Transmit interrupt
        (uint32_t) UART0_ERR_IRQHandler,                 // 48 : UART0 Error interrupt
        (uint32_t)UART1_RX_TX_IRQHandler,               // 49 : UART1 Receive/Transmit interrupt
        (uint32_t)UART1_ERR_IRQHandler,                 // 50 : UART1 Error interrupt
        (uint32_t)UART2_RX_TX_IRQHandler,               // 51 : UART2 Receive/Transmit interrupt
        (uint32_t)UART2_ERR_IRQHandler,                 // 52 : UART2 Error interrupt
        (uint32_t)UART3_RX_TX_IRQHandler,               // 53 : UART3 Receive/Transmit interrupt
        (uint32_t)UART3_ERR_IRQHandler,                 // 54 : UART3 Error interrupt
        (uint32_t)ADC0_IRQHandler,                      // 55 : ADC0 interrupt
        (uint32_t)CMP0_IRQHandler,                      // 56 : CMP0 interrupt
        (uint32_t)CMP1_IRQHandler,                      // 57 : CMP1 interrupt
        (uint32_t)FTM0_IRQHandler,                      // 58 : FTM0 fault, overflow and channels interrupt
        (uint32_t)FTM1_IRQHandler,                      // 59 : FTM1 fault, overflow and channels interrupt
        (uint32_t)FTM2_IRQHandler,                      // 60 : FTM2 fault, overflow and channels interrupt
        (uint32_t)CMT_IRQHandler,                       // 61 : CMT interrupt
        (uint32_t)RTC_IRQHandler,                       // 62 : RTC interrupt
        (uint32_t)RTC_Seconds_IRQHandler,               // 63 : RTC seconds interrupt
        (uint32_t)PIT0_IRQHandler,                      // 64 : PIT timer channel 0 interrupt
        (uint32_t)PIT1_IRQHandler,                      // 65 : PIT timer channel 1 interrupt
        (uint32_t)PIT2_IRQHandler,                      // 66 : PIT timer channel 2 interrupt
        (uint32_t)PIT3_IRQHandler,                      // 67 : PIT timer channel 3 interrupt
        (uint32_t)PDB0_IRQHandler,                      // 68 : PDB0 Interrupt
        (uint32_t)USB0_IRQHandler,                      // 69 : USB0 interrupt
        (uint32_t)USBDCD_IRQHandler,                    // 70 : USBDCD Interrupt
        (uint32_t)Reserved71_IRQHandler,                // 71 : Reserved interrupt
        (uint32_t) DAC0_IRQHandler,                      // 72 : DAC0 interrupt
        (uint32_t)MCG_IRQHandler,                       // 73 : MCG Interrupt
        (uint32_t)LPTMR0_IRQHandler,                    // 74 : LPTimer interrupt
        (uint32_t)PORTA_IRQHandler,                     // 75 : Port A interrupt
        (uint32_t)PORTB_IRQHandler,                     // 76 : Port B interrupt
        (uint32_t)PORTC_IRQHandler,                     // 77 : Port C interrupt
        (uint32_t)PORTD_IRQHandler,                     // 78 : Port D interrupt
        (uint32_t)PORTE_IRQHandler,                     // 79 : Port E interrupt
        (uint32_t)SWI_IRQHandler,                       // 80 : Software interrupt
        (uint32_t)SPI2_IRQHandler,                      // 81 : SPI2 Interrupt
        (uint32_t)UART4_RX_TX_IRQHandler,               // 82 : UART4 Receive/Transmit interrupt
        (uint32_t)UART4_ERR_IRQHandler,                 // 83 : UART4 Error interrupt
        (uint32_t)UART5_RX_TX_IRQHandler,               // 84 : UART5 Receive/Transmit interrupt
        (uint32_t) UART5_ERR_IRQHandler,                 // 85 : UART5 Error interrupt
        (uint32_t)CMP2_IRQHandler,                      // 86 : CMP2 interrupt
        (uint32_t)FTM3_IRQHandler,                      // 87 : FTM3 fault, overflow and channels interrupt
        (uint32_t) DAC1_IRQHandler,                      // 88 : DAC1 interrupt
        (uint32_t)ADC1_IRQHandler,                      // 89 : ADC1 interrupt
        (uint32_t)I2C2_IRQHandler,                      // 90 : I2C2 interrupt
        (uint32_t)CAN0_ORed_Message_buffer_IRQHandler,  // 91 : CAN0 OR'd message buffers interrupt
        (uint32_t)CAN0_Bus_Off_IRQHandler,              // 92 : CAN0 bus off interrupt
        (uint32_t)CAN0_Error_IRQHandler,                // 93 : CAN0 error interrupt
        (uint32_t) CAN0_Tx_Warning_IRQHandler,           // 94 : CAN0 Tx warning interrupt
        (uint32_t) CAN0_Rx_Warning_IRQHandler,           // 95 : CAN0 Rx warning interrupt
        (uint32_t) CAN0_Wake_Up_IRQHandler,              // 96 : CAN0 wake up interrupt
        (uint32_t) SDHC_IRQHandler,                     // 97 : SDHC interrupt
};


//*****************************************************************************
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//*****************************************************************************
WEAK_AV void IntDefaultHandler(void)
{ while(1) {}
}


//*****************************************************************************
// Default application exception handlers. Override the ones here by defining
// your own handler routines in your application code. These routines call
// driver exception handlers or IntDefaultHandler() if no driver exception
// handler is included.
//*****************************************************************************

/* IRQS That have been enabled */

WEAK void SDHC_IRQHandler(void){

    while(1);
}


WEAK void I2S0_Tx_IRQHandler(void)
{
    while(1);
}

WEAK void I2S0_Rx_IRQHandler(void)
{
    while(1);
}

/* End of IRQs that have been enabled */

WEAK void DMA0_IRQHandler(void)
{
    //DMA0_DriverIRQHandler();
}

WEAK void DMA1_IRQHandler(void)
{
    //DMA1_DriverIRQHandler();
}

WEAK void DMA2_IRQHandler(void)
{
    //DMA2_DriverIRQHandler();
}

WEAK void DMA3_IRQHandler(void)
{
    //DMA3_DriverIRQHandler();
}

WEAK void DMA4_IRQHandler(void)
{
    //DMA4_DriverIRQHandler();
}

WEAK void DMA5_IRQHandler(void)
{
    //DMA5_DriverIRQHandler();
}


WEAK void DMA6_IRQHandler(void)
{
    //DMA6_DriverIRQHandler();
}

WEAK void DMA7_IRQHandler(void)
{
    //DMA7_DriverIRQHandler();
}

WEAK void DMA8_IRQHandler(void)
{
    //DMA8_DriverIRQHandler();
}

WEAK void DMA9_IRQHandler(void)
{
    //DMA9_DriverIRQHandler();
}

WEAK void DMA10_IRQHandler(void)
{
    //DMA10_DriverIRQHandler();
}

WEAK void DMA11_IRQHandler(void)
{
    //DMA11_DriverIRQHandler();
}

WEAK void DMA12_IRQHandler(void)
{
    //DMA12_DriverIRQHandler();
}

WEAK void DMA13_IRQHandler(void)
{
    //DMA13_DriverIRQHandler();
}

WEAK void DMA14_IRQHandler(void)
{
    // DMA14_DriverIRQHandler();
}

WEAK void DMA15_IRQHandler(void)
{  // DMA15_DriverIRQHandler();
}

WEAK void DMA_Error_IRQHandler(void)
{  // DMA_Error_DriverIRQHandler();
}

WEAK void MCM_IRQHandler(void)
{   //MCM_DriverIRQHandler();
}

WEAK void FTFE_IRQHandler(void)
{   //FTFE_DriverIRQHandler();
}

WEAK void Read_Collision_IRQHandler(void)
{   //Read_Collision_DriverIRQHandler();
}

WEAK void LVD_LVW_IRQHandler(void)
{   //LVD_LVW_DriverIRQHandler();
}

WEAK void LLWU_IRQHandler(void)
{   //LLWU_DriverIRQHandler();
}

WEAK void WDOG_EWM_IRQHandler(void)
{   //WDOG_EWM_DriverIRQHandler();
}

WEAK void RNG_IRQHandler(void)
{   //RNG_DriverIRQHandler();
}

WEAK void I2C0_IRQHandler(void)
{   //I2C0_DriverIRQHandler();
}

WEAK void I2C1_IRQHandler(void)
{   //I2C1_DriverIRQHandler();
}

WEAK void SPI0_IRQHandler(void)
{   //SPI0_DriverIRQHandler();
}

WEAK void SPI1_IRQHandler(void)
{   //SPI1_DriverIRQHandler();
}

WEAK void UART0_LON_IRQHandler(void)
{   //UART0_LON_DriverIRQHandler();
}

WEAK void UART0_RX_TX_IRQHandler(void)
{   //UART0_RX_TX_DriverIRQHandler();
}

WEAK void UART0_ERR_IRQHandler(void)
{   //UART0_ERR_DriverIRQHandler();
}

WEAK void UART1_RX_TX_IRQHandler(void)
{   //UART1_RX_TX_DriverIRQHandler();
}

WEAK void UART1_ERR_IRQHandler(void)
{   //UART1_ERR_DriverIRQHandler();
}

WEAK void UART2_RX_TX_IRQHandler(void)
{   //UART2_RX_TX_DriverIRQHandler();
}

WEAK void UART2_ERR_IRQHandler(void)
{   //UART2_ERR_DriverIRQHandler();
}

WEAK void UART3_RX_TX_IRQHandler(void)
{   //UART3_RX_TX_DriverIRQHandler();
}

WEAK void UART3_ERR_IRQHandler(void)
{  // UART3_ERR_DriverIRQHandler();
}

WEAK void ADC0_IRQHandler(void)
{   //ADC0_DriverIRQHandler();
}

WEAK void CMP0_IRQHandler(void)
{   //CMP0_DriverIRQHandler();
}

WEAK void CMP1_IRQHandler(void)
{   //CMP1_DriverIRQHandler();
}

WEAK void FTM0_IRQHandler(void)
{   //FTM0_DriverIRQHandler();
}

WEAK void FTM1_IRQHandler(void)
{   //FTM1_DriverIRQHandler();
}

WEAK void FTM2_IRQHandler(void)
{   //FTM2_DriverIRQHandler();
}

WEAK void CMT_IRQHandler(void)
{   //CMT_DriverIRQHandler();
}

WEAK void RTC_IRQHandler(void)
{   //RTC_DriverIRQHandler();
}

WEAK void RTC_Seconds_IRQHandler(void)
{   //RTC_Seconds_DriverIRQHandler();
}

WEAK void PIT0_IRQHandler(void)
{   //PIT0_DriverIRQHandler();
}

WEAK void PIT1_IRQHandler(void)
{   //PIT1_DriverIRQHandler();
}

WEAK void PIT2_IRQHandler(void)
{   //PIT2_DriverIRQHandler();
}

WEAK void PIT3_IRQHandler(void)
{   //PIT3_DriverIRQHandler();
}

WEAK void PDB0_IRQHandler(void)
{   //PDB0_DriverIRQHandler();
}

WEAK void USB0_IRQHandler(void)
{   //USB0_DriverIRQHandler();
}

WEAK void USBDCD_IRQHandler(void)
{   //USBDCD_DriverIRQHandler();
}

WEAK void Reserved71_IRQHandler(void)
{   //Reserved71_DriverIRQHandler();
}

WEAK void DAC0_IRQHandler(void)
{   //DAC0_DriverIRQHandler();
}

WEAK void MCG_IRQHandler(void)
{   //MCG_DriverIRQHandler();
}

WEAK void LPTMR0_IRQHandler(void)
{   //LPTMR0_DriverIRQHandler();
}

WEAK void PORTE_IRQHandler(void)
{   //PORTE_DriverIRQHandler();
}

WEAK void SWI_IRQHandler(void)
{   //SWI_DriverIRQHandler();
}

WEAK void SPI2_IRQHandler(void)
{   //SPI2_DriverIRQHandler();
}

WEAK void UART4_RX_TX_IRQHandler(void)
{   //UART4_RX_TX_DriverIRQHandler();
}

WEAK void UART4_ERR_IRQHandler(void)
{   //UART4_ERR_DriverIRQHandler();
}

WEAK void UART5_RX_TX_IRQHandler(void)
{   //UART5_RX_TX_DriverIRQHandler();
}

WEAK void UART5_ERR_IRQHandler(void)
{   //UART5_ERR_DriverIRQHandler();
}

WEAK void CMP2_IRQHandler(void)
{   //CMP2_DriverIRQHandler();
}

WEAK void FTM3_IRQHandler(void)
{   //FTM3_DriverIRQHandler();
}

WEAK void DAC1_IRQHandler(void)
{   //DAC1_DriverIRQHandler();
}

WEAK void ADC1_IRQHandler(void)
{   //ADC1_DriverIRQHandler();
}

WEAK void I2C2_IRQHandler(void)
{   //I2C2_DriverIRQHandler();
}

WEAK void CAN0_ORed_Message_buffer_IRQHandler(void)
{   //CAN0_DriverIRQHandler();
}

WEAK void CAN0_Bus_Off_IRQHandler(void)
{   //CAN0_DriverIRQHandler();
}

WEAK void CAN0_Error_IRQHandler(void)
{   //CAN0_DriverIRQHandler();
}

WEAK void CAN0_Tx_Warning_IRQHandler(void)
{  // CAN0_DriverIRQHandler();
}

WEAK void CAN0_Rx_Warning_IRQHandler(void)
{  // CAN0_DriverIRQHandler();
}

WEAK void CAN0_Wake_Up_IRQHandler(void)
{  // CAN0_DriverIRQHandler();
}

WEAK void ENET_1588_Timer_IRQHandler(void)
{ //  ENET_1588_Timer_DriverIRQHandler();
}

WEAK void ENET_Transmit_IRQHandler(void)
{  // ENET_Transmit_DriverIRQHandler();
}

WEAK void ENET_Receive_IRQHandler(void)
{ //   ENET_Receive_DriverIRQHandler();
}

WEAK void ENET_Error_IRQHandler(void)
{ //  ENET_Error_DriverIRQHandler();
}

//*****************************************************************************

#if defined (DEBUG)
#pragma GCC pop_options
#endif // (DEBUG)
