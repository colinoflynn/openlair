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
 * Declares ISR prototypes for MK64 MCU family
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/



#if defined (DEBUG)
#pragma GCC push_options
#pragma GCC optimize ("O0")
#endif // (DEBUG)


//*****************************************************************************
// Forward declaration of the core exception handlers.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions
//*****************************************************************************
extern void NMIException           ( void );  // Non Maskable Interrupt
extern void HardFaultException     ( void );  // Hard Fault interrupt
extern void MemManageException     ( void );  // Memory Management Fault interrupt
extern void BusFaultException      ( void );  // Bus Fault interrupt
extern void UsageFaultException    ( void );  // Usage Fault interrupt
extern void SVC_irq                ( void );  // SVC interrupt
extern void DebugMonitor           ( void );  // Debug Monitor interrupt
extern void PENDSV_irq             ( void );  // PendSV interrupt
extern void SYSTICK_irq            ( void );  // Sys Tick Interrupt



//*****************************************************************************
// Forward declaration of the application IRQ handlers. When the application
// defines a handler (with the same name), this will automatically take
// precedence over weak definitions below
//*****************************************************************************
WEAK void DMA0_IRQHandler(void);
WEAK void DMA1_IRQHandler(void);
WEAK void DMA2_IRQHandler(void);
WEAK void DMA3_IRQHandler(void);
WEAK void DMA4_IRQHandler(void);
WEAK void DMA5_IRQHandler(void);
WEAK void DMA6_IRQHandler(void);
WEAK void DMA7_IRQHandler(void);
WEAK void DMA8_IRQHandler(void);
WEAK void DMA9_IRQHandler(void);
WEAK void DMA10_IRQHandler(void);
WEAK void DMA11_IRQHandler(void);
WEAK void DMA12_IRQHandler(void);
WEAK void DMA13_IRQHandler(void);
WEAK void DMA14_IRQHandler(void);
WEAK void DMA15_IRQHandler(void);
WEAK void DMA_Error_IRQHandler(void);
WEAK void MCM_IRQHandler(void);
WEAK void FTFE_IRQHandler(void);
WEAK void Read_Collision_IRQHandler(void);
WEAK void LVD_LVW_IRQHandler(void);
WEAK void LLWU_IRQHandler(void);
WEAK void WDOG_EWM_IRQHandler(void);
WEAK void RNG_IRQHandler(void);
WEAK void I2C0_IRQHandler(void);
WEAK void I2C1_IRQHandler(void);
WEAK void SPI0_IRQHandler(void);
WEAK void SPI1_IRQHandler(void);
WEAK void I2S0_Tx_IRQHandler(void);
WEAK void I2S0_Rx_IRQHandler(void);
WEAK void UART0_LON_IRQHandler(void);
WEAK void UART0_RX_TX_IRQHandler(void);
WEAK void UART0_ERR_IRQHandler(void);
WEAK void UART1_RX_TX_IRQHandler(void);
WEAK void UART1_ERR_IRQHandler(void);
WEAK void UART2_RX_TX_IRQHandler(void);
WEAK void UART2_ERR_IRQHandler(void);
WEAK void UART3_RX_TX_IRQHandler(void);
WEAK void UART3_ERR_IRQHandler(void);
WEAK void ADC0_IRQHandler(void);
WEAK void CMP0_IRQHandler(void);
WEAK void CMP1_IRQHandler(void);
WEAK void FTM0_IRQHandler(void);
WEAK void FTM1_IRQHandler(void);
WEAK void FTM2_IRQHandler(void);
WEAK void CMT_IRQHandler(void);
WEAK void RTC_IRQHandler(void);
WEAK void RTC_Seconds_IRQHandler(void);
WEAK void PIT0_IRQHandler(void);
WEAK void PIT1_IRQHandler(void);
WEAK void PIT2_IRQHandler(void);
WEAK void PIT3_IRQHandler(void);
WEAK void PDB0_IRQHandler(void);
WEAK void USB0_IRQHandler(void);
WEAK void USBDCD_IRQHandler(void);
WEAK void Reserved71_IRQHandler(void);
WEAK void DAC0_IRQHandler(void);
WEAK void MCG_IRQHandler(void);
WEAK void LPTMR0_IRQHandler(void);
WEAK void PORTA_IRQHandler(void);
WEAK void PORTB_IRQHandler(void);
WEAK void PORTC_IRQHandler(void);
WEAK void PORTD_IRQHandler(void);
WEAK void PORTE_IRQHandler(void);
WEAK void SWI_IRQHandler(void);
WEAK void SPI2_IRQHandler(void);
WEAK void UART4_RX_TX_IRQHandler(void);
WEAK void UART4_ERR_IRQHandler(void);
WEAK void UART5_RX_TX_IRQHandler(void);
WEAK void UART5_ERR_IRQHandler(void);
WEAK void CMP2_IRQHandler(void);
WEAK void FTM3_IRQHandler(void);
WEAK void DAC1_IRQHandler(void);
WEAK void ADC1_IRQHandler(void);
WEAK void I2C2_IRQHandler(void);
WEAK void CAN0_ORed_Message_buffer_IRQHandler(void);
WEAK void CAN0_Bus_Off_IRQHandler(void);
WEAK void CAN0_Error_IRQHandler(void);
WEAK void CAN0_Tx_Warning_IRQHandler(void);
WEAK void CAN0_Rx_Warning_IRQHandler(void);
WEAK void CAN0_Wake_Up_IRQHandler(void);
WEAK void SDHC_IRQHandler(void);
WEAK void ENET_1588_Timer_IRQHandler(void);
WEAK void ENET_Transmit_IRQHandler(void);
WEAK void ENET_Receive_IRQHandler(void);
WEAK void ENET_Error_IRQHandler(void);



#if defined (DEBUG)
#pragma GCC pop_options
#endif // (DEBUG)

#ifdef __cplusplus
} /* extern "C" */
#endif

