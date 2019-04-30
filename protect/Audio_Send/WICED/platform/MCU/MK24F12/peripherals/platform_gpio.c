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
 * MK24F12 common GPIO implementation
 */
#include "stdint.h"
#include "string.h"
#include "platform_peripheral.h"
#include "platform_isr.h"
#include "platform_isr_interface.h"
#include "wwd_rtos_isr.h"
#include "wwd_assert.h"
#include "pin_mux.h"
#include "fsl_port.h"
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define PIN_OUTPUT_DRAIN_ENABLE     (1<<5)
#define PIN_HIGH_DRIVE_ENABLE       (1<<6)
#define PIN_PULL_ENABLE             (1<<1)
#define PIN_PULL_DISABLE           ~(1<<1)
#define PIN_PULL_UP                 (0x01)
#define PIN_PULL_DOWN               (0x00)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/* Structure of runtime GPIO IRQ data */
typedef struct
{
    platform_gpio_port_t*        owner_port; // GPIO port owning the IRQ line (line is shared across all GPIO ports)
    platform_gpio_irq_callback_t handler;    // User callback
    void*                        arg;        // User argument to be passed to the callbackA
} platform_gpio_irq_data_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* GPIO peripheral clocks */
static const uint32_t gpio_peripheral_clocks[NUMBER_OF_GPIO_PORTS] =
{
    [0] =  kCLOCK_PortA,
    [1] =  kCLOCK_PortB,
    [2] =  kCLOCK_PortC,
    [3] =  kCLOCK_PortD,
    [4] =  kCLOCK_PortE,
};

/* Runtime GPIO IRQ data */
static volatile platform_gpio_irq_data_t gpio_irq_data[NUMBER_OF_GPIO_IRQ_LINES];

PORT_Type* My_Port;

/******************************************************
 *            Platform Function Definitions
 ******************************************************/

platform_result_t platform_gpio_init( const platform_gpio_t* gpio, platform_pin_config_t config )
{
    /* Initializing MK64 config struct */
    gpio_pin_config_t gpio_config = {0};
    port_pin_config_t port_pin_config;

    /* Getting base address of GPIO being used and mapping to correct port */
    uint32_t mygpio =  (uint32_t)&(*(gpio->port));
    if(mygpio == GPIOA_BASE){
        My_Port = PORTA;
    }else if(mygpio == GPIOB_BASE ) {
        My_Port = PORTB;
    }else if( mygpio == GPIOC_BASE){
        My_Port = PORTC;
    }else if(mygpio == GPIOD_BASE ){
        My_Port = PORTD;
    }else if(mygpio == GPIOE_BASE){
        My_Port = PORTE;
    }else{

    }
    /* Enabling the clocks for all GPIO's*/
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);


    /*Configuring pin as GPIO */
    port_pin_config.mux = kPORT_MuxAsGpio;

    /* Configuring pullup/down and other features */
    switch(config){
    case INPUT_PULL_UP:   port_pin_config.pullSelect = kPORT_PullUp;
                          port_pin_config.lockRegister = kPORT_UnlockRegister;
                          break;
    case OUTPUT_PUSH_PULL: port_pin_config.lockRegister = kPORT_UnlockRegister;
                           gpio_config.pinDirection = 1;
                           break;
    case INPUT_HIGH_IMPEDANCE: port_pin_config.lockRegister = kPORT_UnlockRegister;
                               break;
    case OUTPUT_OPEN_DRAIN_NO_PULL: port_pin_config.openDrainEnable =  kPORT_OpenDrainEnable;
                                    gpio_config.pinDirection = 1;
                                    port_pin_config.lockRegister = kPORT_UnlockRegister;
                                    break;
    case OUTPUT_OPEN_DRAIN_PULL_UP: port_pin_config.pullSelect = kPORT_PullUp;
                                    port_pin_config.openDrainEnable = kPORT_OpenDrainEnable;
                                    gpio_config.pinDirection = 1;
                                    port_pin_config.lockRegister = kPORT_UnlockRegister;
                                    break;
    default: break;
    }

    /* Initializing pin and setting configuration using */

    GPIO_PinInit(mygpio, (uint32_t)gpio->pin_number, &gpio_config);
    PORT_SetPinConfig(My_Port,(uint32_t) gpio->pin_number, &port_pin_config);

    platform_mcu_powersave_enable();
    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_deinit( const platform_gpio_t* gpio )
{
    /* Setting pin config for de-init */
    gpio_pin_config_t gpio_config = { kGPIO_DigitalOutput,0};

    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable();


    /* Setting the Init */
    GPIO_PinInit(gpio->port,gpio->pin_number, &gpio_config);

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_output_high( const platform_gpio_t* gpio )
{
    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable( );

    GPIO_WritePinOutput(gpio->port,gpio->pin_number,1);

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_output_low( const platform_gpio_t* gpio )
{
    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable();

    GPIO_WritePinOutput(gpio->port,gpio->pin_number,0);

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

wiced_bool_t platform_gpio_input_get( const platform_gpio_t* gpio )
{
    wiced_bool_t result;

    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable();

    result = ( ( GPIO_ReadPinInput(gpio->port,gpio->pin_number) & (uint32_t) ( 1 << gpio->pin_number ) ) != 0 ) ? WICED_TRUE : WICED_FALSE;

    platform_mcu_powersave_enable();

    return result;
}

platform_result_t platform_gpio_irq_enable( const platform_gpio_t* gpio, platform_gpio_irq_trigger_t trigger, platform_gpio_irq_callback_t handler, void* arg )
{
    uint32_t            interrupt_line;
    port_interrupt_t    port_trigger;
    IRQn_Type           interrupt_vector = 0;
    uint32_t            gpio_number;


     wiced_assert( "bad argument", ( gpio != NULL ) && ( handler != NULL ) );

     /* Grabbing pin line that is interrupting */
    interrupt_line = (uint32_t) (gpio->pin_number );

    /* Configuring trigger type */
    switch ( trigger )
    {
        case IRQ_TRIGGER_RISING_EDGE:
        {
            port_trigger = kPORT_InterruptRisingEdge  ;
            break;
        }
        case IRQ_TRIGGER_FALLING_EDGE:
        {
            port_trigger = kPORT_InterruptFallingEdge;
            break;
        }
        case IRQ_TRIGGER_BOTH_EDGES:
        {
            port_trigger =  kPORT_InterruptEitherEdge;
            break;
        }
        default:
        {
            return PLATFORM_BADARG;
        }
    }

    platform_mcu_powersave_disable();

    /* Grabbing base addr of GPIO to determine which port is set */
    uint32_t mygpio =  (uint32_t)&(*(gpio->port));
    if(mygpio == GPIOA_BASE){
        My_Port = PORTA;
        interrupt_vector =  PORTA_IRQn;
    }else if(mygpio == GPIOB_BASE ) {
        My_Port = PORTB;
        interrupt_vector =  PORTB_IRQn;
    }else if( mygpio == GPIOC_BASE){
        My_Port = PORTC;
        interrupt_vector =  PORTC_IRQn;
    }else if(mygpio == GPIOD_BASE ){
        My_Port = PORTD;
        interrupt_vector =  PORTD_IRQn;
    }else if(mygpio == GPIOE_BASE){
        My_Port = PORTE;
        interrupt_vector =  PORTE_IRQn;
    }else{

    }


    if ( (My_Port->ISFR  & interrupt_line ) == 0 )
    {

        /* Configuring pin for interrupt */
        PORT_SetPinInterruptConfig(My_Port,gpio->pin_number,port_trigger);

        /* Enabling interrupt */
        NVIC_EnableIRQ( interrupt_vector );

        /* Assinging port handler and arguments*/
        gpio_irq_data[gpio->pin_number].owner_port = gpio->port;
        gpio_irq_data[gpio->pin_number].handler    = handler;
        gpio_irq_data[gpio->pin_number].arg        = arg;

        platform_mcu_powersave_enable();

        return PLATFORM_SUCCESS;
    }

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;

}

platform_result_t platform_gpio_irq_disable( const platform_gpio_t* gpio )
{
   //TFL Deleted Code

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_deepsleep_wakeup_enable( const platform_gpio_t* gpio, platform_gpio_irq_trigger_t trigger )
{
    return PLATFORM_UNSUPPORTED;
}

/******************************************************
 *      MK24F12 Internal Function Definitions
 ******************************************************/

platform_result_t platform_gpio_irq_manager_init( void )
{
    memset( (void*)gpio_irq_data, 0, sizeof( gpio_irq_data ) );

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_set_alternate_function( platform_gpio_port_t* gpio_port, uint8_t pin_number, port_pin_config_t* config){

    //TFL Deleted code
    PORT_SetPinConfig(My_Port, pin_number, config);

    return PLATFORM_SUCCESS;
}

uint8_t platform_gpio_get_port_number( platform_gpio_port_t* gpio_port )
{
    return 0x00;
}

/******************************************************
 *               IRQ Handler Definitions
 ******************************************************/

/* Common IRQ handler for all GPIOs */
WWD_RTOS_DEFINE_ISR( gpio_irq )
{
    uint32_t gpio_number = 0;
    uint32_t mask = 0x01;
    uint32_t active_interrupt_vector;
    int i;
    PORT_Type* myPort;
    myPort = PORTA;
    for(i = 0; i<= 5; i++){
        /* Determining which port is triggering irq*/
        if(myPort->ISFR & PORT_ISFR_ISF_MASK){

            /*Determining which gpio pin is interrupting*/
            while(!(mask & myPort->ISFR)){
                mask = mask<<1;
                gpio_number++;
            }

            active_interrupt_vector = (uint32_t) ( myPort->ISFR & PORT_ISFR_ISF_MASK );
            /* Clear interrupt flag */
            myPort->ISFR = myPort->ISFR & active_interrupt_vector;

            /* Call the respective GPIO interrupt handler/callback after check for invalid GPIO number */
            if ( gpio_number >= NUMBER_OF_GPIO_IRQ_LINES )
            {
                /* interrupt not found; while perhaps rare, we should not operate on bad data whatever the source */
                return;
            }
            else if ( gpio_irq_data[gpio_number].handler != NULL )
            {

                /* Calling handler*/
                void * arg = gpio_irq_data[gpio_number].arg; /* Avoids undefined order of access to volatiles */
                gpio_irq_data[gpio_number].handler( arg );
            }

            return;
        }
        myPort = (PORT_Type*)((uint32_t)myPort + 0x1000);
    }
}

/******************************************************
 *               IRQ Handler Mapping
 ******************************************************/

WWD_RTOS_MAP_ISR( gpio_irq , PORTA_IRQHandler     )
WWD_RTOS_MAP_ISR( gpio_irq , PORTB_IRQHandler     )
WWD_RTOS_MAP_ISR( gpio_irq , PORTC_IRQHandler     )
WWD_RTOS_MAP_ISR( gpio_irq , PORTD_IRQHandler     )
WWD_RTOS_MAP_ISR( gpio_irq , PORTE_IRQHandler     )

