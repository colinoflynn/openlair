/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
PinsProfile:
- !!product 'Pins v2.0'
- !!processor 'MK64FN1M0xxx12'
- !!package 'MK64FN1M0VLL12'
- !!mcu_data 'ksdk2_0'
- !!processor_version '1.0.9'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"


#define PIN0_IDX                         0u   /*!< Pin number for pin 0 in a port */

#define PIN1_IDX                         1u   /*!< Pin number for pin 1 in a port */

#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */

#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port */

#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */

#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */

#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port */

#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */

#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */

#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART 0 transmit data source select: UART0_TX pin */
/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
BOARD_InitPins:
- options: {coreID: singlecore, enableClock: 'true'}
- pin_list:
  - {pin_num: '62', peripheral: UART0, signal: RX, pin_signal: PTB16/SPI1_SOUT/UART0_RX/FTM_CLKIN0/FB_AD17/EWM_IN}
  - {pin_num: '63', peripheral: UART0, signal: TX, pin_signal: PTB17/SPI1_SIN/UART0_TX/FTM_CLKIN1/FB_AD16/EWM_OUT_b}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/

void BOARD_InitPins(void) {

      CLOCK_EnableClock(kCLOCK_PortA);
      CLOCK_EnableClock(kCLOCK_PortB);                           /* Port B Clock Gate Control: Clock enabled */
      CLOCK_EnableClock(kCLOCK_PortE);                           /* Port E Clock Gate Control: Clock enabled */
      CLOCK_EnableClock(kCLOCK_PortC);
      //PORT_SetPinMux(PORTB, 9, kPORT_MuxAsGpio);
      PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_MuxAlt3);
      PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_MuxAlt3);           /* PORTB16 (pin 62) is configured as UART0_RX */
      const port_pin_config_t portb0_pin9_config = {                /* PORTB17 (pin 63) is configured as UART0_TX */
          kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
          kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
          kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
          kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
          kPORT_HighDriveStrength,                                 /* High drive strength is configured */
          kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_D3 */
          kPORT_UnlockRegister
      };
      PORT_SetPinConfig(PORTB, 9, &portb0_pin9_config);
      const port_pin_config_t porte0_pin1_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
        kPORT_HighDriveStrength,                                 /* High drive strength is configured */
        kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_D1 */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
      PORT_SetPinConfig(PORTE, PIN0_IDX, &porte0_pin1_config);   /* PORTE0 (pin 1) is configured as SDHC0_D1 */
      const port_pin_config_t porte1_pin2_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
        kPORT_HighDriveStrength,                                 /* High drive strength is configured */
        kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_D0 */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
      PORT_SetPinConfig(PORTE, PIN1_IDX, &porte1_pin2_config);   /* PORTE1 (pin 2) is configured as SDHC0_D0 */
      const port_pin_config_t porte2_pin3_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
        kPORT_HighDriveStrength,                                 /* High drive strength is configured */
        kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_DCLK */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
      PORT_SetPinConfig(PORTE, PIN2_IDX, &porte2_pin3_config);   /* PORTE2 (pin 3) is configured as SDHC0_DCLK */
      const port_pin_config_t porte3_pin4_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
        kPORT_HighDriveStrength,                                 /* High drive strength is configured */
        kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_CMD */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
      PORT_SetPinConfig(PORTE, PIN3_IDX, &porte3_pin4_config);   /* PORTE3 (pin 4) is configured as SDHC0_CMD */
      const port_pin_config_t porte4_pin5_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
        kPORT_HighDriveStrength,                                 /* High drive strength is configured */
        kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_D3 */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
      PORT_SetPinConfig(PORTE, PIN4_IDX, &porte4_pin5_config);   /* PORTE4 (pin 5) is configured as SDHC0_D3 */
      const port_pin_config_t porte5_pin6_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
        kPORT_HighDriveStrength,                                 /* High drive strength is configured */
        kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_D2 */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
      PORT_SetPinConfig(PORTE, PIN5_IDX, &porte5_pin6_config);   /* PORTE5 (pin 6) is configured as SDHC0_D2 */
      const port_pin_config_t portc4_pin5_config = {
        kPORT_PullDown,                                          /* Internal pull-down resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
        kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
        kPORT_MuxAsGpio,                                         /* Pin is configured as PTE6 */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
      PORT_SetPinConfig(PORTC, PIN4_IDX, &portc4_pin5_config);   /* PORTC4 (pin 5) is configured as PTE6 */
      SIM->SOPT5 = ((SIM->SOPT5 &
        (~(SIM_SOPT5_UART0TXSRC_MASK)))                          /* Mask bits to zero which are setting */
          | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)       /* UART 0 transmit data source select: UART0_TX pin */
        );
      /* Port B Clock Gate Control: Clock enabled */
      CLOCK_EnableClock(kCLOCK_PortB);
      /* Port C Clock Gate Control: Clock enabled */
      CLOCK_EnableClock(kCLOCK_PortC);
      /* Port D Clock Gate Control: Clock enabled */
      CLOCK_EnableClock(kCLOCK_PortD);
      /* Port E Clock Gate Control: Clock enabled */
      CLOCK_EnableClock(kCLOCK_PortE);

      gpio_pin_config_t GPIOB_1_config = {
          .pinDirection = kGPIO_DigitalOutput,
          .outputLogic = 1U
      };
      /* Initialize GPIO functionality on pin PTB1 (pin 54)  */
      GPIO_PinInit(BOARD_INITPINS_GPIOB_1_GPIO, BOARD_INITPINS_GPIOB_1_PIN, &GPIOB_1_config);

      gpio_pin_config_t GPIOB_9_config = {
          .pinDirection = kGPIO_DigitalInput,
          .outputLogic = 0U
      };
      /* Initialize GPIO functionality on pin PTB9 (pin 57)  */
      GPIO_PinInit(BOARD_INITPINS_GPIOB_9_GPIO, BOARD_INITPINS_GPIOB_9_PIN, &GPIOB_9_config);

      gpio_pin_config_t GPIOC_0_config = {
          .pinDirection = kGPIO_DigitalOutput,
          .outputLogic = 1U
      };
      /* Initialize GPIO functionality on pin PTC0 (pin 70)  */
      GPIO_PinInit(BOARD_INITPINS_GPIOC_0_GPIO, BOARD_INITPINS_GPIOC_0_PIN, &GPIOC_0_config);

      gpio_pin_config_t GPIOD_0_config = {
          .pinDirection = kGPIO_DigitalOutput,
          .outputLogic = 1U
      };
      /* Initialize GPIO functionality on pin PTD0 (pin 93)  */
      GPIO_PinInit(BOARD_INITPINS_GPIOD_0_GPIO, BOARD_INITPINS_GPIOD_0_PIN, &GPIOD_0_config);

      /* PORTB1 (pin 54) is configured as PTB1 */
      PORT_SetPinMux(BOARD_INITPINS_GPIOB_1_PORT, BOARD_INITPINS_GPIOB_1_PIN, kPORT_MuxAsGpio);

      PORTB->PCR[1] = ((PORTB->PCR[1] &
                        /* Mask bits to zero which are setting */
                        (~(PORT_PCR_ODE_MASK | PORT_PCR_ISF_MASK)))

                       /* Open Drain Enable: Open drain output is disabled on the corresponding pin. */
                       | PORT_PCR_ODE(kPORT_OpenDrainDisable));

      /* PORTB9 (pin 57) is configured as PTB9 */
      PORT_SetPinMux(BOARD_INITPINS_GPIOB_9_PORT, BOARD_INITPINS_GPIOB_9_PIN, kPORT_MuxAsGpio);

      /* PORTC0 (pin 70) is configured as PTC0 */
      PORT_SetPinMux(BOARD_INITPINS_GPIOC_0_PORT, BOARD_INITPINS_GPIOC_0_PIN, kPORT_MuxAsGpio);

      PORTC->PCR[0] = ((PORTC->PCR[0] &
                        /* Mask bits to zero which are setting */
                        (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                       /* Pull Select: Internal pulldown resistor is enabled on the corresponding pin, if the
                        * corresponding PE field is set. */
                       | (uint32_t)(kPORT_PullDown));

      /* PORTD0 (pin 93) is configured as PTD0 */
      PORT_SetPinMux(BOARD_INITPINS_GPIOD_0_PORT, BOARD_INITPINS_GPIOD_0_PIN, kPORT_MuxAsGpio);

      /* PORTE0 (pin 1) is configured as SDHC0_D1 */
      PORT_SetPinMux(BOARD_INITPINS_GPIOE_0_PORT, BOARD_INITPINS_GPIOE_0_PIN, kPORT_MuxAlt4);

      /* PORTE1 (pin 2) is configured as SDHC0_D0 */
      PORT_SetPinMux(BOARD_INITPINS_GPIOE_1_PORT, BOARD_INITPINS_GPIOE_1_PIN, kPORT_MuxAlt4);

      /* PORTE2 (pin 3) is configured as SDHC0_DCLK */
      PORT_SetPinMux(BOARD_INITPINS_GPIOE_2_PORT, BOARD_INITPINS_GPIOE_2_PIN, kPORT_MuxAlt4);

      PORTE->PCR[2] = ((PORTE->PCR[2] &
                        /* Mask bits to zero which are setting */
                        (~(PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK)))

                       /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin
                        * is configured as a digital output. */
                       | PORT_PCR_DSE(kPORT_HighDriveStrength));

      /* PORTE3 (pin 4) is configured as SDHC0_CMD */
      PORT_SetPinMux(BOARD_INITPINS_GPIOE_3_PORT, BOARD_INITPINS_GPIOE_3_PIN, kPORT_MuxAlt4);

      /* PORTE4 (pin 5) is configured as SDHC0_D3 */
      PORT_SetPinMux(BOARD_INITPINS_GPIOE_4_PORT, BOARD_INITPINS_GPIOE_4_PIN, kPORT_MuxAlt4);

      /* PORTE5 (pin 6) is configured as SDHC0_D2 */
      PORT_SetPinMux(BOARD_INITPINS_GPIOE_5_PORT, BOARD_INITPINS_GPIOE_5_PIN, kPORT_MuxAlt4);

}


/*******************************************************************************
 * EOF
 ******************************************************************************/
