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

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
  kPIN_MUX_DirectionInput = 0U,         /* Input direction */
  kPIN_MUX_DirectionOutput = 1U,        /* Output direction */
  kPIN_MUX_DirectionInputOrOutput = 2U  /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);



/*! @name PORTB1 (number 54), GPIOB_1
  @{ */
#define BOARD_INITPINS_GPIOB_1_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITPINS_GPIOB_1_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITPINS_GPIOB_1_PIN 1U     /*!<@brief PORTB pin index: 1 */
                                          /* @} */

/*! @name PORTB9 (number 57), GPIOB_9
  @{ */
#define BOARD_INITPINS_GPIOB_9_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITPINS_GPIOB_9_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITPINS_GPIOB_9_PIN 9U     /*!<@brief PORTB pin index: 9 */
                                          /* @} */

/*! @name PORTC0 (number 70), GPIOC_0
  @{ */
#define BOARD_INITPINS_GPIOC_0_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITPINS_GPIOC_0_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_GPIOC_0_PIN 0U     /*!<@brief PORTC pin index: 0 */
                                          /* @} */

/*! @name PORTE0 (number 1), GPIOE_0
  @{ */
#define BOARD_INITPINS_GPIOE_0_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_GPIOE_0_PIN 0U     /*!<@brief PORTE pin index: 0 */
                                          /* @} */

/*! @name PORTE1 (number 2), GPIOE_1
  @{ */
#define BOARD_INITPINS_GPIOE_1_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_GPIOE_1_PIN 1U     /*!<@brief PORTE pin index: 1 */
                                          /* @} */

/*! @name PORTE3 (number 4), GPIOE_3
  @{ */
#define BOARD_INITPINS_GPIOE_3_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_GPIOE_3_PIN 3U     /*!<@brief PORTE pin index: 3 */
                                          /* @} */

/*! @name PORTE4 (number 5), GPIOE_4
  @{ */
#define BOARD_INITPINS_GPIOE_4_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_GPIOE_4_PIN 4U     /*!<@brief PORTE pin index: 4 */
                                          /* @} */

/*! @name PORTE5 (number 6), GPIOE_5
  @{ */
#define BOARD_INITPINS_GPIOE_5_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_GPIOE_5_PIN 5U     /*!<@brief PORTE pin index: 5 */
                                          /* @} */

/*! @name PORTD0 (number 93), GPIOD_0
  @{ */
#define BOARD_INITPINS_GPIOD_0_GPIO GPIOD /*!<@brief GPIO device name: GPIOD */
#define BOARD_INITPINS_GPIOD_0_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_GPIOD_0_PIN 0U     /*!<@brief PORTD pin index: 0 */
                                          /* @} */

/*! @name PORTE2 (number 3), GPIOE_2
  @{ */
#define BOARD_INITPINS_GPIOE_2_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_GPIOE_2_PIN 2U     /*!<@brief PORTE pin index: 2 */
                                          /* @} */



#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
