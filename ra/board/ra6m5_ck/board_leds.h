/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*******************************************************************************************************************//**
 * @ingroup BOARD_RA6M5_CK
 * @defgroup BOARD_RA6M5_CK_LEDS Board LEDs
 * @brief LED information for this board.
 *
 * This is code specific to the CK board. It includes info on the number of LEDs and which pins are they
 * are on.
 *
 * @{
 **********************************************************************************************************************/

#ifndef BOARD_LEDS_H
#define BOARD_LEDS_H

/** Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/** Information on how many LEDs and what pins they are on. */
typedef struct st_bsp_leds
{
    uint16_t         led_count;        ///< The number of LEDs on this board
    uint16_t const * p_leds;           ///< Pointer to an array of IOPORT pins for controlling LEDs
} bsp_leds_t;

/** Available user-controllable LEDs on this board. These enums can be can be used to index into the array of LED pins
 * found in the bsp_leds_t structure. */
typedef enum e_bsp_led
{
    BSP_LED_LED1,                      ///< LED1 RED
    BSP_LED_LED3,                      ///< LED3 GREEN
    BSP_LED_LED4,                      ///< LED4 BLUE
    BSP_LED_LED2_RED,                  ///< LED2_TRI_COLOUR_RED
    BSP_LED_LED2_GREEN,                ///< LED2_TRI_COLOUR_GREEN
    BSP_LED_LED2_BLUE,                 ///< LED2_TRI_COLOUR_BLUE
} bsp_led_t;

/***********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/** Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif

/** @} (end defgroup BOARD_RA6M5_CK_LEDS) */
