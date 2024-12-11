/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*******************************************************************************************************************//**
 * @addtogroup BOARD_RA6E2_FPB_LEDS
 *
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "bsp_api.h"
#if defined(BOARD_RA6E2_FPB)

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private global variables and functions
 **********************************************************************************************************************/

/** Array of LED IOPORT pins. */
static const uint16_t g_bsp_prv_leds[] =
{
    (uint16_t) BSP_IO_PORT_02_PIN_07,  ///< LED1
    (uint16_t) BSP_IO_PORT_02_PIN_06,  ///< LED2
};

/***********************************************************************************************************************
 * Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/

/** Structure with LED information for this board. */

const bsp_leds_t g_bsp_leds =
{
    .led_count = (uint16_t) ((sizeof(g_bsp_prv_leds) / sizeof(g_bsp_prv_leds[0]))),
    .p_leds    = &g_bsp_prv_leds[0]
};

/***********************************************************************************************************************
 * Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/

#endif

/** @} (end addtogroup BOARD_RA6E2_FPB_LEDS) */
