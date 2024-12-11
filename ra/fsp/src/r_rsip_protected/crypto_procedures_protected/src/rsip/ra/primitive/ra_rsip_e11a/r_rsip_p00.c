/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "r_rsip_primitive.h"
#include "r_rsip_reg.h"
#include "r_rsip_util.h"

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

void r_rsip_p00 (void)
{
    WR1_PROG(REG_000CH, 0x00000000U);
    WR1_PROG(REG_0024H, 0x00000000U);
    WR1_PROG(REG_0024H, 0x00000000U);
    WR1_PROG(REG_0024H, 0x00000000U);
}
