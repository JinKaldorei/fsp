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

void r_rsip_func216 (void)
{
    WR1_PROG(REG_1444H, 0x00000000U);
    WAIT_STS(REG_1828H, 6, 0);
    WR1_PROG(REG_143CH, 0x00000400U);
    WR1_PROG(REG_143CH, 0x00000600U);
    WR1_PROG(REG_143CH, 0x00000500U);
    WR1_PROG(REG_1824H, 0x00000000U);
}
