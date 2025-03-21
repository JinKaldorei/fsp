/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "hw_sce_ra_private.h"

void HW_SCE_p_func008 (void)
{
    WR1_PROG(REG_B4H, 0x0037000fU);
    WR1_PROG(REG_B8H, 0x00190005U);

    WR1_PROG(REG_A4H, 0x04040002U);
    WR1_PROG(REG_A0H, 0x20010001U);
    WAIT_STS(REG_A8H, 0, 1);
    WR1_PROG(REG_ACH, 0x00000001U);

    WR1_PROG(REG_B4H, 0x00370014U);
    WR1_PROG(REG_B8H, 0x001e0005U);

    WR1_PROG(REG_A4H, 0x04040002U);
    WR1_PROG(REG_A0H, 0x20010001U);
    WAIT_STS(REG_A8H, 0, 1);
    WR1_PROG(REG_ACH, 0x00000001U);

    WR1_PROG(REG_B4H, 0x00190019U);
    WR1_PROG(REG_B8H, 0x000f0005U);

    WR1_PROG(REG_A4H, 0x04040002U);
    WR1_PROG(REG_A0H, 0x20010001U);
    WAIT_STS(REG_A8H, 0, 1);
    WR1_PROG(REG_ACH, 0x00000001U);

    WR1_PROG(REG_B4H, 0x005f000fU);
    WR1_PROG(REG_B8H, 0x00140005U);

    WR1_PROG(REG_A4H, 0x04040005U);
    WR1_PROG(REG_A0H, 0x20010001U);
    WAIT_STS(REG_A8H, 0, 1);
    WR1_PROG(REG_ACH, 0x00000001U);

    WR1_PROG(REG_B4H, 0x00190014U);
    WR1_PROG(REG_B8H, 0x000f0005U);

    WR1_PROG(REG_A4H, 0x04040002U);
    WR1_PROG(REG_A0H, 0x20010001U);
    WAIT_STS(REG_A8H, 0, 1);
    WR1_PROG(REG_ACH, 0x00000001U);

    WR1_PROG(REG_B4H, 0x0064000fU);
    WR1_PROG(REG_B8H, 0x00140005U);

    WR1_PROG(REG_A4H, 0x04040005U);
    WR1_PROG(REG_A0H, 0x20010001U);
    WAIT_STS(REG_A8H, 0, 1);
    WR1_PROG(REG_ACH, 0x00000001U);

    WR1_PROG(REG_B4H, 0x001e001eU);
    WR1_PROG(REG_B8H, 0x000f0005U);

    WR1_PROG(REG_A4H, 0x04040002U);
    WR1_PROG(REG_A0H, 0x20010001U);
    WAIT_STS(REG_A8H, 0, 1);
    WR1_PROG(REG_ACH, 0x00000001U);

    WR1_PROG(REG_B4H, 0x0014000fU);
    WR1_PROG(REG_B8H, 0x00230000U);

    WR1_PROG(REG_A4H, 0x0404000aU);
    WR1_PROG(REG_A0H, 0x20010001U);
    WAIT_STS(REG_A8H, 0, 1);
    WR1_PROG(REG_ACH, 0x00000001U);

    WR1_PROG(REG_40H, 0x00210000U);

    WR1_PROG(REG_B4H, 0x000f0014U);
    WR1_PROG(REG_B8H, 0x00230000U);

    WR1_PROG(REG_A4H, 0x0404000aU);
    WR1_PROG(REG_A0H, 0x20010001U);
    WAIT_STS(REG_A8H, 0, 1);
    WR1_PROG(REG_ACH, 0x00000001U);

    WR1_PROG(REG_40H, 0x00210000U);
}
