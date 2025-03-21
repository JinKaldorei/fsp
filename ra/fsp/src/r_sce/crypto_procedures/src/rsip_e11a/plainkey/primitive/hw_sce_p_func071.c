/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "hw_sce_ra_private.h"

void HW_SCE_p_func071 (const uint32_t ARG1[])
{
    WR1_PROG(REG_D0H, 0x300710c4U);
    WAIT_STS(REG_14H, 31, 1);
    WR1_PROG(REG_2CH, change_endian_long(0x01b03468U));

    WR1_PROG(REG_A0H, 0x20010000U);
    WR1_PROG(REG_B0H, 0x00001419U);
    WR1_PROG(REG_14H, 0x00000fc1U);
    WR1_PROG(REG_D4H, 0x00000300U);
    WR1_PROG(REG_D0H, 0xf7049d07U);
    WAIT_STS(REG_14H, 31, 1);
    WR4_ADDR(REG_2CH, &ARG1[0]);
    WAIT_STS(REG_14H, 31, 1);
    WR4_ADDR(REG_2CH, &ARG1[4]);
    WR1_PROG(REG_00H, 0x00c10021U);
    WAIT_STS(REG_04H, 30, 0);
    WR1_PROG(REG_40H, 0x00001800U);

    WR1_PROG(REG_B0H, 0x0000141eU);
    WAIT_STS(REG_14H, 31, 1);
    WR4_ADDR(REG_2CH, &ARG1[8]);
    WAIT_STS(REG_14H, 31, 1);
    WR4_ADDR(REG_2CH, &ARG1[12]);
    WR1_PROG(REG_00H, 0x00c10021U);
    WAIT_STS(REG_04H, 30, 0);
    WR1_PROG(REG_40H, 0x00001800U);

    WR1_PROG(REG_D0H, 0x07040d05U);
    WAIT_STS(REG_14H, 31, 1);
    WR4_ADDR(REG_2CH, &ARG1[16]);

    WR1_PROG(REG_D0H, 0x8c100005U);
    WR1_PROG(REG_00H, 0x00410011U);
    WAIT_STS(REG_04H, 30, 0);
    WR1_PROG(REG_40H, 0x00001800U);
}
