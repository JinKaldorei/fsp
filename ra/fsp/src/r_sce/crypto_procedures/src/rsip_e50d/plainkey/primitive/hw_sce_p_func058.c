/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "hw_sce_ra_private.h"

void HW_SCE_p_func058_r1 (const uint32_t ARG1[], uint32_t ARG2)
{
    HW_SCE_p_func100(0x5ffef0baU, 0x29cc50fbU, 0x46eaa5d1U, 0xaae7ba36U);
    WR1_PROG(REG_1A24H, 0x4a070044U);
    WAIT_STS(REG_1444H, 31, 1);
    WR1_PROG(REG_1420H, change_endian_long(0x00000000U));

    WR1_PROG(REG_1A24H, 0x0e0704c4U);
    WAIT_STS(REG_1444H, 31, 1);
    WR1_PROG(REG_1420H, change_endian_long(0x0174d08aU));

    HW_SCE_p_func100(0x59e30291U, 0x2827e8ffU, 0xa1a34481U, 0x74ebf5acU);
    WR1_PROG(REG_1600H, 0x00009020U);
    WR1_PROG(REG_1600H, 0x01000000U);

    WR1_PROG(REG_1A24H, 0x4a040044U);
    WAIT_STS(REG_1444H, 31, 1);
    WR1_PROG(REG_1420H, change_endian_long(0x00000000U));

    WR1_PROG(REG_1A24H, 0x0e0404c4U);
    WR1_PROG(REG_1608H, 0x81010020U);
    WR1_PROG(REG_1400H, 0x00890005U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);
    WAIT_STS(REG_1A28H, 6, 0);
    WR1_PROG(REG_143CH, 0x00000900U);

    HW_SCE_p_func100(0x98570920U, 0x84daac3eU, 0x976b3fe4U, 0x4d390fdcU);
    WR1_PROG(REG_1824H, 0xf7041cb5U);
    WAIT_STS(REG_1444H, 31, 1);
    WR4_ADDR(REG_1420H, &S_FLASH2[ARG2]);

    HW_SCE_p_func100(0x09851d2cU, 0xccc578ddU, 0x5c237f48U, 0x911f18c0U);
    WR1_PROG(REG_1824H, 0x07040d05U);
    WAIT_STS(REG_1444H, 31, 1);
    WR4_ADDR(REG_1420H, &S_FLASH2[ARG2 + 4]);

    WR1_PROG(REG_1824H, 0x8c100005U);
    WR1_PROG(REG_1400H, 0x00410011U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_1444H, 0x000003c1U);
    WR1_PROG(REG_1824H, 0x0a03008dU);
    WAIT_STS(REG_1444H, 31, 1);
    WR4_ADDR(REG_1420H, &ARG1[0]);

    HW_SCE_p_func100(0x28c96cb6U, 0xbcfdff4bU, 0x423ccc5fU, 0xea3ff045U);
    WR1_PROG(REG_1444H, 0x000003c1U);
    WR1_PROG(REG_1824H, 0x0a03009dU);
    WAIT_STS(REG_1444H, 31, 1);
    WR4_ADDR(REG_1420H, &ARG1[4]);

    WR1_PROG(REG_1600H, 0x00007c1dU);
    WR1_PROG(REG_143CH, 0x00602000U);
    WR1_PROG(REG_1458H, 0x00000000U);
}
