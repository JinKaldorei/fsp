/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "hw_sce_ra_private.h"

void HW_SCE_p_func031_r1 (const uint32_t ARG1[])
{
    WR1_PROG(REG_1600H, 0x0000356aU);
    WR1_PROG(REG_1600H, 0x0420a960U);
    WR1_PROG(REG_1600H, 0x00000002U);

    WR1_PROG(REG_1600H, 0x0001696bU);
    WR1_PROG(REG_1600H, 0x00036d6bU);

    WR1_PROG(REG_1600H, 0x00009160U);
    WR1_PROG(REG_1600H, 0x00000042U);
    WR1_PROG(REG_1600H, 0x00186d6bU);

    WR1_PROG(REG_1600H, 0x00008c60U);
    WR1_PROG(REG_1600H, 0x00ffffffU);
    WR1_PROG(REG_1600H, 0x0000106bU);
    WR1_PROG(REG_1600H, 0x000010c9U);

    WR1_PROG(REG_1824H, 0x08000105U);
    WR1_PROG(REG_1608H, 0x81040060U);
    WR1_PROG(REG_1400H, 0x00490011U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_1600H, 0x00000821U);
    WR1_PROG(REG_1608H, 0x80840001U);
    WR1_PROG(REG_1400H, 0x03410011U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_1444H, 0x000003c1U);
    WR1_PROG(REG_1824H, 0x08000105U);
    WAIT_STS(REG_1444H, 31, 1);
    WR4_ADDR(REG_1420H, &ARG1[0]);
    WR1_PROG(REG_1608H, 0x80040180U);
    WR1_PROG(REG_1400H, 0x03410011U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_1600H, 0x0000b560U);
    WR1_PROG(REG_1600H, 0x00000005U);
    WR1_PROG(REG_1600H, 0x01906d6cU);
    WR1_PROG(REG_1600H, 0x01906d8dU);
    WR1_PROG(REG_1600H, 0x000009adU);
    WR1_PROG(REG_1600H, 0x000009ceU);

    WR1_PROG(REG_1824H, 0x08000105U);
    WR1_PROG(REG_1608H, 0x81040160U);
    WR1_PROG(REG_1400H, 0x00490011U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_1600H, 0x0000a420U);
    WR1_PROG(REG_1600H, 0x00000010U);
    WR1_PROG(REG_1608H, 0x80840001U);
    WR1_PROG(REG_1400H, 0x03410011U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_182CH, 0x00000100U);
    WR1_PROG(REG_1824H, 0xf8008007U);
    WR1_PROG(REG_1600H, 0x00000821U);
    WR1_PROG(REG_1608H, 0x81880001U);
    WR1_PROG(REG_1400H, 0x00490021U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);
}
