/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "hw_sce_ra_private.h"

void HW_SCE_p_func310 (void)
{
    uint32_t oLoop = 0U;

    WR1_PROG(REG_1404H, 0x18c00000U);
    WR1_PROG(REG_1400H, 0x00c00089U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_1404H, 0x19d00000U);
    WR1_PROG(REG_1444H, 0x000000a2U);
    WR1_PROG(REG_1A24H, 0x08000104U);
    WAIT_STS(REG_1444H, 31, 1);
    WR1_PROG(REG_1420H, change_endian_long(0x00000001U));
    WR1_PROG(REG_1400H, 0x00c00085U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);
    WR1_PROG(REG_1400H, 0x00c20005U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);
    WR1_PROG(REG_1400H, 0x0002000dU);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_1014H, 0x00000630U);
    WR1_PROG(REG_1018H, 0x00000948U);
    WR1_PROG(REG_1020H, 0x00000b68U);

    WR1_PROG(REG_1004H, 0x10100009U);
    WR1_PROG(REG_1000H, 0x00010001U);
    WAIT_STS(REG_1000H, 0, 0);

    WR1_PROG(REG_1404H, 0x1bf00000U);
    WR1_PROG(REG_1400H, 0x00c00089U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_1600H, 0x00000800U);

    WR1_PROG(REG_1600H, 0x0000b420U);
    WR1_PROG(REG_1600H, 0x00000400U);

    WR1_PROG(REG_1600H, 0x00000bffU);

    for (oLoop = 0U; oLoop < 1024U; oLoop++)
    {
        WR1_PROG(REG_1404H, 0x19400000U);
        WR1_PROG(REG_1608H, 0x800103e0U);
        WR1_PROG(REG_1400H, 0x00030005U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);
        WR1_PROG(REG_1400H, 0x03430005U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        WR1_PROG(REG_1600H, 0x3800dbe0U);
        WR1_PROG(REG_1608H, 0x00000080U);
        WR1_PROG(REG_143CH, 0x00260000U);

        HW_SCE_p_func100(0xdd5a4bc8U, 0xbd4d2eddU, 0x55fa46a4U, 0x467e1969U);
        WR1_PROG(REG_143CH, 0x00400000U);

        if (CHCK_STS(REG_143CH, 22, 1))
        {
            WR1_PROG(REG_1014H, 0x00000948U);
            WR1_PROG(REG_1018H, 0x00000b68U);
            WR1_PROG(REG_1020H, 0x00000948U);

            WR1_PROG(REG_1004H, 0x11110009U);
            WR1_PROG(REG_1000H, 0x00010001U);
            WAIT_STS(REG_1000H, 0, 0);

            WR1_PROG(REG_1014H, 0x00000c78U);
            WR1_PROG(REG_1018H, 0x00000a58U);
            WR1_PROG(REG_1020H, 0x00000c78U);

            WR1_PROG(REG_1004H, 0x11110009U);
            WR1_PROG(REG_1000H, 0x00010001U);
            WAIT_STS(REG_1000H, 0, 0);

            HW_SCE_p_func101(0x22406a23U, 0x7071a1a9U, 0x5bc1cc1cU, 0x18c1e40eU);
        }
        else
        {
            HW_SCE_p_func101(0x47f4957cU, 0xd5e64b20U, 0xd7c32b6fU, 0x1b31ddeeU);
        }

        WR1_PROG(REG_1458H, 0x00000000U);

        WR1_PROG(REG_1014H, 0x00000948U);
        WR1_PROG(REG_1020H, 0x00000948U);

        WR1_PROG(REG_1004H, 0x1111000cU);
        WR1_PROG(REG_1000H, 0x00010001U);
        WAIT_STS(REG_1000H, 0, 0);

        WR1_PROG(REG_1014H, 0x00000a58U);
        WR1_PROG(REG_1018H, 0x00000a58U);
        WR1_PROG(REG_1020H, 0x00000a58U);

        WR1_PROG(REG_1004H, 0x11110009U);
        WR1_PROG(REG_1000H, 0x00010001U);
        WAIT_STS(REG_1000H, 0, 0);

        WR1_PROG(REG_1600H, 0x00002c00U);

        HW_SCE_p_func101(0xc97e576aU, 0x96b4c22dU, 0x30c99674U, 0x00d94a38U);
    }

    WR1_PROG(REG_1600H, 0x38000801U);
    WR1_PROG(REG_1608H, 0x00000080U);
    WR1_PROG(REG_143CH, 0x00260000U);

    WR1_PROG(REG_143CH, 0x00402000U);
    WR1_PROG(REG_1458H, 0x00000000U);

    WR1_PROG(REG_1600H, 0x00000800U);
    WR1_PROG(REG_1608H, 0x80a00000U);
    WR1_PROG(REG_1404H, 0x1bf80000U);
    WR1_PROG(REG_1400H, 0x03430081U);
    WAIT_STS(REG_1404H, 30, 0);
    WR1_PROG(REG_143CH, 0x00001800U);

    WR1_PROG(REG_1600H, 0x00007c1dU);
    WR1_PROG(REG_143CH, 0x00602000U);
    WR1_PROG(REG_1458H, 0x00000000U);
}
