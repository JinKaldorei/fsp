/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "hw_sce_ra_private.h"

fsp_err_t HW_SCE_SelfCheck2Sub (void)
{
    uint32_t iLoop = 0U;
    uint32_t jLoop = 0U;

    if (RD1_MASK(REG_14BCH, 0x0000001fU) != 0)
    {
        return FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT;
    }
    else
    {
        ;
    }

    WR1_PROG(REG_1B00H, 0x00820001U);
    WR1_PROG(REG_144CH, 0x00000000U);

    WR1_PROG(REG_1444H, 0x000000a2U);
    WR1_PROG(REG_1A24H, 0x0b070194U);
    WAIT_STS(REG_1444H, 31, 1);
    WR1_PROG(REG_1420H, change_endian_long(0x01c7ba56U));

    WR1_PROG(REG_1444H, 0x000000a2U);
    WR1_PROG(REG_1A24H, 0x08000074U);
    WAIT_STS(REG_1444H, 31, 1);
    WR1_PROG(REG_1420H, change_endian_long(0x00000000U));

    WR1_PROG(REG_1600H, 0x3000a820U);
    WR1_PROG(REG_1600H, 0x00000003U);
    WR1_PROG(REG_1600H, 0x00010020U);
    WR1_PROG(REG_1600H, 0x00000821U);
    WR1_PROG(REG_1600H, 0x00000080U);

    WAIT_STS(REG_1708H, 0, 0);
    WAIT_STS(REG_1708H, 0, 0);
    WR1_PROG(REG_1704H, 0x00000080U);

    WR1_PROG(REG_1600H, 0x00000863U);

    WR1_PROG(REG_1600H, 0x00000884U);

    WR1_PROG(REG_1600H, 0x000008a5U);

    WR1_PROG(REG_1600H, 0x0000b4c0U);
    WR1_PROG(REG_1600H, 0x00000013U);

    WR1_PROG(REG_1600H, 0x0000b4e0U);
    WR1_PROG(REG_1600H, 0x00000348U);

    WR1_PROG(REG_1600H, 0x0000b500U);
    WR1_PROG(REG_1600H, 0x000000b7U);

    for (jLoop = 0U; jLoop < 1U; jLoop++)
    {
        HW_SCE_p_func100(0x001aafa7U, 0xe7194d3bU, 0x4660e909U, 0xd6443c2fU);
        WR1_PROG(REG_1600H, 0x00007c01U);
        WR1_PROG(REG_143CH, 0x00600000U);
        WR1_PROG(REG_1458H, 0x00000000U);

        if (RD1_MASK(REG_1440H, 0xffffffffU) == 0x00000000U)
        {
            WAIT_STS(REG_1708H, 0, 0);
            WAIT_STS(REG_1708H, 0, 0);
            WR1_PROG(REG_1704H, 0x00200007U);

            HW_SCE_p_func101(0x7be8f4f9U, 0xdc28fc58U, 0x713f283aU, 0x5493f74dU);
        }
        else if (RD1_MASK(REG_1440H, 0xffffffffU) == 0x00000001U)
        {
            WAIT_STS(REG_1708H, 0, 0);
            WAIT_STS(REG_1708H, 0, 0);
            WR1_PROG(REG_1704H, 0x00200005U);

            HW_SCE_p_func101(0xf0e92192U, 0xf7b43edcU, 0x62c423c7U, 0x6705c4adU);
        }
        else if (RD1_MASK(REG_1440H, 0xffffffffU) == 0x00000002U)
        {
            WAIT_STS(REG_1708H, 0, 0);
            WAIT_STS(REG_1708H, 0, 0);
            WR1_PROG(REG_1704H, 0x00200006U);

            HW_SCE_p_func101(0xca955613U, 0xc28a0078U, 0x4607d6eaU, 0x76ebd60eU);
        }

        WR1_PROG(REG_1A2CH, 0x00000700U);
        WR1_PROG(REG_1A24H, 0x0e3d8407U);
        WR1_PROG(REG_1400H, 0x00840081U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);
        WAIT_STS(REG_1708H, 2, 1);
        WR1_PROG(REG_143CH, 0x00001200U);

        WAIT_STS(REG_1A28H, 6, 0);
        WR1_PROG(REG_143CH, 0x00000a00U);
        WR1_PROG(REG_1600H, 0x00000800U);
        WR1_PROG(REG_1608H, 0x808a0000U);
        WR1_PROG(REG_1400H, 0x03440029U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        WR1_PROG(REG_1600H, 0x000038a0U);
        WR1_PROG(REG_1600H, 0x00003405U);
        WR1_PROG(REG_1600H, 0x00002804U);

        WR1_PROG(REG_1600H, 0x342028e0U);
        WR1_PROG(REG_1600H, 0x10005066U);

        WR1_PROG(REG_1600H, 0x34202808U);
        WR1_PROG(REG_1600H, 0x10005066U);

        WR1_PROG(REG_1600H, 0x00003485U);

        HW_SCE_p_func101(0x5de0ca1fU, 0x89976bf9U, 0xa823becaU, 0xf60099dbU);
    }

    WR1_PROG(REG_1458H, 0x00000000U);

    WR1_PROG(REG_1600H, 0x0000b4e0U);
    WR1_PROG(REG_1600H, 0x0000005AU);

    WR1_PROG(REG_1600H, 0x00000842U);

    WR1_PROG(REG_1600H, 0x000008c6U);

    WR1_PROG(REG_1600H, 0x0000b480U);
    WR1_PROG(REG_1600H, 0x00000004U);

    WR1_PROG(REG_1600H, 0x0000b4a0U);
    WR1_PROG(REG_1600H, 0x00000002U);

    for (iLoop = 0U; iLoop < 16U; iLoop++)
    {
        WR1_PROG(REG_1600H, 0x01003804U);

        WR1_PROG(REG_1600H, 0x342028e0U);
        WR1_PROG(REG_1600H, 0x10005066U);

        WR1_PROG(REG_1600H, 0x00002440U);

        WR1_PROG(REG_1600H, 0x00002cc0U);

        WR1_PROG(REG_1600H, 0x00002485U);
    }

    WR1_PROG(REG_1458H, 0x00000000U);

    WR1_PROG(REG_1600H, 0x00002c20U);

    WR1_PROG(REG_1600H, 0x38008840U);
    WR1_PROG(REG_1600H, 0x00000100U);
    WR1_PROG(REG_1608H, 0x00000080U);
    WR1_PROG(REG_143CH, 0x00260000U);

    WR1_PROG(REG_143CH, 0x00402000U);
    WR1_PROG(REG_1458H, 0x00000000U);

    WR1_PROG(REG_1600H, 0x0000b4e0U);
    WR1_PROG(REG_1600H, 0x00000033U);

    WR1_PROG(REG_1600H, 0x0000b480U);
    WR1_PROG(REG_1600H, 0x00000024U);

    WR1_PROG(REG_1600H, 0x01003804U);

    WR1_PROG(REG_1600H, 0x342028e0U);
    WR1_PROG(REG_1600H, 0x10005066U);

    WR1_PROG(REG_1600H, 0x00002cc0U);

    WR1_PROG(REG_1600H, 0x0000b480U);
    WR1_PROG(REG_1600H, 0x00000026U);

    WR1_PROG(REG_1600H, 0x01003804U);

    WR1_PROG(REG_1600H, 0x342028e0U);
    WR1_PROG(REG_1600H, 0x10005066U);

    WR1_PROG(REG_1600H, 0x38008860U);
    WR1_PROG(REG_1600H, 0x00000000U);
    WR1_PROG(REG_1608H, 0x00000080U);
    WR1_PROG(REG_143CH, 0x00A60000U);

    HW_SCE_p_func100(0x33c0a71bU, 0x60edb617U, 0x3f685978U, 0x8dc79b15U);
    WR1_PROG(REG_143CH, 0x00400000U);

    if (CHCK_STS(REG_143CH, 22, 1))
    {
        HW_SCE_p_func102(0xf4ae769bU, 0xf95a155cU, 0x3c3776f0U, 0x745e670dU);
        WR1_PROG(REG_14BCH, 0x00000040U);
        WAIT_STS(REG_142CH, 12, 0);

        return FSP_ERR_CRYPTO_SCE_RETRY;
    }
    else
    {
        HW_SCE_p_func100(0x3e45e5b5U, 0xc8971bf2U, 0x1965908bU, 0x7d1cdc17U);

        WR1_PROG(REG_1444H, 0x000000a2U);
        WR1_PROG(REG_1A24H, 0x0c300104U);
        WAIT_STS(REG_1444H, 31, 1);
        WR1_PROG(REG_1420H, change_endian_long(0x00000000U));
        WR1_PROG(REG_1608H, 0x80040000U);
        WR1_PROG(REG_1400H, 0x03420011U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        WR1_PROG(REG_1A2CH, 0x00000600U);
        WR1_PROG(REG_1A24H, 0x0e3d9407U);
        WAIT_STS(REG_1708H, 0, 0);
        WAIT_STS(REG_1708H, 0, 0);
        WR1_PROG(REG_1704H, 0x001c0013U);
        WR1_PROG(REG_1400H, 0x00840071U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);
        WR1_PROG(REG_1A24H, 0x0e3d0505U);
        WAIT_STS(REG_1708H, 0, 0);
        WAIT_STS(REG_1708H, 0, 0);
        WR1_PROG(REG_1704H, 0x00040013U);
        WR1_PROG(REG_1400H, 0x00840011U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        WAIT_STS(REG_1708H, 3, 0);
        WR1_PROG(REG_1700H, 0x00000000U);
        WR1_PROG(REG_1608H, 0x80040080U);
        WR1_PROG(REG_1400H, 0x03420011U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        WR1_PROG(REG_1444H, 0x000000a2U);
        WR1_PROG(REG_1A24H, 0x080000b4U);
        WAIT_STS(REG_1444H, 31, 1);
        WR1_PROG(REG_1420H, change_endian_long(0x00000000U));

        HW_SCE_p_func100(0x4dfc9991U, 0x2ce09ff6U, 0x7b4a9406U, 0xa97f2d24U);
        WR1_PROG(REG_1444H, 0x000003a2U);
        WR1_PROG(REG_1A24H, 0x08000075U);
        WAIT_STS(REG_1444H, 31, 1);
        WR4_PROG(REG_1420H, change_endian_long(0x00000000U), change_endian_long(0x00000000U), change_endian_long(0x00000000U), change_endian_long(0x00000001U));

        WR1_PROG(REG_1A2CH, 0x00000100U);
        WR1_PROG(REG_1A24H, 0x07338d07U);
        WR1_PROG(REG_1608H, 0x81080000U);
        WR1_PROG(REG_1400H, 0x00890021U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        WR1_PROG(REG_1A24H, 0x080000b5U);
        WR1_PROG(REG_1400H, 0x00820011U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        HW_SCE_p_func100(0x68c2c5c4U, 0x638352ebU, 0xa543f65dU, 0x859825e1U);
        WR1_PROG(REG_1A24H, 0x08000075U);
        WR1_PROG(REG_1400H, 0x00820011U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        HW_SCE_p_func103();
        HW_SCE_p_func100(0x9266f1ecU, 0x77eb56acU, 0x3284ac2eU, 0x4b9125ceU);
        WR1_PROG(REG_1444H, 0x000000a2U);
        WR1_PROG(REG_1A24H, 0x0c2000d4U);
        WAIT_STS(REG_1444H, 31, 1);
        WR1_PROG(REG_1420H, change_endian_long(0x00000000U));

        HW_SCE_p_func100(0x1afe63ceU, 0xd88eca4aU, 0xf27d3f5bU, 0xb9c3be74U);
        HW_SCE_p_func103();
        HW_SCE_p_func100(0x8de0a389U, 0xe4c924aaU, 0x56eefab3U, 0x635bcc39U);
        WR1_PROG(REG_1444H, 0x000000a2U);
        WR1_PROG(REG_1A24H, 0x0c200104U);
        WAIT_STS(REG_1444H, 31, 1);
        WR1_PROG(REG_1420H, change_endian_long(0x00000000U));

        WR1_PROG(REG_1408H, 0x00002012U);
        WAIT_STS(REG_1408H, 30, 1);
        RD4_ADDR(REG_1420H, &S_RAM[16]);
        S_RAM[16] = change_endian_long(S_RAM[16]);
        S_RAM[17] = change_endian_long(S_RAM[17]);
        S_RAM[18] = change_endian_long(S_RAM[18]);
        S_RAM[19] = change_endian_long(S_RAM[19]);

        HW_SCE_p_func100(0xb2c72868U, 0xae7edfcaU, 0x17c9d00bU, 0xf0d467aeU);
        HW_SCE_p_func103();
        WR1_PROG(REG_1444H, 0x000000a2U);
        WR1_PROG(REG_1A24H, 0x0c200104U);
        WAIT_STS(REG_1444H, 31, 1);
        WR1_PROG(REG_1420H, change_endian_long(0x00000000U));

        WR1_PROG(REG_1404H, 0x20000000U);
        WR1_PROG(REG_1400H, 0x00c20005U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);
        WR1_PROG(REG_1400H, 0x0002000dU);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        WR1_PROG(REG_1404H, 0x10000000U);
        WR1_PROG(REG_1400H, 0x00c01001U);
        WAIT_STS(REG_1404H, 30, 0);
        WR1_PROG(REG_143CH, 0x00001800U);

        WR1_PROG(REG_1B00H, 0x00008002U);
        WR1_PROG(REG_1B08H, 0x00000d01U);

        WR1_PROG(REG_1B00H, 0x00008001U);

        WR1_PROG(REG_1B08H, 0x00000214U);

        HW_SCE_p_func102(0x5a5abf48U, 0x1a95a6a8U, 0xf67faab6U, 0x18846492U);
        WR1_PROG(REG_14BCH, 0x00000040U);
        WAIT_STS(REG_142CH, 12, 0);

        return FSP_SUCCESS;
    }
}
