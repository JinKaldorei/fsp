/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include "r_sce.h"
#include "r_sce_private.h"

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

void R_SCE_func059(void)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    R_SCE_func100(0x360cd89eU, 0xcbf6aa10U, 0xf73f5dbbU, 0x7474f188U);
    SCE->REG_ECH = 0x00008d40U;
    SCE->REG_ECH = 0x00ffffffU;
    SCE->REG_ECH = 0x00009140U;
    SCE->REG_ECH = 0x01000000U;
    SCE->REG_104H = 0x00000052U;
    SCE->REG_C4H = 0x00442a0cU;
    /* WAIT_LOOP */
    while (1U != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = change_endian_long(0x00000000U);
    SCE->REG_C4H = 0x00082e0dU;
    SCE->REG_E0H = 0x81040140U;
    SCE->REG_00H = 0x00002813U;
    /* WAIT_LOOP */
    while (0U != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800U;
    R_SCE_func100(0xbe97f1d5U, 0xcf552fedU, 0xec1700ddU, 0x9edff943U);
    SCE->REG_ECH = 0x00008d40U;
    SCE->REG_ECH = 0x00ffffffU;
    SCE->REG_ECH = 0x00009140U;
    SCE->REG_ECH = 0x02000000U;
    SCE->REG_C4H = 0x00092e0dU;
    SCE->REG_E0H = 0x81040140U;
    SCE->REG_00H = 0x00002813U;
    /* WAIT_LOOP */
    while (0U != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800U;
    SCE->REG_ECH = 0x00007c1dU;
    SCE->REG_1CH = 0x00602000U;
}

/***********************************************************************************************************************
End of function ./input_dir/S6C1/Cryptographic/R_SCE_func059.prc
***********************************************************************************************************************/
