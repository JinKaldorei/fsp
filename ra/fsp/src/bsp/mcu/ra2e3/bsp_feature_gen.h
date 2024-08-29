/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef BSP_FEATURE_GEN_H
#define BSP_FEATURE_GEN_H

/***********************************************************************************************************************
 * Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private global variables and functions
 **********************************************************************************************************************/

// *UNCRUSTIFY-OFF*
#define BSP_FEATURE_GPT_AD_DIRECT_START_CHANNEL_MASK  (0)
#define BSP_FEATURE_GPT_AD_DIRECT_START_SUPPORTED     (0)
#define BSP_FEATURE_GPT_EVENT_COUNT_CHANNEL_MASK      (0x3F0)
#define BSP_FEATURE_GPT_EVENT_COUNT_SUPPORTED         (1)
#define BSP_FEATURE_GPT_GPTE_CHANNEL_MASK             (0)
#define BSP_FEATURE_GPT_GPTE_SUPPORTED                (0)
#define BSP_FEATURE_GPT_GPTEH_CHANNEL_MASK            (0)
#define BSP_FEATURE_GPT_GPTEH_SUPPORTED               (0)
#define BSP_FEATURE_GPT_GTDVU_CHANNEL_MASK            (0x3F0)
#define BSP_FEATURE_GPT_GTDVU_SUPPORTED               (1)
#define BSP_FEATURE_GPT_OPS_CHANNEL_MASK              (0)
#define BSP_FEATURE_GPT_OPS_SUPPORTED                 (0)

#define BSP_PERIPHERAL_ACMP_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_ACMP_PRESENT                   (0)
#define BSP_PERIPHERAL_ACMPHS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_ACMPHS_PRESENT                 (0)
#define BSP_PERIPHERAL_ACMPHS_NS_CHANNEL_MASK         (0)
#define BSP_PERIPHERAL_ACMPHS_NS_PRESENT              (0)
#define BSP_PERIPHERAL_ACMPLP_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_ACMPLP_PRESENT                 (0)
#define BSP_PERIPHERAL_ADC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_ADC_PRESENT                    (1)
#define BSP_PERIPHERAL_ADC_B_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_ADC_B_PRESENT                  (0)
#define BSP_PERIPHERAL_ADC_D_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_ADC_D_PRESENT                  (0)
#define BSP_PERIPHERAL_AES_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_AES_PRESENT                    (0)
#define BSP_PERIPHERAL_AGT_CHANNEL_MASK               (0x3)
#define BSP_PERIPHERAL_AGT_PRESENT                    (1)
#define BSP_PERIPHERAL_AGTW_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_AGTW_PRESENT                   (0)
#define BSP_PERIPHERAL_AMI_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_AMI_PRESENT                    (0)
#define BSP_PERIPHERAL_ANALOG_CHANNEL_MASK            (0x1)
#define BSP_PERIPHERAL_ANALOG_PRESENT                 (1)
#define BSP_PERIPHERAL_BUS_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_BUS_PRESENT                    (1)
#define BSP_PERIPHERAL_BUS_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_BUS_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_CAC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_CAC_PRESENT                    (1)
#define BSP_PERIPHERAL_CAC_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_CAC_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_CACHE_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_CACHE_PRESENT                  (0)
#define BSP_PERIPHERAL_CAN_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_CAN_PRESENT                    (0)
#define BSP_PERIPHERAL_CANFD_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_CANFD_PRESENT                  (0)
#define BSP_PERIPHERAL_CEC_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_CEC_PRESENT                    (0)
#define BSP_PERIPHERAL_CEU_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_CEU_PRESENT                    (0)
#define BSP_PERIPHERAL_CEU_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_CEU_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_CGC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_CGC_PRESENT                    (1)
#define BSP_PERIPHERAL_CPSCU_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_CPSCU_PRESENT                  (0)
#define BSP_PERIPHERAL_CPSCU_NS_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_CPSCU_NS_PRESENT               (0)
#define BSP_PERIPHERAL_CPU_CTRL_NS_CHANNEL_MASK       (0)
#define BSP_PERIPHERAL_CPU_CTRL_NS_PRESENT            (0)
#define BSP_PERIPHERAL_CRC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_CRC_PRESENT                    (1)
#define BSP_PERIPHERAL_CRC_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_CRC_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_CTSU_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_CTSU_PRESENT                   (0)
#define BSP_PERIPHERAL_DAC8_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_DAC8_PRESENT                   (0)
#define BSP_PERIPHERAL_DAC12_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_DAC12_PRESENT                  (0)
#define BSP_PERIPHERAL_DAC_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_DAC_PRESENT                    (0)
#define BSP_PERIPHERAL_DEBUG_CHANNEL_MASK             (0x1)
#define BSP_PERIPHERAL_DEBUG_PRESENT                  (1)
#define BSP_PERIPHERAL_DEBUG_NS_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_DEBUG_NS_PRESENT               (0)
#define BSP_PERIPHERAL_DMA_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_DMA_PRESENT                    (0)
#define BSP_PERIPHERAL_DMA_DMAC_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_DMA_DMAC_PRESENT               (0)
#define BSP_PERIPHERAL_DMA_DMAC_NS_CHANNEL_MASK       (0)
#define BSP_PERIPHERAL_DMA_DMAC_NS_PRESENT            (0)
#define BSP_PERIPHERAL_DMA_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_DMA_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_DOC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_DOC_PRESENT                    (1)
#define BSP_PERIPHERAL_DOC_B_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_DOC_B_PRESENT                  (0)
#define BSP_PERIPHERAL_DOC_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_DOC_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_DPHYCNT_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_DPHYCNT_PRESENT                (0)
#define BSP_PERIPHERAL_DPHYCNT_NS_CHANNEL_MASK        (0)
#define BSP_PERIPHERAL_DPHYCNT_NS_PRESENT             (0)
#define BSP_PERIPHERAL_DRW_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_DRW_PRESENT                    (0)
#define BSP_PERIPHERAL_DRW_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_DRW_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_DSILINK_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_DSILINK_PRESENT                (0)
#define BSP_PERIPHERAL_DSILINK_NS_CHANNEL_MASK        (0)
#define BSP_PERIPHERAL_DSILINK_NS_PRESENT             (0)
#define BSP_PERIPHERAL_DTC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_DTC_PRESENT                    (1)
#define BSP_PERIPHERAL_DTC_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_DTC_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_ECCAFL_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_ECCAFL_PRESENT                 (0)
#define BSP_PERIPHERAL_ECCMB_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_ECCMB_PRESENT                  (0)
#define BSP_PERIPHERAL_ECCMB_NS_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_ECCMB_NS_PRESENT               (0)
#define BSP_PERIPHERAL_ELC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_ELC_PRESENT                    (1)
#define BSP_PERIPHERAL_ELC_B_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_ELC_B_PRESENT                  (0)
#define BSP_PERIPHERAL_ELC_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_ELC_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_ETHERC_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_ETHERC_PRESENT                 (0)
#define BSP_PERIPHERAL_ETHERC_EDMAC_CHANNEL_MASK      (0)
#define BSP_PERIPHERAL_ETHERC_EDMAC_PRESENT           (0)
#define BSP_PERIPHERAL_ETHERC_EDMAC_NS_CHANNEL_MASK   (0)
#define BSP_PERIPHERAL_ETHERC_EDMAC_NS_PRESENT        (0)
#define BSP_PERIPHERAL_ETHERC_EPTPC_CHANNEL_MASK      (0)
#define BSP_PERIPHERAL_ETHERC_EPTPC_PRESENT           (0)
#define BSP_PERIPHERAL_ETHERC_EPTPC_CFG_CHANNEL_MASK  (0)
#define BSP_PERIPHERAL_ETHERC_EPTPC_CFG_PRESENT       (0)
#define BSP_PERIPHERAL_ETHERC_MII_CHANNEL_MASK        (0)
#define BSP_PERIPHERAL_ETHERC_MII_PRESENT             (0)
#define BSP_PERIPHERAL_ETHERC_NS_CHANNEL_MASK         (0)
#define BSP_PERIPHERAL_ETHERC_NS_PRESENT              (0)
#define BSP_PERIPHERAL_ETHERC_RMII_CHANNEL_MASK       (0)
#define BSP_PERIPHERAL_ETHERC_RMII_PRESENT            (0)
#define BSP_PERIPHERAL_FACI_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_FACI_PRESENT                   (0)
#define BSP_PERIPHERAL_FACI_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_FACI_NS_PRESENT                (0)
#define BSP_PERIPHERAL_FCACHE_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_FCACHE_PRESENT                 (0)
#define BSP_PERIPHERAL_FCACHE_NS_CHANNEL_MASK         (0)
#define BSP_PERIPHERAL_FCACHE_NS_PRESENT              (0)
#define BSP_PERIPHERAL_FLAD_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_FLAD_PRESENT                   (0)
#define BSP_PERIPHERAL_FLAD_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_FLAD_NS_PRESENT                (0)
#define BSP_PERIPHERAL_FLASH_CHANNEL_MASK             (0x1)
#define BSP_PERIPHERAL_FLASH_PRESENT                  (1)
#define BSP_PERIPHERAL_FLASH_HP_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_FLASH_HP_PRESENT               (0)
#define BSP_PERIPHERAL_FLASH_LP_CHANNEL_MASK          (0x1)
#define BSP_PERIPHERAL_FLASH_LP_PRESENT               (1)
#define BSP_PERIPHERAL_GLCDC_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_GLCDC_PRESENT                  (0)
#define BSP_PERIPHERAL_GLCDC_NS_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_GLCDC_NS_PRESENT               (0)
#define BSP_PERIPHERAL_GPT_CHANNEL_MASK               (0x3F1)
#define BSP_PERIPHERAL_GPT_PRESENT                    (1)
#define BSP_PERIPHERAL_GPT_GTCLK_CHANNEL_MASK         (0)
#define BSP_PERIPHERAL_GPT_GTCLK_PRESENT              (0)
#define BSP_PERIPHERAL_GPT_ODC_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_GPT_ODC_PRESENT                (0)
#define BSP_PERIPHERAL_GPT_OPS_CHANNEL_MASK           (0x1)
#define BSP_PERIPHERAL_GPT_OPS_PRESENT                (1)
#define BSP_PERIPHERAL_GPT_OPS_NS_CHANNEL_MASK        (0)
#define BSP_PERIPHERAL_GPT_OPS_NS_PRESENT             (0)
#define BSP_PERIPHERAL_GPT_POEG_CHANNEL_MASK          (0x3)
#define BSP_PERIPHERAL_GPT_POEG_PRESENT               (1)
#define BSP_PERIPHERAL_GPT_POEG_NS_CHANNEL_MASK       (0)
#define BSP_PERIPHERAL_GPT_POEG_NS_PRESENT            (0)
#define BSP_PERIPHERAL_I3C_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_I3C_PRESENT                    (0)
#define BSP_PERIPHERAL_I3C_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_I3C_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_ICU_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_ICU_PRESENT                    (1)
#define BSP_PERIPHERAL_ICU_EXT_IRQ_CHANNEL_MASK       (0xFF)
#define BSP_PERIPHERAL_ICU_EXT_IRQ_PRESENT            (1)
#define BSP_PERIPHERAL_ICU_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_ICU_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_IIC0WU_CHANNEL_MASK            (0x1)
#define BSP_PERIPHERAL_IIC0WU_PRESENT                 (1)
#define BSP_PERIPHERAL_IIC0WU_B_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_IIC0WU_B_PRESENT               (0)
#define BSP_PERIPHERAL_IIC0WU_NS_CHANNEL_MASK         (0)
#define BSP_PERIPHERAL_IIC0WU_NS_PRESENT              (0)
#define BSP_PERIPHERAL_IIC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_IIC_PRESENT                    (1)
#define BSP_PERIPHERAL_IIC_B_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_IIC_B_PRESENT                  (0)
#define BSP_PERIPHERAL_IIC_B_NS_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_IIC_B_NS_PRESENT               (0)
#define BSP_PERIPHERAL_IIC_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_IIC_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_IICA_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_IICA_PRESENT                   (0)
#define BSP_PERIPHERAL_IIRFA_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_IIRFA_PRESENT                  (0)
#define BSP_PERIPHERAL_IRDA_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_IRDA_PRESENT                   (0)
#define BSP_PERIPHERAL_IRTC_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_IRTC_PRESENT                   (0)
#define BSP_PERIPHERAL_IWDT_CHANNEL_MASK              (0x1)
#define BSP_PERIPHERAL_IWDT_PRESENT                   (1)
#define BSP_PERIPHERAL_IWDT_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_IWDT_NS_PRESENT                (0)
#define BSP_PERIPHERAL_JPEG_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_JPEG_PRESENT                   (0)
#define BSP_PERIPHERAL_KINT_CHANNEL_MASK              (0x1)
#define BSP_PERIPHERAL_KINT_PRESENT                   (1)
#define BSP_PERIPHERAL_MACL_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_MACL_PRESENT                   (0)
#define BSP_PERIPHERAL_MIPI_DSI_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_MIPI_DSI_PRESENT               (0)
#define BSP_PERIPHERAL_MMF_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_MMF_PRESENT                    (0)
#define BSP_PERIPHERAL_MPU_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_MPU_PRESENT                    (1)
#define BSP_PERIPHERAL_MPU_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_MPU_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_MSTP_CHANNEL_MASK              (0x1)
#define BSP_PERIPHERAL_MSTP_PRESENT                   (1)
#define BSP_PERIPHERAL_MSTP_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_MSTP_NS_PRESENT                (0)
#define BSP_PERIPHERAL_OCD_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_OCD_PRESENT                    (0)
#define BSP_PERIPHERAL_OCD_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_OCD_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_OPAMP_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_OPAMP_PRESENT                  (0)
#define BSP_PERIPHERAL_OSPI_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_OSPI_PRESENT                   (0)
#define BSP_PERIPHERAL_OSPI_B_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_OSPI_B_PRESENT                 (0)
#define BSP_PERIPHERAL_OSPI_B_NS_CHANNEL_MASK         (0)
#define BSP_PERIPHERAL_OSPI_B_NS_PRESENT              (0)
#define BSP_PERIPHERAL_PCLBUZ_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_PCLBUZ_PRESENT                 (0)
#define BSP_PERIPHERAL_PDC_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_PDC_PRESENT                    (0)
#define BSP_PERIPHERAL_PDG_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_PDG_PRESENT                    (0)
#define BSP_PERIPHERAL_PFS_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_PFS_PRESENT                    (1)
#define BSP_PERIPHERAL_PFS_B_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_PFS_B_PRESENT                  (0)
#define BSP_PERIPHERAL_PFS_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_PFS_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_PMISC_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_PMISC_PRESENT                  (0)
#define BSP_PERIPHERAL_PORGA_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_PORGA_PRESENT                  (0)
#define BSP_PERIPHERAL_PORT_CHANNEL_MASK              (0x23F)
#define BSP_PERIPHERAL_PORT_PRESENT                   (1)
#define BSP_PERIPHERAL_PORT_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_PORT_NS_PRESENT                (0)
#define BSP_PERIPHERAL_PSCU_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_PSCU_PRESENT                   (0)
#define BSP_PERIPHERAL_PSCU_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_PSCU_NS_PRESENT                (0)
#define BSP_PERIPHERAL_PTPEDMAC_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_PTPEDMAC_PRESENT               (0)
#define BSP_PERIPHERAL_QSPI_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_QSPI_PRESENT                   (0)
#define BSP_PERIPHERAL_RADIO_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_RADIO_PRESENT                  (0)
#define BSP_PERIPHERAL_RTC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_RTC_PRESENT                    (1)
#define BSP_PERIPHERAL_RTC_C_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_RTC_C_PRESENT                  (0)
#define BSP_PERIPHERAL_RTC_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_RTC_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_SAU_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_SAU_PRESENT                    (0)
#define BSP_PERIPHERAL_SAU_I2C_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_SAU_I2C_PRESENT                (0)
#define BSP_PERIPHERAL_SAU_SPI_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_SAU_SPI_PRESENT                (0)
#define BSP_PERIPHERAL_SAU_UART_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_SAU_UART_PRESENT               (0)
#define BSP_PERIPHERAL_SCE5_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_SCE5_PRESENT                   (0)
#define BSP_PERIPHERAL_SCE7_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_SCE7_PRESENT                   (0)
#define BSP_PERIPHERAL_SCE9_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_SCE9_PRESENT                   (0)
#define BSP_PERIPHERAL_SCE_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_SCE_PRESENT                    (0)
#define BSP_PERIPHERAL_SCI_CHANNEL_MASK               (0x207)
#define BSP_PERIPHERAL_SCI_PRESENT                    (1)
#define BSP_PERIPHERAL_SCI_B_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_SCI_B_PRESENT                  (0)
#define BSP_PERIPHERAL_SCI_B_NS_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_SCI_B_NS_PRESENT               (0)
#define BSP_PERIPHERAL_SDADC_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_SDADC_PRESENT                  (0)
#define BSP_PERIPHERAL_SDADC_B_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_SDADC_B_PRESENT                (0)
#define BSP_PERIPHERAL_SDHI_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_SDHI_PRESENT                   (0)
#define BSP_PERIPHERAL_SDHI_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_SDHI_NS_PRESENT                (0)
#define BSP_PERIPHERAL_SLCDC_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_SLCDC_PRESENT                  (0)
#define BSP_PERIPHERAL_SPI_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_SPI_PRESENT                    (1)
#define BSP_PERIPHERAL_SPI_B_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_SPI_B_PRESENT                  (0)
#define BSP_PERIPHERAL_SPI_B_NS_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_SPI_B_NS_PRESENT               (0)
#define BSP_PERIPHERAL_SPMON_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_SPMON_PRESENT                  (0)
#define BSP_PERIPHERAL_SRAM_CHANNEL_MASK              (0x1)
#define BSP_PERIPHERAL_SRAM_PRESENT                   (1)
#define BSP_PERIPHERAL_SRAM_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_SRAM_NS_PRESENT                (0)
#define BSP_PERIPHERAL_SRC_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_SRC_PRESENT                    (0)
#define BSP_PERIPHERAL_SRCRAM_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_SRCRAM_PRESENT                 (0)
#define BSP_PERIPHERAL_SSI_COMMON_CHANNEL_MASK        (0)
#define BSP_PERIPHERAL_SSI_COMMON_PRESENT             (0)
#define BSP_PERIPHERAL_SSIE_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_SSIE_PRESENT                   (0)
#define BSP_PERIPHERAL_SSIE_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_SSIE_NS_PRESENT                (0)
#define BSP_PERIPHERAL_SYSTEM_CHANNEL_MASK            (0x1)
#define BSP_PERIPHERAL_SYSTEM_PRESENT                 (1)
#define BSP_PERIPHERAL_SYSTEM_NS_CHANNEL_MASK         (0)
#define BSP_PERIPHERAL_SYSTEM_NS_PRESENT              (0)
#define BSP_PERIPHERAL_TAU_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_TAU_PRESENT                    (0)
#define BSP_PERIPHERAL_TFU_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_TFU_PRESENT                    (0)
#define BSP_PERIPHERAL_TML_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_TML_PRESENT                    (0)
#define BSP_PERIPHERAL_TRNG_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_TRNG_PRESENT                   (0)
#define BSP_PERIPHERAL_TSD_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_TSD_PRESENT                    (0)
#define BSP_PERIPHERAL_TSD_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_TSD_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_TSN_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_TSN_PRESENT                    (0)
#define BSP_PERIPHERAL_TSN_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_TSN_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_TZF_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_TZF_PRESENT                    (0)
#define BSP_PERIPHERAL_TZF_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_TZF_NS_PRESENT                 (0)
#define BSP_PERIPHERAL_UARTA_CHANNEL_MASK             (0)
#define BSP_PERIPHERAL_UARTA_PRESENT                  (0)
#define BSP_PERIPHERAL_UARTA_CK_CHANNEL_MASK          (0)
#define BSP_PERIPHERAL_UARTA_CK_PRESENT               (0)
#define BSP_PERIPHERAL_ULPT_CHANNEL_MASK              (0)
#define BSP_PERIPHERAL_ULPT_PRESENT                   (0)
#define BSP_PERIPHERAL_ULPT_NS_CHANNEL_MASK           (0)
#define BSP_PERIPHERAL_ULPT_NS_PRESENT                (0)
#define BSP_PERIPHERAL_USB_CHANNEL_MASK               (0)
#define BSP_PERIPHERAL_USB_PRESENT                    (0)
#define BSP_PERIPHERAL_USB_FS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_USB_FS_PRESENT                 (0)
#define BSP_PERIPHERAL_USB_HS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_USB_HS_PRESENT                 (0)
#define BSP_PERIPHERAL_WDT_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_WDT_PRESENT                    (1)
#define BSP_PERIPHERAL_WDT_NS_CHANNEL_MASK            (0)
#define BSP_PERIPHERAL_WDT_NS_PRESENT                 (0)
// *UNCRUSTIFY-ON*
#endif
