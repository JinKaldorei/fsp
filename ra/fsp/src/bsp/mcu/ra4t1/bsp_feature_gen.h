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
#define BSP_PERIPHERAL_ACMP_PRESENT                   (0)
#define BSP_PERIPHERAL_ACMPHS_PRESENT                 (1)
#define BSP_PERIPHERAL_ACMPHS_CHANNEL_MASK            (0x7)
#define BSP_PERIPHERAL_ACMPLP_PRESENT                 (0)
#define BSP_PERIPHERAL_ADC_PRESENT                    (1)
#define BSP_PERIPHERAL_ADC_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_ADC_B_PRESENT                  (0)
#define BSP_PERIPHERAL_ADC_D_PRESENT                  (0)
#define BSP_PERIPHERAL_AGT_PRESENT                    (1)
#define BSP_PERIPHERAL_AGT_CHANNEL_MASK               (0x3)
#define BSP_PERIPHERAL_AGTW_PRESENT                   (0)
#define BSP_PERIPHERAL_AMI_PRESENT                    (0)
#define BSP_PERIPHERAL_ANALOG_PRESENT                 (1)
#define BSP_PERIPHERAL_BUS_PRESENT                    (1)
#define BSP_PERIPHERAL_CAC_PRESENT                    (1)
#define BSP_PERIPHERAL_CACHE_PRESENT                  (1)
#define BSP_PERIPHERAL_CAN_PRESENT                    (0)
#define BSP_PERIPHERAL_CANFD_PRESENT                  (1)
#define BSP_PERIPHERAL_CANFD_CHANNEL_MASK             (0x1)
#define BSP_PERIPHERAL_CEC_PRESENT                    (0)
#define BSP_PERIPHERAL_CEU_PRESENT                    (0)
#define BSP_PERIPHERAL_CGC_PRESENT                    (1)
#define BSP_PERIPHERAL_CPSCU_PRESENT                  (1)
#define BSP_PERIPHERAL_CPU_CTRL_PRESENT               (0)
#define BSP_PERIPHERAL_CRC_PRESENT                    (1)
#define BSP_PERIPHERAL_CTSU_PRESENT                   (0)
#define BSP_PERIPHERAL_DAC_PRESENT                    (1)
#define BSP_PERIPHERAL_DAC_CHANNEL_MASK               (0x3)
#define BSP_PERIPHERAL_DAC12_PRESENT                  (1)
#define BSP_PERIPHERAL_DAC12_CHANNEL_MASK             (0x3)
#define BSP_PERIPHERAL_DAC8_PRESENT                   (0)
#define BSP_PERIPHERAL_DEBUG_PRESENT                  (1)
#define BSP_PERIPHERAL_DMA_DMAC_PRESENT               (1)
#define BSP_PERIPHERAL_DMA_DMAC_CHANNEL_MASK          (0xFF)
#define BSP_PERIPHERAL_DMA_PRESENT                    (1)
#define BSP_PERIPHERAL_DOC_PRESENT                    (1)
#define BSP_PERIPHERAL_DOC_B_PRESENT                  (0)
#define BSP_PERIPHERAL_DPHYCNT_PRESENT                (0)
#define BSP_PERIPHERAL_DRW_PRESENT                    (0)
#define BSP_PERIPHERAL_DSILINK_PRESENT                (0)
#define BSP_PERIPHERAL_DTC_PRESENT                    (1)
#define BSP_PERIPHERAL_ECCAFL_PRESENT                 (0)
#define BSP_PERIPHERAL_ECCMB_PRESENT                  (0)
#define BSP_PERIPHERAL_ELC_PRESENT                    (1)
#define BSP_PERIPHERAL_ELC_B_PRESENT                  (0)
#define BSP_PERIPHERAL_ETHERC_EDMAC_PRESENT           (0)
#define BSP_PERIPHERAL_ETHERC_EPTPC_PRESENT           (0)
#define BSP_PERIPHERAL_ETHERC_EPTPC_CFG_PRESENT       (0)
#define BSP_PERIPHERAL_ETHERC_MII_PRESENT             (0)
#define BSP_PERIPHERAL_ETHERC_RMII_PRESENT            (0)
#define BSP_PERIPHERAL_ETHERC_PRESENT                 (0)
#define BSP_PERIPHERAL_FACI_PRESENT                   (1)
#define BSP_PERIPHERAL_FCACHE_PRESENT                 (1)
#define BSP_PERIPHERAL_FLAD_PRESENT                   (1)
#define BSP_PERIPHERAL_FLASH_HP_PRESENT               (1)
#define BSP_PERIPHERAL_FLASH_LP_PRESENT               (0)
#define BSP_PERIPHERAL_FLASH_PRESENT                  (1)
#define BSP_PERIPHERAL_GLCDC_PRESENT                  (0)
#define BSP_PERIPHERAL_GPT_GTCLK_PRESENT              (0)
#define BSP_PERIPHERAL_GPT_ODC_PRESENT                (0)
#define BSP_FEATURE_GPT_ODC_128_RESOLUTION_SUPPORTED  (0)
#define BSP_PERIPHERAL_GPT_OPS_PRESENT                (1)
#define BSP_PERIPHERAL_GPT_POEG_PRESENT               (1)
#define BSP_PERIPHERAL_GPT_POEG_CHANNEL_MASK          (0xF)
#define BSP_PERIPHERAL_GPT_PRESENT                    (1)
#define BSP_PERIPHERAL_GPT_CHANNEL_MASK               (0x3F)
#define BSP_FEATURE_GPT_AD_DIRECT_START_SUPPORTED     (0)
#define BSP_FEATURE_GPT_EVENT_COUNT_SUPPORTED         (1)
#define BSP_FEATURE_GPT_EVENT_COUNT_CHANNEL_MASK      (0x3F)
#define BSP_FEATURE_GPT_GPTE_SUPPORTED                (1)
#define BSP_FEATURE_GPT_GPTE_CHANNEL_MASK             (0x3F)
#define BSP_FEATURE_GPT_GPTEH_SUPPORTED               (0)
#define BSP_FEATURE_GPT_GTDVU_SUPPORTED               (1)
#define BSP_FEATURE_GPT_GTDVU_CHANNEL_MASK            (0x3F)
#define BSP_FEATURE_GPT_OPS_SUPPORTED                 (1)
#define BSP_FEATURE_GPT_OPS_CHANNEL_MASK              (0x1)
#define BSP_PERIPHERAL_TAU_PRESENT                    (0)
#define BSP_PERIPHERAL_TML_PRESENT                    (0)
#define BSP_PERIPHERAL_I3C_PRESENT                    (1)
#define BSP_PERIPHERAL_I3C_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_ICU_EXT_IRQ_PRESENT            (1)
#define BSP_PERIPHERAL_ICU_EXT_IRQ_CHANNEL_MASK       (0x7FFF)
#define BSP_PERIPHERAL_ICU_PRESENT                    (1)
#define BSP_PERIPHERAL_IIC_PRESENT                    (0)
#define BSP_PERIPHERAL_IICA_PRESENT                   (0)
#define BSP_PERIPHERAL_IIC_B_PRESENT                  (1)
#define BSP_PERIPHERAL_IIC_B_CHANNEL_MASK             (0x1)
#define BSP_PERIPHERAL_IIC0WU_PRESENT                 (0)
#define BSP_PERIPHERAL_IIC0WU_B_PRESENT               (0)
#define BSP_PERIPHERAL_TFU_PRESENT                    (0)
#define BSP_PERIPHERAL_IIRFA_PRESENT                  (0)
#define BSP_PERIPHERAL_IRDA_PRESENT                   (0)
#define BSP_PERIPHERAL_IWDT_PRESENT                   (1)
#define BSP_PERIPHERAL_JPEG_PRESENT                   (0)
#define BSP_PERIPHERAL_KINT_PRESENT                   (0)
#define BSP_PERIPHERAL_MACL_PRESENT                   (0)
#define BSP_PERIPHERAL_MMF_PRESENT                    (0)
#define BSP_PERIPHERAL_MIPI_DSI_PRESENT               (0)
#define BSP_PERIPHERAL_MPU_PRESENT                    (1)
#define BSP_PERIPHERAL_MPU_CHANNEL_MASK               (0x1)
#define BSP_PERIPHERAL_MSTP_PRESENT                   (1)
#define BSP_PERIPHERAL_OCD_PRESENT                    (0)
#define BSP_PERIPHERAL_OPAMP_PRESENT                  (0)
#define BSP_PERIPHERAL_OSPI_PRESENT                   (0)
#define BSP_PERIPHERAL_OSPI_B_PRESENT                 (0)
#define BSP_PERIPHERAL_PCLBUZ_PRESENT                 (0)
#define BSP_PERIPHERAL_PDC_PRESENT                    (0)
#define BSP_PERIPHERAL_PFS_PRESENT                    (1)
#define BSP_PERIPHERAL_PFS_B_PRESENT                  (0)
#define BSP_PERIPHERAL_PORT_PRESENT                   (1)
#define BSP_PERIPHERAL_PORT_CHANNEL_MASK              (0x13F)
#define BSP_PERIPHERAL_PORGA_PRESENT                  (0)
#define BSP_PERIPHERAL_PMISC_PRESENT                  (0)
#define BSP_PERIPHERAL_PSCU_PRESENT                   (1)
#define BSP_PERIPHERAL_PTPEDMAC_PRESENT               (0)
#define BSP_PERIPHERAL_QSPI_PRESENT                   (0)
#define BSP_PERIPHERAL_RADIO_PRESENT                  (0)
#define BSP_PERIPHERAL_RTC_PRESENT                    (0)
#define BSP_PERIPHERAL_RTC_C_PRESENT                  (0)
#define BSP_PERIPHERAL_IRTC_PRESENT                   (0)
#define BSP_PERIPHERAL_RSIP_PRESENT                   (1)
#define BSP_FEATURE_RSIP_AES_SUPPORTED                (0)
#define BSP_FEATURE_RSIP_AES_B_SUPPORTED              (0)
#define BSP_FEATURE_RSIP_RSIP_E11A_SUPPORTED          (0)
#define BSP_FEATURE_RSIP_RSIP_E50D_SUPPORTED          (0)
#define BSP_FEATURE_RSIP_RSIP_E51A_SUPPORTED          (0)
#define BSP_FEATURE_RSIP_SCE5_SUPPORTED               (0)
#define BSP_FEATURE_RSIP_SCE5B_SUPPORTED              (0)
#define BSP_FEATURE_RSIP_SCE7_SUPPORTED               (0)
#define BSP_FEATURE_RSIP_SCE9_SUPPORTED               (0)
#define BSP_FEATURE_RSIP_TRNG_SUPPORTED               (1)
#define BSP_PERIPHERAL_SCI_PRESENT                    (1)
#define BSP_PERIPHERAL_SCI_CHANNEL_MASK               (0x201)
#define BSP_FEATURE_SCI_IRDA_SUPPORTED                (0)
#define BSP_PERIPHERAL_SCI_B_PRESENT                  (0)
#define BSP_PERIPHERAL_SAU_PRESENT                    (0)
#define BSP_PERIPHERAL_SAU_SPI_PRESENT                (0)
#define BSP_PERIPHERAL_SAU_UART_PRESENT               (0)
#define BSP_PERIPHERAL_SAU_I2C_PRESENT                (0)
#define BSP_PERIPHERAL_SDADC_PRESENT                  (0)
#define BSP_PERIPHERAL_SDADC_B_PRESENT                (0)
#define BSP_PERIPHERAL_SDHI_PRESENT                   (0)
#define BSP_PERIPHERAL_SLCDC_PRESENT                  (0)
#define BSP_PERIPHERAL_SPI_PRESENT                    (1)
#define BSP_PERIPHERAL_SPI_CHANNEL_MASK               (0x3)
#define BSP_PERIPHERAL_SPI_B_PRESENT                  (0)
#define BSP_PERIPHERAL_SPMON_PRESENT                  (0)
#define BSP_PERIPHERAL_SRAM_PRESENT                   (1)
#define BSP_PERIPHERAL_SRCRAM_PRESENT                 (0)
#define BSP_PERIPHERAL_SRC_PRESENT                    (0)
#define BSP_PERIPHERAL_SSI_COMMON_PRESENT             (0)
#define BSP_PERIPHERAL_SSIE_PRESENT                   (0)
#define BSP_PERIPHERAL_SYSTEM_PRESENT                 (1)
#define BSP_PERIPHERAL_TRNG_PRESENT                   (0)
#define BSP_PERIPHERAL_TSD_PRESENT                    (1)
#define BSP_PERIPHERAL_TSN_PRESENT                    (1)
#define BSP_PERIPHERAL_TZF_PRESENT                    (1)
#define BSP_PERIPHERAL_UARTA_PRESENT                  (0)
#define BSP_PERIPHERAL_UARTA_CK_PRESENT               (0)
#define BSP_PERIPHERAL_ULPT_PRESENT                   (0)
#define BSP_PERIPHERAL_USB_FS_PRESENT                 (0)
#define BSP_PERIPHERAL_USB_HS_PRESENT                 (0)
#define BSP_PERIPHERAL_USB_PRESENT                    (0)
#define BSP_PERIPHERAL_WDT_PRESENT                    (1)
#define BSP_PERIPHERAL_WDT_CHANNEL_MASK               (0x1)
// *UNCRUSTIFY-ON*
#endif
