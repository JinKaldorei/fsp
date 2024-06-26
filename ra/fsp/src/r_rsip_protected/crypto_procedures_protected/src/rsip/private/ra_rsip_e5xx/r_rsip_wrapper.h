/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef R_RSIP_WRAPPER_H
#define R_RSIP_WRAPPER_H

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/* Key update inst data word */
#define RSIP_OEM_KEY_SIZE_DUMMY_INST_DATA_WORD                        (0)
#define RSIP_OEM_KEY_SIZE_AES128_INST_DATA_WORD                       (8)
#define RSIP_OEM_KEY_SIZE_AES256_INST_DATA_WORD                       (12)
#define RSIP_OEM_KEY_SIZE_AES128_XTS_INST_DATA_WORD                   (12)
#define RSIP_OEM_KEY_SIZE_AES256_XTS_INST_DATA_WORD                   (20)

#define RSIP_OEM_KEY_SIZE_ECC_SECP256R1_PUBLIC_KEY_INST_DATA_WORD     (20)
#define RSIP_OEM_KEY_SIZE_ECC_SECP256R1_PRIVATE_KEY_INST_DATA_WORD    (12)
#define RSIP_OEM_KEY_SIZE_ECC_SECP384R1_PUBLIC_KEY_INST_DATA_WORD     (28)
#define RSIP_OEM_KEY_SIZE_ECC_SECP384R1_PRIVATE_KEY_INST_DATA_WORD    (16)
#define RSIP_OEM_KEY_SIZE_ECC_SECP521R1_PUBLIC_KEY_INST_DATA_WORD     (44)
#define RSIP_OEM_KEY_SIZE_ECC_SECP521R1_PRIVATE_KEY_INST_DATA_WORD    (24)

#define RSIP_OEM_KEY_SIZE_HMAC_SHA256_KEY_INST_DATA_WORD              (12)

#define RSIP_OEM_KEY_SIZE_RSA2048_PUBLIC_KEY_INST_DATA_WORD           (72)
#define RSIP_OEM_KEY_SIZE_RSA2048_PRIVATE_KEY_INST_DATA_WORD          (132)
#define RSIP_OEM_KEY_SIZE_RSA3072_PUBLIC_KEY_INST_DATA_WORD           (104)
#define RSIP_OEM_KEY_SIZE_RSA3072_PRIVATE_KEY_INST_DATA_WORD          (196)
#define RSIP_OEM_KEY_SIZE_RSA4096_PUBLIC_KEY_INST_DATA_WORD           (136)
#define RSIP_OEM_KEY_SIZE_RSA4096_PRIVATE_KEY_INST_DATA_WORD          (260)

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

typedef enum e_rsip_ecc_curve_type
{
    RSIP_ECC_CURVE_TYPE_NIST
} rsip_ecc_type_t;

typedef enum e_rsip_ecc_cmd_type
{
    RSIP_ECC_CMD_P256
} rsip_ecc_cmd_type_t;

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
rsip_ret_t r_rsip_wrapper_pf0_secp256r1(const uint32_t InData_KeyIndex[],
                                        const uint32_t InData_MsgDgst[],
                                        uint32_t       OutData_Signature[]);
rsip_ret_t r_rsip_wrapper_pf1_secp256r1(const uint32_t InData_KeyIndex[],
                                        const uint32_t InData_MsgDgst[],
                                        const uint32_t InData_Signature[]);
rsip_ret_t r_rsip_wrapper_pf4_secp256r1(uint32_t OutData_PubKeyIndex[], uint32_t OutData_PrivKeyIndex[]);
rsip_ret_t r_rsip_wrapper_pf5_secp384r1(const uint32_t InData_KeyIndex[],
                                        const uint32_t InData_MsgDgst[],
                                        uint32_t       OutData_Signature[]);
rsip_ret_t r_rsip_wrapper_pf6_secp384r1(const uint32_t InData_KeyIndex[],
                                        const uint32_t InData_MsgDgst[],
                                        const uint32_t InData_Signature[]);
rsip_ret_t r_rsip_wrapper_pf9_secp384r1(uint32_t OutData_PubKeyIndex[], uint32_t OutData_PrivKeyIndex[]);
rsip_ret_t r_rsip_wrapper_p11_secp521r1(const uint32_t InData_KeyIndex[],
                                        const uint32_t InData_MsgDgst[],
                                        uint32_t       OutData_Signature[]);
rsip_ret_t r_rsip_wrapper_p12_secp521r1(const uint32_t InData_KeyIndex[],
                                        const uint32_t InData_MsgDgst[],
                                        const uint32_t InData_Signature[]);
rsip_ret_t r_rsip_wrapper_p13_secp521r1(uint32_t OutData_PubKeyIndex[], uint32_t OutData_PrivKeyIndex[]);
rsip_ret_t r_rsip_wrapper_p2b_rsa2048(uint32_t OutData_PubKeyIndex[], uint32_t OutData_PrivKeyIndex[]);
rsip_ret_t r_rsip_wrapper_p3a_rsa3072(uint32_t OutData_PubKeyIndex[], uint32_t OutData_PrivKeyIndex[]);
rsip_ret_t r_rsip_wrapper_p3b_rsa4096(uint32_t OutData_PubKeyIndex[], uint32_t OutData_PrivKeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_aes128(const uint32_t InData_IV[],
                                     const uint32_t InData_InstData[],
                                     uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_aes256(const uint32_t InData_IV[],
                                     const uint32_t InData_InstData[],
                                     uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_aes128xts(const uint32_t InData_IV[],
                                        const uint32_t InData_InstData[],
                                        uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_aes256xts(const uint32_t InData_IV[],
                                        const uint32_t InData_InstData[],
                                        uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_secp256r1_pub(const uint32_t InData_IV[],
                                            const uint32_t InData_InstData[],
                                            uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_secp256r1_priv(const uint32_t InData_IV[],
                                             const uint32_t InData_InstData[],
                                             uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_secp384r1_pub(const uint32_t InData_IV[],
                                            const uint32_t InData_InstData[],
                                            uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_secp384r1_priv(const uint32_t InData_IV[],
                                             const uint32_t InData_InstData[],
                                             uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_secp521r1_pub(const uint32_t InData_IV[],
                                            const uint32_t InData_InstData[],
                                            uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_secp521r1_priv(const uint32_t InData_IV[],
                                             const uint32_t InData_InstData[],
                                             uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_rsa2048_pub(const uint32_t InData_IV[],
                                          const uint32_t InData_InstData[],
                                          uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_rsa2048_priv(const uint32_t InData_IV[],
                                           const uint32_t InData_InstData[],
                                           uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_rsa3072_pub(const uint32_t InData_IV[],
                                          const uint32_t InData_InstData[],
                                          uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_rsa3072_priv(const uint32_t InData_IV[],
                                           const uint32_t InData_InstData[],
                                           uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_rsa4096_pub(const uint32_t InData_IV[],
                                          const uint32_t InData_InstData[],
                                          uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_rsa4096_priv(const uint32_t InData_IV[],
                                           const uint32_t InData_InstData[],
                                           uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p6f_hmacsha256(const uint32_t InData_IV[],
                                         const uint32_t InData_InstData[],
                                         uint32_t       OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p8f_aes128 (const uint32_t        InData_KeyIndex[],
                                      const rsip_key_type_t key_type,
                                      const uint32_t        InData_WrappedKeyIndex[],
                                      uint32_t              OutData_Text[]);
rsip_ret_t r_rsip_wrapper_p8f_aes256 (const uint32_t        InData_KeyIndex[],
                                      const rsip_key_type_t key_type,
                                      const uint32_t        InData_WrappedKeyIndex[],
                                      uint32_t              OutData_Text[]);
rsip_ret_t r_rsip_wrapper_p90_aes128 (const uint32_t        InData_KeyIndex[],
                                      const rsip_key_type_t key_type,
                                      const uint32_t        InData_Text[],
                                      uint32_t              OutData_KeyIndex[]);
rsip_ret_t r_rsip_wrapper_p90_aes256 (const uint32_t        InData_KeyIndex[],
                                      const rsip_key_type_t key_type,
                                      const uint32_t        InData_Text[],
                                      uint32_t              OutData_KeyIndex[]);

/* AES-ECB/CBC/CTR */
rsip_ret_t r_rsip_wrapper_p47i_aes128ecb_encrypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);
rsip_ret_t r_rsip_wrapper_p47i_aes128ecb_decrypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);
rsip_ret_t r_rsip_wrapper_p47i_aes128cbc_encrypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);
rsip_ret_t r_rsip_wrapper_p47i_aes128cbc_decrypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);
rsip_ret_t r_rsip_wrapper_p47i_aes128ctr_crypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);
rsip_ret_t r_rsip_wrapper_p50i_aes256ecb_encrypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);
rsip_ret_t r_rsip_wrapper_p50i_aes256ecb_decrypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);
rsip_ret_t r_rsip_wrapper_p50i_aes256cbc_encrypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);
rsip_ret_t r_rsip_wrapper_p50i_aes256cbc_decrypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);
rsip_ret_t r_rsip_wrapper_p50i_aes256ctr_crypt(const uint32_t InData_KeyIndex[], const uint32_t InData_IV[]);

/* AES-GCM */
rsip_ret_t r_rsip_wrapper_p29i_aes128gcm_encrypt(const uint32_t * InData_KeyIndex, const uint32_t * InData_IV);
rsip_ret_t r_rsip_wrapper_p32i_aes128gcm_decrypt(const uint32_t * InData_KeyIndex, const uint32_t * InData_IV);
rsip_ret_t r_rsip_wrapper_p34i_aes256gcm_encrypt(const uint32_t * InData_KeyIndex, const uint32_t * InData_IV);
rsip_ret_t r_rsip_wrapper_p36i_aes256gcm_decrypt(const uint32_t * InData_KeyIndex, const uint32_t * InData_IV);

/* AES-CCM */
rsip_ret_t r_rsip_wrapper_p95i_aes128ccm_encrypt(const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_TextLen,
                                                 const uint32_t * InData_IV,
                                                 const uint32_t * InData_Header,
                                                 uint32_t         Header_Len);
rsip_ret_t r_rsip_wrapper_p95f_aes128ccm_encrypt(const uint32_t * InData_Text,
                                                 const uint32_t * InData_TextLen,
                                                 uint32_t       * OutData_Text,
                                                 uint32_t       * OutData_MAC);
rsip_ret_t r_rsip_wrapper_p98i_aes128ccm_decrypt(const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_TextLen,
                                                 const uint32_t * InData_MACLength,
                                                 const uint32_t * InData_IV,
                                                 const uint32_t * InData_Header,
                                                 uint32_t         Header_Len);
rsip_ret_t r_rsip_wrapper_p98f_aes128ccm_decrypt(const uint32_t * InData_Text,
                                                 const uint32_t * InData_TextLen,
                                                 const uint32_t * InData_MAC,
                                                 const uint32_t * InData_MACLength,
                                                 uint32_t       * OutData_Text);

rsip_ret_t r_rsip_wrapper_pa1i_aes256ccm_encrypt(const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_TextLen,
                                                 const uint32_t * InData_IV,
                                                 const uint32_t * InData_Header,
                                                 uint32_t         Header_Len);
rsip_ret_t r_rsip_wrapper_pa4i_aes256ccm_decrypt(const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_TextLen,
                                                 const uint32_t * InData_MACLength,
                                                 const uint32_t * InData_IV,
                                                 const uint32_t * InData_Header,
                                                 uint32_t         Header_Len);

/* AES-MAC */
rsip_ret_t r_rsip_wrapper_p41i_aes128mac(const uint32_t * InData_KeyIndex);
rsip_ret_t r_rsip_wrapper_p41f_aes128mac_generate(const uint32_t * InData_Text,
                                                  uint32_t       * OutData_DataT,
                                                  const uint32_t   all_msg_len);
rsip_ret_t r_rsip_wrapper_p41f_aes128mac_verify(const uint32_t * InData_Text,
                                                const uint32_t * InData_DataT,
                                                const uint32_t * InData_DataTLen,
                                                const uint32_t   all_msg_len);
rsip_ret_t r_rsip_wrapper_p44i_aes256mac(const uint32_t * InData_KeyIndex);
rsip_ret_t r_rsip_wrapper_p44f_aes256mac_generate(const uint32_t * InData_Text,
                                                  uint32_t       * OutData_DataT,
                                                  const uint32_t   all_msg_len);
rsip_ret_t r_rsip_wrapper_p44f_aes256mac_verify(const uint32_t * InData_Text,
                                                const uint32_t * InData_DataT,
                                                const uint32_t * InData_DataTLen,
                                                const uint32_t   all_msg_len);

#endif                                 /* R_RSIP_WRAPPER_H */
