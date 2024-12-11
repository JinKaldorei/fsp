/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*******************************************************************************************************************//**
 * @addtogroup RSIP_PROTECTED
 * @{
 **********************************************************************************************************************/

#ifndef R_RSIP_H
#define R_RSIP_H

/* Register definitions, common services and error codes. */
#include "bsp_api.h"
#include "r_rsip_api.h"
#include "r_rsip_cfg.h"

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/* Version Number of Module. */
#define FSP_R_RSIP_PROTECTED_VERSION_MAJOR  (1U)
#define FSP_R_RSIP_PROTECTED_VERSION_MINOR  (5U)
#define FSP_R_RSIP_PROTECTED_VERSION_PATCH  (0U)

#define RSIP_PRV_BYTE_SIZE_AES_BLOCK               (16U)
#define RSIP_PRV_BYTE_SIZE_CCM_NONCE_MAX           (13U)
#define RSIP_PRV_BYTE_SIZE_CCM_AAD_MAX             (110U)

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/* Working area for AES block cipher */
typedef struct st_rsip_aes_cihper_handle
{
    const void * p_func;               // Pointer to primitive functions
} rsip_aes_cipher_handle_t;

/* Working area for AES-GCM */
typedef struct st_rsip_aes_gcm_handle
{
    const void * p_func;                               // Pointer to primitive functions
    uint8_t      buffer[RSIP_PRV_BYTE_SIZE_AES_BLOCK]; // Buffer for AAD and plaintext/ciphertext
    uint32_t     buffered_length;                      // Buffered AAD and plaintext/ciphertext length
    uint32_t     total_length;                         // Total plaintext/ciphertext length
    uint32_t     total_aad_length;                     // Total AAD length
} rsip_aes_gcm_handle_t;

/* Working area for AES-CCM */
typedef struct st_rsip_aes_ccm_handle
{
    const void * p_func;                                          // Pointer to primitive functions
    uint8_t      wrapped_key[RSIP_BYTE_SIZE_WRAPPED_KEY_AES_256]; // Stored wrapped key
    uint8_t      nonce_buffer[RSIP_PRV_BYTE_SIZE_CCM_NONCE_MAX];  // Buffer for nonce
    uint32_t     nonce_length;                                    // Nonce length
    uint8_t      buffer[RSIP_PRV_BYTE_SIZE_CCM_AAD_MAX];          // Buffer for AAD and plaintext/ciphertext
    uint32_t     buffered_length;                                 // Buffered AAD and plaintext/ciphertext length
    uint32_t     input_aad_length;                                // Input AAD length
    uint32_t     total_aad_length;                                // Total AAD length
    uint32_t     input_length;                                    // Input plaintext/ciphertext length
    uint32_t     total_length;                                    // Total plaintext/ciphertext length
    uint32_t     tag_length;                                      // Tag length
} rsip_aes_ccm_handle_t;

/* Working area for AES-CMAC */
typedef struct st_rsip_aes_cmac_handle
{
    const void * p_func;                               // Pointer to primitive functions
    uint8_t      buffer[RSIP_PRV_BYTE_SIZE_AES_BLOCK]; // Buffer for message
    uint32_t     buffered_length;                      // Buffered message length
    uint32_t     total_length;                         // Total message length
} rsip_aes_cmac_handle_t;

/* Working area for RSIP cryptographic algorithms. This union is included in private control block. */
typedef union u_rsip_handle
{
    rsip_aes_cipher_handle_t aes_cipher; // AES block cipher
    rsip_aes_gcm_handle_t    aes_gcm;    // AES-GCM
    rsip_aes_ccm_handle_t    aes_ccm;    // AES-CCM
    rsip_aes_cmac_handle_t   aes_cmac;   // AES-CMAC
    rsip_sha_handle_t        sha;        // SHA
    rsip_hmac_handle_t       hmac;       // HMAC
    rsip_kdf_hmac_handle_t   kdf_hmac;   // KDF HMAC
} rsip_handle_t;

/* State that specifies functions that can be called next */
typedef enum e_rsip_state
{
    RSIP_STATE_MAIN,                    // Main
    RSIP_STATE_AES_CIPHER,              // AES-ECB, AES-CBC, AES-CTR
    RSIP_STATE_AES_XTS_UPDATE,          // XTS-AES update
    RSIP_STATE_AES_XTS_FINISH,          // XTS-AES finalization
    RSIP_STATE_AES_GCM_ENC_UPDATE_AAD,  // AES-GCM encryption, AAD input
    RSIP_STATE_AES_GCM_ENC_UPDATE_TEXT, // AES-GCM encryption, plaintext/ciphertext input
    RSIP_STATE_AES_GCM_DEC_UPDATE_AAD,  // AES-GCM decryption, AAD input
    RSIP_STATE_AES_GCM_DEC_UPDATE_TEXT, // AES-GCM decryption, plaintext/ciphertext input
    RSIP_STATE_AES_CCM_ENC_SET_LENGTH,  // AES-CCM encryption, length setting
    RSIP_STATE_AES_CCM_ENC_UPDATE_AAD,  // AES-CCM encryption, AAD input
    RSIP_STATE_AES_CCM_ENC_UPDATE_TEXT, // AES-CCM encryption, plaintext/ciphertext input
    RSIP_STATE_AES_CCM_ENC_FINISH,      // AES-CCM encryption, finish
    RSIP_STATE_AES_CCM_DEC_SET_LENGTH,  // AES-CCM decryption, length setting
    RSIP_STATE_AES_CCM_DEC_UPDATE_AAD,  // AES-CCM decryption, AAD input
    RSIP_STATE_AES_CCM_DEC_UPDATE_TEXT, // AES-CCM decryption, plaintext/ciphertext input
    RSIP_STATE_AES_CCM_DEC_VERI,        // AES-CCM encryption, verification
    RSIP_STATE_AES_CMAC,                // AES-CMAC
    RSIP_STATE_SHA,                     // SHA
    RSIP_STATE_HMAC,                    // HMAC
    RSIP_STATE_KDF_HMAC,                // KDF HMAC
} rsip_state_t;

/** RSIP private control block. DO NOT MODIFY. Initialization occurs when R_RSIP_Open() is called. */
typedef struct st_rsip_instance_ctrl
{
    uint32_t           open;                          // Indicates whether the open() API has been successfully
    rsip_cfg_t const * p_cfg;                         // Pointer to the configuration block
    rsip_handle_t      handle;                        // Handle of algorithms that cannot be suspended
    rsip_state_t       state;                         // Flags to limit the next API to call

    rsip_verified_cert_info_t pki_verified_cert_info; // Verified certificate info
} rsip_instance_ctrl_t;

/**********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/

/** @cond INC_HEADER_DEFS_SEC */
/** Filled in Interface API structure for this Instance. */
extern const rsip_api_t g_rsip_on_rsip;

/** @endcond */

/**********************************************************************************************************************
 * Public Function Prototypes
 **********************************************************************************************************************/

/* r_rsip.c */
fsp_err_t R_RSIP_Open(rsip_ctrl_t * const p_ctrl, rsip_cfg_t const * const p_cfg);
fsp_err_t R_RSIP_Close(rsip_ctrl_t * const p_ctrl);
fsp_err_t R_RSIP_RandomNumberGenerate(rsip_ctrl_t * const p_ctrl, uint8_t * const p_random);
fsp_err_t R_RSIP_KeyGenerate(rsip_ctrl_t * const        p_ctrl,
                             rsip_key_type_t const      key_type,
                             rsip_wrapped_key_t * const p_wrapped_key);
fsp_err_t R_RSIP_KeyPairGenerate(rsip_ctrl_t * const        p_ctrl,
                                 rsip_key_pair_type_t const key_pair_type,
                                 rsip_wrapped_key_t * const p_wrapped_public_key,
                                 rsip_wrapped_key_t * const p_wrapped_private_key);
fsp_err_t R_RSIP_EncryptedKeyWrap(rsip_ctrl_t * const                 p_ctrl,
                                  rsip_key_update_key_t const * const p_key_update_key,
                                  uint8_t const * const               p_initial_vector,
                                  rsip_key_type_t const               key_type,
                                  uint8_t const * const               p_encrypted_key,
                                  rsip_wrapped_key_t * const          p_wrapped_key);
fsp_err_t R_RSIP_RFC3394_KeyWrap(rsip_ctrl_t * const              p_ctrl,
                                 rsip_wrapped_key_t const * const p_wrapped_kek,
                                 rsip_wrapped_key_t const * const p_wrapped_target_key,
                                 uint8_t * const                  p_rfc3394_wrapped_target_key);
fsp_err_t R_RSIP_RFC3394_KeyUnwrap(rsip_ctrl_t * const              p_ctrl,
                                   rsip_wrapped_key_t const * const p_wrapped_kek,
                                   rsip_key_type_t const            key_type,
                                   uint8_t const * const            p_rfc3394_wrapped_target_key,
                                   rsip_wrapped_key_t * const       p_wrapped_target_key);
fsp_err_t R_RSIP_InjectedKeyImport(rsip_key_type_t const      key_type,
                                   uint8_t const * const      p_injected_key,
                                   rsip_wrapped_key_t * const p_wrapped_key,
                                   uint32_t const             wrapped_key_buffer_length);
fsp_err_t R_RSIP_PublicKeyExport(rsip_wrapped_key_t const * const p_wrapped_public_key,
                                 uint8_t * const                  p_raw_public_key);

/* r_rsip_aes.c */
fsp_err_t R_RSIP_AES_Cipher_Init(rsip_ctrl_t * const              p_ctrl,
                                 rsip_aes_cipher_mode_t const     mode,
                                 rsip_wrapped_key_t const * const p_wrapped_key,
                                 uint8_t const * const            p_initial_vector);
fsp_err_t R_RSIP_AES_Cipher_Update(rsip_ctrl_t * const   p_ctrl,
                                   uint8_t const * const p_input,
                                   uint8_t * const       p_output,
                                   uint32_t const        length);
fsp_err_t R_RSIP_AES_Cipher_Finish(rsip_ctrl_t * const p_ctrl);
fsp_err_t R_RSIP_AES_AEAD_Init(rsip_ctrl_t * const              p_ctrl,
                               rsip_aes_aead_mode_t const       mode,
                               rsip_wrapped_key_t const * const p_wrapped_key,
                               uint8_t const * const            p_nonce,
                               uint32_t const                   nonce_length);
fsp_err_t R_RSIP_AES_AEAD_LengthsSet(rsip_ctrl_t * const p_ctrl,
                                     uint32_t const      total_aad_length,
                                     uint32_t const      total_text_length,
                                     uint32_t const      tag_length);
fsp_err_t R_RSIP_AES_AEAD_AADUpdate(rsip_ctrl_t * const p_ctrl, uint8_t const * const p_aad, uint32_t const aad_length);
fsp_err_t R_RSIP_AES_AEAD_Update(rsip_ctrl_t * const   p_ctrl,
                                 uint8_t const * const p_input,
                                 uint32_t const        input_length,
                                 uint8_t * const       p_output,
                                 uint32_t * const      p_output_length);
fsp_err_t R_RSIP_AES_AEAD_Finish(rsip_ctrl_t * const p_ctrl,
                                 uint8_t * const     p_output,
                                 uint32_t * const    p_output_length,
                                 uint8_t * const     p_tag);
fsp_err_t R_RSIP_AES_AEAD_Verify(rsip_ctrl_t * const   p_ctrl,
                                 uint8_t * const       p_output,
                                 uint32_t * const      p_output_length,
                                 uint8_t const * const p_tag,
                                 uint32_t const        tag_length);

fsp_err_t R_RSIP_AES_MAC_Init(rsip_ctrl_t * const              p_ctrl,
                              rsip_aes_mac_mode_t const        mode,
                              rsip_wrapped_key_t const * const p_wrapped_key);
fsp_err_t R_RSIP_AES_MAC_Update(rsip_ctrl_t * const   p_ctrl,
                                uint8_t const * const p_message,
                                uint32_t const        message_length);
fsp_err_t R_RSIP_AES_MAC_SignFinish(rsip_ctrl_t * const p_ctrl, uint8_t * const p_mac);
fsp_err_t R_RSIP_AES_MAC_VerifyFinish(rsip_ctrl_t * const p_ctrl, uint8_t const * const p_mac,
                                      uint32_t const mac_length);

/* r_rsip_ecc.c */
fsp_err_t R_RSIP_ECDSA_Sign(rsip_ctrl_t * const              p_ctrl,
                            rsip_wrapped_key_t const * const p_wrapped_private_key,
                            uint8_t const * const            p_hash,
                            uint8_t * const                  p_signature);
fsp_err_t R_RSIP_ECDSA_Verify(rsip_ctrl_t * const              p_ctrl,
                              rsip_wrapped_key_t const * const p_wrapped_public_key,
                              uint8_t const * const            p_hash,
                              uint8_t const * const            p_signature);
fsp_err_t R_RSIP_ECDH_KeyAgree(rsip_ctrl_t * const              p_ctrl,
                               rsip_wrapped_key_t const * const p_wrapped_private_key,
                               rsip_wrapped_key_t const * const p_wrapped_public_key,
                               rsip_wrapped_secret_t * const    p_wrapped_secret);
fsp_err_t R_RSIP_ECDH_PlainKeyAgree(rsip_ctrl_t * const              p_ctrl,
                                    rsip_wrapped_key_t const * const p_wrapped_private_key,
                                    uint8_t const * const            p_plain_public_key,
                                    rsip_wrapped_secret_t * const    p_wrapped_secret);

/* r_rsip_rsa.c */
fsp_err_t R_RSIP_RSA_Encrypt(rsip_ctrl_t * const              p_ctrl,
                             rsip_wrapped_key_t const * const p_wrapped_public_key,
                             uint8_t const * const            p_plain,
                             uint8_t * const                  p_cipher);
fsp_err_t R_RSIP_RSA_Decrypt(rsip_ctrl_t * const              p_ctrl,
                             rsip_wrapped_key_t const * const p_wrapped_private_key,
                             uint8_t const * const            p_cipher,
                             uint8_t * const                  p_plain);
fsp_err_t R_RSIP_RSAES_PKCS1_V1_5_Encrypt(rsip_ctrl_t * const              p_ctrl,
                                          rsip_wrapped_key_t const * const p_wrapped_public_key,
                                          uint8_t const * const            p_plain,
                                          uint32_t const                   plain_length,
                                          uint8_t * const                  p_cipher);
fsp_err_t R_RSIP_RSAES_PKCS1_V1_5_Decrypt(rsip_ctrl_t * const              p_ctrl,
                                          rsip_wrapped_key_t const * const p_wrapped_private_key,
                                          uint8_t const * const            p_cipher,
                                          uint8_t * const                  p_plain,
                                          uint32_t * const                 p_plain_length,
                                          uint32_t const                   plain_buffer_length);
fsp_err_t R_RSIP_RSAES_OAEP_Encrypt(rsip_ctrl_t * const              p_ctrl,
                                    rsip_wrapped_key_t const * const p_wrapped_public_key,
                                    rsip_hash_type_t const           hash_function,
                                    rsip_mgf_type_t const            mask_generation_function,
                                    uint8_t const * const            p_label,
                                    uint32_t const                   label_length,
                                    uint8_t const * const            p_plain,
                                    uint32_t const                   plain_length,
                                    uint8_t * const                  p_cipher);
fsp_err_t R_RSIP_RSAES_OAEP_Decrypt(rsip_ctrl_t * const              p_ctrl,
                                    rsip_wrapped_key_t const * const p_wrapped_private_key,
                                    rsip_hash_type_t const           hash_function,
                                    rsip_mgf_type_t const            mask_generation_function,
                                    uint8_t const * const            p_label,
                                    uint32_t const                   label_length,
                                    uint8_t const * const            p_cipher,
                                    uint8_t * const                  p_plain,
                                    uint32_t * const                 p_plain_length,
                                    uint32_t const                   plain_buffer_length);
fsp_err_t R_RSIP_RSASSA_PKCS1_V1_5_Sign(rsip_ctrl_t * const              p_ctrl,
                                        rsip_wrapped_key_t const * const p_wrapped_private_key,
                                        rsip_hash_type_t const           hash_function,
                                        uint8_t const * const            p_hash,
                                        uint8_t * const                  p_signature);
fsp_err_t R_RSIP_RSASSA_PKCS1_V1_5_Verify(rsip_ctrl_t * const              p_ctrl,
                                          rsip_wrapped_key_t const * const p_wrapped_public_key,
                                          rsip_hash_type_t const           hash_function,
                                          uint8_t const * const            p_hash,
                                          uint8_t const * const            p_signature);
fsp_err_t R_RSIP_RSASSA_PSS_Sign(rsip_ctrl_t * const              p_ctrl,
                                 rsip_wrapped_key_t const * const p_wrapped_private_key,
                                 rsip_hash_type_t const           hash_function,
                                 rsip_mgf_type_t const            mask_generation_function,
                                 uint32_t const                   salt_length,
                                 uint8_t const * const            p_hash,
                                 uint8_t * const                  p_signature);
fsp_err_t R_RSIP_RSASSA_PSS_Verify(rsip_ctrl_t * const              p_ctrl,
                                   rsip_wrapped_key_t const * const p_wrapped_public_key,
                                   rsip_hash_type_t const           hash_function,
                                   rsip_mgf_type_t const            mask_generation_function,
                                   uint32_t const                   salt_length,
                                   uint8_t const * const            p_hash,
                                   uint8_t const * const            p_signature);

/* r_rsip_sha.c */
fsp_err_t R_RSIP_SHA_Compute(rsip_ctrl_t * const    p_ctrl,
                             rsip_hash_type_t const hash_type,
                             uint8_t const * const  p_message,
                             uint32_t const         message_length,
                             uint8_t * const        p_digest);
fsp_err_t R_RSIP_SHA_Init(rsip_ctrl_t * const p_ctrl, rsip_hash_type_t const hash_type);
fsp_err_t R_RSIP_SHA_Update(rsip_ctrl_t * const p_ctrl, uint8_t const * const p_message, uint32_t const message_length);
fsp_err_t R_RSIP_SHA_Finish(rsip_ctrl_t * const p_ctrl, uint8_t * const p_digest);
fsp_err_t R_RSIP_SHA_Suspend(rsip_ctrl_t * const p_ctrl, rsip_sha_handle_t * const p_handle);
fsp_err_t R_RSIP_SHA_Resume(rsip_ctrl_t * const p_ctrl, rsip_sha_handle_t const * const p_handle);
fsp_err_t R_RSIP_HMAC_Compute(rsip_ctrl_t * const        p_ctrl,
                              const rsip_wrapped_key_t * p_wrapped_key,
                              uint8_t const * const      p_message,
                              uint32_t const             message_length,
                              uint8_t * const            p_mac);
fsp_err_t R_RSIP_HMAC_Verify(rsip_ctrl_t * const        p_ctrl,
                             const rsip_wrapped_key_t * p_wrapped_key,
                             uint8_t const * const      p_message,
                             uint32_t const             message_length,
                             uint8_t const * const      p_mac,
                             uint32_t const             mac_length);
fsp_err_t R_RSIP_HMAC_Init(rsip_ctrl_t * const p_ctrl, rsip_wrapped_key_t const * const p_wrapped_key);
fsp_err_t R_RSIP_HMAC_Update(rsip_ctrl_t * const p_ctrl, uint8_t const * const p_message,
                             uint32_t const message_length);
fsp_err_t R_RSIP_HMAC_SignFinish(rsip_ctrl_t * const p_ctrl, uint8_t * const p_mac);
fsp_err_t R_RSIP_HMAC_VerifyFinish(rsip_ctrl_t * const p_ctrl, uint8_t const * const p_mac, uint32_t const mac_length);
fsp_err_t R_RSIP_HMAC_Suspend(rsip_ctrl_t * const p_ctrl, rsip_hmac_handle_t * const p_handle);
fsp_err_t R_RSIP_HMAC_Resume(rsip_ctrl_t * const p_ctrl, rsip_hmac_handle_t const * const p_handle);

/* r_rsip_pki.c */
fsp_err_t R_RSIP_PKI_ECDSA_CertVerify(rsip_ctrl_t * const              p_ctrl,
                                      rsip_wrapped_key_t const * const p_wrapped_public_key,
                                      uint8_t const * const            p_hash,
                                      uint8_t const * const            p_signature);
fsp_err_t R_RSIP_PKI_VerifiedCertInfoExport(rsip_ctrl_t * const               p_ctrl,
                                            rsip_verified_cert_info_t * const p_verified_cert_info);
fsp_err_t R_RSIP_PKI_VerifiedCertInfoImport(rsip_ctrl_t * const                     p_ctrl,
                                            rsip_verified_cert_info_t const * const p_verified_cert_info);
fsp_err_t R_RSIP_PKI_CertKeyImport(rsip_ctrl_t * const        p_ctrl,
                                   uint8_t const * const      p_cert,
                                   uint32_t const             cert_length,
                                   rsip_key_type_t const      key_type,
                                   uint8_t const * const      p_key_param1,
                                   uint32_t const             key_param1_length,
                                   uint8_t const * const      p_key_param2,
                                   uint32_t const             key_param2_length,
                                   rsip_hash_type_t const     hash_function,
                                   rsip_wrapped_key_t * const p_wrapped_public_key);

/* r_rsip_kdf.c */
fsp_err_t R_RSIP_KDF_MACKeyImport(rsip_ctrl_t * const              p_ctrl,
                                  rsip_key_type_t const            key_type,
                                  rsip_wrapped_mac_t const * const p_wrapped_mac,
                                  uint32_t const                   key_length,
                                  rsip_wrapped_key_t * const       p_wrapped_key);
fsp_err_t R_RSIP_KDF_ECDHSecretKeyImport(rsip_ctrl_t * const                 p_ctrl,
                                         rsip_key_type_t const               key_type,
                                         rsip_wrapped_secret_t const * const p_wrapped_secret,
                                         rsip_wrapped_key_t * const          p_wrapped_key);
fsp_err_t R_RSIP_KDF_HMAC_Init(rsip_ctrl_t * const p_ctrl, rsip_wrapped_key_t const * const p_wrapped_key);
fsp_err_t R_RSIP_KDF_HMAC_MACUpdate(rsip_ctrl_t * const p_ctrl, rsip_wrapped_mac_t const * const p_wrapped_mac);
fsp_err_t R_RSIP_KDF_HMAC_ECDHSecretUpdate(rsip_ctrl_t * const                 p_ctrl,
                                           rsip_wrapped_secret_t const * const p_wrapped_secret);
fsp_err_t R_RSIP_KDF_HMAC_Update(rsip_ctrl_t * const   p_ctrl,
                                 uint8_t const * const p_message,
                                 uint32_t const        message_length);
fsp_err_t R_RSIP_KDF_HMAC_SignFinish(rsip_ctrl_t * const p_ctrl, rsip_wrapped_mac_t * const p_wrapped_mac);
fsp_err_t R_RSIP_KDF_HMAC_Suspend(rsip_ctrl_t * const p_ctrl, rsip_kdf_hmac_handle_t * const p_handle);
fsp_err_t R_RSIP_KDF_HMAC_Resume(rsip_ctrl_t * const p_ctrl, rsip_kdf_hmac_handle_t const * const p_handle);
fsp_err_t R_RSIP_KDF_MACConcatenate(rsip_wrapped_mac_t * const       p_wrapped_mac1,
                                    rsip_wrapped_mac_t const * const p_wrapped_mac2,
                                    uint32_t const                   wrapped_mac1_buffer_length);
fsp_err_t R_RSIP_KDF_DerivedKeyImport(rsip_ctrl_t * const              p_ctrl,
                                      rsip_wrapped_mac_t const * const p_wrapped_mac,
                                      rsip_key_type_t const            key_type,
                                      uint32_t const                   position,
                                      rsip_wrapped_key_t * const       p_wrapped_key);
fsp_err_t R_RSIP_KDF_DerivedIVWrap(rsip_ctrl_t * const              p_ctrl,
                                   rsip_wrapped_mac_t const * const p_wrapped_mac,
                                   rsip_initial_vector_type_t const initial_vector_type,
                                   uint32_t const                   position,
                                   uint8_t const * const            p_tls_sequence_num,
                                   uint8_t * const                  p_wrapped_initial_vector);

/* r_rsip_otf.c */
fsp_err_t R_RSIP_OTF_Init(rsip_ctrl_t * const        p_ctrl,
                          rsip_otf_channel_t const   channel,
                          rsip_wrapped_key_t * const p_wrapped_key,
                          uint8_t const * const      p_seed);

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif                                 /* R_RSIP_H */

/*******************************************************************************************************************//**
 * @} (end addtogroup RSIP_PROTECTED)
 **********************************************************************************************************************/
