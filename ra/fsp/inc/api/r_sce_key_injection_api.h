/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*******************************************************************************************************************//**
 * @ingroup RENESAS_SECURITY_INTERFACES
 * @defgroup SCE_KEY_INJECTION_API SCE key injection Interface
 * @brief Interface for key injection by Secure Crypto Engine (SCE) functions.
 *
 * @section SCE_API_Summary Summary
 * The SCE key injection interface provides SCE functionality.
 *
 *
 * @{
 **********************************************************************************************************************/

#ifndef R_SCE_KEY_INJECTION_API_H
#define R_SCE_KEY_INJECTION_API_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/* For AES operation. */
#define R_SCE_AES256_KEY_INDEX_WORD_SIZE                         (16U)

/* Key update. */
#define R_SCE_UPDATE_KEY_RING_INDEX_WORD_SIZE                    (16U)
#define R_SCE_INSTALL_KEY_RING_INDEX                             (0U)

/* For RSA operation. */
#define R_SCE_RSA_2048_KEY_N_LENGTH_BYTE_SIZE                    (256U)
#define R_SCE_RSA_3072_KEY_N_LENGTH_BYTE_SIZE                    (348U)
#define R_SCE_RSA_4096_KEY_N_LENGTH_BYTE_SIZE                    (512U)
#define R_SCE_RSA_2048_KEY_E_LENGTH_BYTE_SIZE                    (4U)
#define R_SCE_RSA_3072_KEY_E_LENGTH_BYTE_SIZE                    (4U)
#define R_SCE_RSA_4096_KEY_E_LENGTH_BYTE_SIZE                    (4U)
#define R_SCE_RSA_2048_KEY_D_LENGTH_BYTE_SIZE                    (256U)
#define R_SCE_RSA_KEY_GENERATION_DUMMY_BYTE_SIZE                 (12U)
#define R_SCE_RSA_2048_PUBLIC_KEY_MANAGEMENT_INFO1_WORD_SIZE     (4U)
#define R_SCE_RSA_3072_PUBLIC_KEY_MANAGEMENT_INFO1_WORD_SIZE     (4U)
#define R_SCE_RSA_4096_PUBLIC_KEY_MANAGEMENT_INFO1_WORD_SIZE     (4U)
#define R_SCE_RSA_2048_PUBLIC_KEY_MANAGEMENT_INFO2_WORD_SIZE     (68U)
#define R_SCE_RSA_3072_PUBLIC_KEY_MANAGEMENT_INFO2_WORD_SIZE     (4U)
#define R_SCE_RSA_4096_PUBLIC_KEY_MANAGEMENT_INFO2_WORD_SIZE     (5U)
#define R_SCE_RSA_2048_PRIVATE_KEY_MANAGEMENT_INFO1_WORD_SIZE    (4U)
#define R_SCE_RSA_3072_PRIVATE_KEY_MANAGEMENT_INFO1_WORD_SIZE    (4U)
#define R_SCE_RSA_4096_PRIVATE_KEY_MANAGEMENT_INFO1_WORD_SIZE    (4U)
#define R_SCE_RSA_2048_PRIVATE_KEY_MANAGEMENT_INFO2_WORD_SIZE    (132U)
#define R_SCE_RSA_3072_PRIVATE_KEY_MANAGEMENT_INFO2_WORD_SIZE    (196U)
#define R_SCE_RSA_4096_PRIVATE_KEY_MANAGEMENT_INFO2_WORD_SIZE    (260U)

/* For ECC operation. */
#define R_SCE_ECC_PUBLIC_KEY_MANAGEMENT_INFO_WORD_SIZE           (4U)
#define R_SCE_ECC_KEY_LENGTH_BYTE_SIZE                           (96U)
#define R_SCE_ECC_PRIVATE_KEY_MANAGEMENT_INFO_WORD_SIZE          (20U)

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * @cond
 **********************************************************************************************************************/

/** AES wrapped key data structure. DO NOT MODIFY. */
typedef struct sce_aes_wrapped_key
{
    uint32_t type;                                    ///< key type

    /* AES128, AES256 are supported */
    uint32_t value[R_SCE_AES256_KEY_INDEX_WORD_SIZE]; ///< wrapped key value
} sce_aes_wrapped_key_t;

/** Update key ring index data structure. DO NOT MODIFY. */
typedef struct sce_key_update_key
{
    uint32_t type;                                         ///< key type
    uint32_t value[R_SCE_UPDATE_KEY_RING_INDEX_WORD_SIZE]; ///< wrapped key value
} sce_key_update_key_t;

/** RSA 2048bit public wrapped key data structure. DO NOT MODIFY. */
typedef struct sce_rsa2048_public_wrapped_key
{
    uint32_t type;                                                ///< Key type
    struct
    {
        uint32_t key_management_info1[R_SCE_RSA_2048_PUBLIC_KEY_MANAGEMENT_INFO1_WORD_SIZE];
        uint8_t  key_n[R_SCE_RSA_2048_KEY_N_LENGTH_BYTE_SIZE];    /* plaintext */
        uint8_t  key_e[R_SCE_RSA_2048_KEY_E_LENGTH_BYTE_SIZE];    /* plaintext */
        uint8_t  dummy[R_SCE_RSA_KEY_GENERATION_DUMMY_BYTE_SIZE]; /* dummy data */
        uint32_t key_management_info2[R_SCE_RSA_2048_PUBLIC_KEY_MANAGEMENT_INFO2_WORD_SIZE];
    } value;
} sce_rsa2048_public_wrapped_key_t;

/** RSA 3072bit public wrapped key data structure. DO NOT MODIFY. */
typedef struct sce_rsa3072_public_wrapped_key
{
    uint32_t type;                                                ///< Key type
    struct
    {
        uint32_t key_management_info1[R_SCE_RSA_3072_PUBLIC_KEY_MANAGEMENT_INFO1_WORD_SIZE];
        uint8_t  key_n[R_SCE_RSA_3072_KEY_N_LENGTH_BYTE_SIZE];    /* plaintext */
        uint8_t  key_e[R_SCE_RSA_3072_KEY_E_LENGTH_BYTE_SIZE];    /* plaintext */
        uint8_t  dummy[R_SCE_RSA_KEY_GENERATION_DUMMY_BYTE_SIZE]; /* dummy data */
        uint32_t key_management_info2[R_SCE_RSA_3072_PUBLIC_KEY_MANAGEMENT_INFO2_WORD_SIZE];
    } value;
} sce_rsa3072_public_wrapped_key_t;

/** RSA 4096bit public wrapped key data structure. DO NOT MODIFY. */
typedef struct sce_rsa4096_public_wrapped_key
{
    uint32_t type;                                                ///< Key type
    struct
    {
        uint32_t key_management_info1[R_SCE_RSA_4096_PUBLIC_KEY_MANAGEMENT_INFO1_WORD_SIZE];
        uint8_t  key_n[R_SCE_RSA_4096_KEY_N_LENGTH_BYTE_SIZE];    /* plaintext */
        uint8_t  key_e[R_SCE_RSA_4096_KEY_E_LENGTH_BYTE_SIZE];    /* plaintext */
        uint8_t  dummy[R_SCE_RSA_KEY_GENERATION_DUMMY_BYTE_SIZE]; /* dummy data */
        uint32_t key_management_info2[R_SCE_RSA_4096_PUBLIC_KEY_MANAGEMENT_INFO2_WORD_SIZE];
    } value;
} sce_rsa4096_public_wrapped_key_t;

/** RSA 2048bit private wrapped key data structure. DO NOT MODIFY. */
typedef struct sce_rsa2048_private_wrapped_key
{
    uint32_t type;                                             ///< key type
    struct
    {
        uint32_t key_management_info1[R_SCE_RSA_2048_PRIVATE_KEY_MANAGEMENT_INFO1_WORD_SIZE];
        uint8_t  key_n[R_SCE_RSA_2048_KEY_N_LENGTH_BYTE_SIZE]; /* plaintext */
        uint32_t key_management_info2[R_SCE_RSA_2048_PRIVATE_KEY_MANAGEMENT_INFO2_WORD_SIZE];
    } value;
} sce_rsa2048_private_wrapped_key_t;

/** RSA 3072bit private wrapped key data structure. DO NOT MODIFY. */
typedef struct sce_rsa3072_private_wrapped_key
{
    uint32_t type;                                             ///< key type
    struct
    {
        uint32_t key_management_info1[R_SCE_RSA_3072_PRIVATE_KEY_MANAGEMENT_INFO1_WORD_SIZE];
        uint8_t  key_n[R_SCE_RSA_3072_KEY_N_LENGTH_BYTE_SIZE]; /* plaintext */
        uint32_t key_management_info2[R_SCE_RSA_3072_PRIVATE_KEY_MANAGEMENT_INFO2_WORD_SIZE];
    } value;
} sce_rsa3072_private_wrapped_key_t;

/** RSA 4096bit private wrapped key data structure. DO NOT MODIFY. */
typedef struct sce_rsa4096_private_wrapped_key
{
    uint32_t type;                                             ///< key type
    struct
    {
        uint32_t key_management_info1[R_SCE_RSA_4096_PRIVATE_KEY_MANAGEMENT_INFO1_WORD_SIZE];
        uint8_t  key_n[R_SCE_RSA_4096_KEY_N_LENGTH_BYTE_SIZE]; /* plaintext */
        uint32_t key_management_info2[R_SCE_RSA_4096_PRIVATE_KEY_MANAGEMENT_INFO2_WORD_SIZE];
    } value;
} sce_rsa4096_private_wrapped_key_t;

/** ECC 256/384 public wrapped key data structure */
typedef struct sce_ecc_public_wrapped_key
{
    uint32_t type;
    struct
    {
        uint32_t key_management_info[R_SCE_ECC_PUBLIC_KEY_MANAGEMENT_INFO_WORD_SIZE];
        uint8_t  key_q[R_SCE_ECC_KEY_LENGTH_BYTE_SIZE];
        uint32_t key_management_info2[R_SCE_ECC_PUBLIC_KEY_MANAGEMENT_INFO_WORD_SIZE];
    } value;
} sce_ecc_public_wrapped_key_t;

/** ECC 256/384 private wrapped key data structure */
typedef struct sce_ecc_private_wrapped_key
{
    uint32_t type;
    uint32_t value[R_SCE_ECC_PRIVATE_KEY_MANAGEMENT_INFO_WORD_SIZE];
} sce_ecc_private_wrapped_key_t;

/*******************************************************************************************************************//**
 * @endcond
 **********************************************************************************************************************/

/** Functions implemented at the HAL layer will follow this API. */
typedef struct st_sce_key_injection_api
{
    /** This API outputs 128-bit AES wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           128-bit AES wrapped key
     */
    fsp_err_t (* AES128_InitialKeyWrap)(const uint8_t * const key_type,
                                        const uint8_t * const wrapped_user_factory_programming_key,
                                        const uint8_t * const initial_vector,
                                        const uint8_t * const encrypted_key, sce_aes_wrapped_key_t * const wrapped_key);

    /** This API outputs 192-bit AES wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           192-bit AES wrapped key
     */
    fsp_err_t (* AES192_InitialKeyWrap)(const uint8_t * const key_type,
                                        const uint8_t * const wrapped_user_factory_programming_key,
                                        const uint8_t * const initial_vector,
                                        const uint8_t * const encrypted_key, sce_aes_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit AES wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           256-bit AES wrapped key
     */
    fsp_err_t (* AES256_InitialKeyWrap)(const uint8_t * const key_type,
                                        const uint8_t * const wrapped_user_factory_programming_key,
                                        const uint8_t * const initial_vector,
                                        const uint8_t * const encrypted_key, sce_aes_wrapped_key_t * const wrapped_key);

    /** This API outputs key update key.
     *
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] key_update_key                        Key update key
     */
    fsp_err_t (* KeyUpdateKeyWrap)(const uint8_t * const wrapped_user_factory_programming_key,
                                   const uint8_t * const initial_vector, const uint8_t * const encrypted_key,
                                   sce_key_update_key_t * const key_update_key);

    /** This API updates 128-bit AES wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    128-bit AES wrapped key
     */
    fsp_err_t (* AES128_EncryptedKeyWrap)(const uint8_t * const initial_vector, const uint8_t * const encrypted_key,
                                          const sce_key_update_key_t * const key_update_key,
                                          sce_aes_wrapped_key_t * const wrapped_key);

    /** This API updates 192-bit AES wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    192-bit AES wrapped key
     */
    fsp_err_t (* AES192_EncryptedKeyWrap)(const uint8_t * const initial_vector, const uint8_t * const encrypted_key,
                                          const sce_key_update_key_t * const key_update_key,
                                          sce_aes_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit AES wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    256-bit AES wrapped key
     */
    fsp_err_t (* AES256_EncryptedKeyWrap)(const uint8_t * const initial_vector, const uint8_t * const encrypted_key,
                                          const sce_key_update_key_t * const key_update_key,
                                          sce_aes_wrapped_key_t * const wrapped_key);

    /** This API outputs 2048-bit RSA public wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           2048-bit RSA wrapped key
     */
    fsp_err_t (* RSA2048_InitialPublicKeyWrap)(const uint8_t * const key_type,
                                               const uint8_t * const
                                               wrapped_user_factory_programming_key,
                                               const uint8_t * const                    initial_vector,
                                               const uint8_t * const                    encrypted_key,
                                               sce_rsa2048_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 3072-bit RSA public wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           3072-bit RSA wrapped key
     */
    fsp_err_t (* RSA3072_InitialPublicKeyWrap)(const uint8_t * const key_type,
                                               const uint8_t * const
                                               wrapped_user_factory_programming_key,
                                               const uint8_t * const                    initial_vector,
                                               const uint8_t * const                    encrypted_key,
                                               sce_rsa3072_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 4096-bit RSA public wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           4096-bit RSA wrapped key
     */
    fsp_err_t (* RSA4096_InitialPublicKeyWrap)(const uint8_t * const key_type,
                                               const uint8_t * const
                                               wrapped_user_factory_programming_key,
                                               const uint8_t * const                    initial_vector,
                                               const uint8_t * const                    encrypted_key,
                                               sce_rsa4096_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 2048-bit RSA private wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           2048-bit RSA wrapped key
     */
    fsp_err_t (* RSA2048_InitialPrivateKeyWrap)(const uint8_t * const key_type,
                                                const uint8_t * const
                                                wrapped_user_factory_programming_key,
                                                const uint8_t * const                     initial_vector,
                                                const uint8_t * const                     encrypted_key,
                                                sce_rsa2048_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 2048-bit RSA public wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    2048-bit RSA wrapped key
     */
    fsp_err_t (* RSA2048_EncryptedPublicKeyWrap)(const uint8_t * const                    initial_vector,
                                                 const uint8_t * const                    encrypted_key,
                                                 const sce_key_update_key_t * const       key_update_key,
                                                 sce_rsa2048_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 2048-bit RSA private wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    256-bit RSA wrapped key
     */
    fsp_err_t (* RSA2048_EncryptedPrivateKeyWrap)(const uint8_t * const                     initial_vector,
                                                  const uint8_t * const                     encrypted_key,
                                                  const sce_key_update_key_t * const        key_update_key,
                                                  sce_rsa2048_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC public wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp256r1_InitialPublicKeyWrap)(const uint8_t * const key_type,
                                                     const uint8_t * const
                                                     wrapped_user_factory_programming_key,
                                                     const uint8_t * const                initial_vector,
                                                     const uint8_t * const                encrypted_key,
                                                     sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC public wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp256k1_InitialPublicKeyWrap)(const uint8_t * const key_type,
                                                     const uint8_t * const
                                                     wrapped_user_factory_programming_key,
                                                     const uint8_t * const                initial_vector,
                                                     const uint8_t * const                encrypted_key,
                                                     sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 384-bit ECC public wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           384-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp384r1_InitialPublicKeyWrap)(const uint8_t * const key_type,
                                                     const uint8_t * const
                                                     wrapped_user_factory_programming_key,
                                                     const uint8_t * const                initial_vector,
                                                     const uint8_t * const                encrypted_key,
                                                     sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC private wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp256r1_InitialPrivateKeyWrap)(const uint8_t * const key_type,
                                                      const uint8_t * const
                                                      wrapped_user_factory_programming_key,
                                                      const uint8_t * const                 initial_vector,
                                                      const uint8_t * const                 encrypted_key,
                                                      sce_ecc_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC private wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp256k1_InitialPrivateKeyWrap)(const uint8_t * const key_type,
                                                      const uint8_t * const
                                                      wrapped_user_factory_programming_key,
                                                      const uint8_t * const                 initial_vector,
                                                      const uint8_t * const                 encrypted_key,
                                                      sce_ecc_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 384-bit ECC private wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           384-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp384r1_InitialPrivateKeyWrap)(const uint8_t * const key_type,
                                                      const uint8_t * const
                                                      wrapped_user_factory_programming_key,
                                                      const uint8_t * const                 initial_vector,
                                                      const uint8_t * const                 encrypted_key,
                                                      sce_ecc_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC public wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp256r1_EncryptedPublicKeyWrap)(const uint8_t * const                initial_vector,
                                                       const uint8_t * const                encrypted_key,
                                                       const sce_key_update_key_t * const   key_update_key,
                                                       sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC public wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp256k1_EncryptedPublicKeyWrap)(const uint8_t * const                initial_vector,
                                                       const uint8_t * const                encrypted_key,
                                                       const sce_key_update_key_t * const   key_update_key,
                                                       sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 384-bit ECC public wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    384-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp384r1_EncryptedPublicKeyWrap)(const uint8_t * const                initial_vector,
                                                       const uint8_t * const                encrypted_key,
                                                       const sce_key_update_key_t * const   key_update_key,
                                                       sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC private wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp256r1_EncryptedPrivateKeyWrap)(const uint8_t * const                 initial_vector,
                                                        const uint8_t * const                 encrypted_key,
                                                        const sce_key_update_key_t * const    key_update_key,
                                                        sce_ecc_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC private wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp256k1_EncryptedPrivateKeyWrap)(const uint8_t * const                 initial_vector,
                                                        const uint8_t * const                 encrypted_key,
                                                        const sce_key_update_key_t * const    key_update_key,
                                                        sce_ecc_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 384-bit ECC private wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    384-bit ECC wrapped key
     */
    fsp_err_t (* ECC_secp384r1_EncryptedPrivateKeyWrap)(const uint8_t * const                 initial_vector,
                                                        const uint8_t * const                 encrypted_key,
                                                        const sce_key_update_key_t * const    key_update_key,
                                                        sce_ecc_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit Brainpool ECC public wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_brainpoolP256r1_InitialPublicKeyWrap)(const uint8_t * const key_type,
                                                           const uint8_t * const
                                                           wrapped_user_factory_programming_key,
                                                           const uint8_t * const                initial_vector,
                                                           const uint8_t * const                encrypted_key,
                                                           sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit Brainpool ECC private wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_brainpoolP256r1_InitialPrivateKeyWrap)(const uint8_t * const key_type,
                                                            const uint8_t * const
                                                            wrapped_user_factory_programming_key,
                                                            const uint8_t * const                 initial_vector,
                                                            const uint8_t * const                 encrypted_key,
                                                            sce_ecc_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 384-bit Brainpool ECC public wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           384-bit ECC wrapped key
     */
    fsp_err_t (* ECC_brainpoolP384r1_InitialPublicKeyWrap)(const uint8_t * const key_type,
                                                           const uint8_t * const
                                                           wrapped_user_factory_programming_key,
                                                           const uint8_t * const                initial_vector,
                                                           const uint8_t * const                encrypted_key,
                                                           sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 384-bit Brainpool ECC private wrapped key.
     *
     * @param[in]     key_type                              Key type whether encrypted_key or plain key
     * @param[in]     wrapped_user_factory_programming_key  Provisioning key wrapped by the DLM server
     * @param[in]     initial_vector                        Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key                         User key encrypted and MAC appended
     * @param[in,out] wrapped_key                           384-bit ECC wrapped key
     */
    fsp_err_t (* ECC_brainpoolP384r1_InitialPrivateKeyWrap)(const uint8_t * const key_type,
                                                            const uint8_t * const
                                                            wrapped_user_factory_programming_key,
                                                            const uint8_t * const                 initial_vector,
                                                            const uint8_t * const                 encrypted_key,
                                                            sce_ecc_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC public wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_brainpoolP256r1_EncryptedPublicKeyWrap)(const uint8_t * const                initial_vector,
                                                             const uint8_t * const                encrypted_key,
                                                             const sce_key_update_key_t * const   key_update_key,
                                                             sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 256-bit ECC private wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_brainpoolP256r1_EncryptedPrivateKeyWrap)(const uint8_t * const                 initial_vector,
                                                              const uint8_t * const                 encrypted_key,
                                                              const sce_key_update_key_t * const    key_update_key,
                                                              sce_ecc_private_wrapped_key_t * const wrapped_key);

    /** This API outputs 384-bit ECC public wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    256-bit ECC wrapped key
     */
    fsp_err_t (* ECC_brainpoolP384r1_EncryptedPublicKeyWrap)(const uint8_t * const                initial_vector,
                                                             const uint8_t * const                encrypted_key,
                                                             const sce_key_update_key_t * const   key_update_key,
                                                             sce_ecc_public_wrapped_key_t * const wrapped_key);

    /** This API outputs 384-bit ECC private wrapped key.
     *
     * @param[in]     initial_vector Initialization vector when generating encrypted_key
     * @param[in]     encrypted_key  User key encryptedand MAC appended
     * @param[in]     key_update_key Key update keyring
     * @param[in,out] wrapped_key    384-bit ECC wrapped key
     */
    fsp_err_t (* ECC_brainpoolP384r1_EncryptedPrivateKeyWrap)(const uint8_t * const                 initial_vector,
                                                              const uint8_t * const                 encrypted_key,
                                                              const sce_key_update_key_t * const    key_update_key,
                                                              sce_ecc_private_wrapped_key_t * const wrapped_key);
} sce_key_injection_api_t;

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif                                 /* R_SCE_KEY_INJECTION_API_H */

/*******************************************************************************************************************//**
 * @} (end addtogroup SCE_KEY_INJECTION_API)
 **********************************************************************************************************************/
