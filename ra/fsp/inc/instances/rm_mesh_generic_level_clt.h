/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef RM_MESH_GENERIC_LEVEL_CLT_H
#define RM_MESH_GENERIC_LEVEL_CLT_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

#include "rm_ble_mesh_model_client_api.h"
#include "rm_ble_mesh_access.h"

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

/**********************************************************************************************************************
 * Macro definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Typedef definitions
 *********************************************************************************************************************/

/**
 * Generic Level Set message parameters
 */
typedef struct st_rm_mesh_generic_level_set_info
{
    /**
     * The target value of the Generic Level state.
     *
     * The Generic Level state is a 16-bit signed integer (2's complement) representing
     * the state of an element.
     */
    uint16_t level;

    /** Transaction Identifier */
    uint8_t transaction_id;

    /**
     * Transition Time is a 1-octet value that consists of two fields:
     * - a 2-bit bit field representing the step resolution
     * - a 6-bit bit field representing the number of transition steps.
     *
     * Field                      | Size (bits) | Description
     * ---------------------------|-------------|----------------
     * Transition Number of Steps | 6           | The number of Steps
     * Transition Step Resolution | 2           | The resolution of the Default Transition
     *                                          | Number of Steps field
     */
    uint8_t transition_time;

    /** Message execution delay in 5 milliseconds steps */
    uint8_t delay;

    /** Flag: To represent if optional Transaction time and Delay fields are valid */
    uint8_t optional_fields_present;
} rm_mesh_generic_level_set_info_t;

/**
 * Generic Level Status message parameters
 */
typedef struct st_rm_mesh_generic_level_status_info
{
    /** The present value of the Generic Level state. */
    uint16_t present_level;

    /** The target value of the Generic Level state */
    uint16_t target_level;

    /**
     * Remaining Time is a 1-octet value that consists of two fields:
     * - a 2-bit bit field representing the step resolution
     * - a 6-bit bit field representing the number of transition steps.
     *
     * Field                      | Size (bits) | Description
     * ---------------------------|-------------|----------------
     * Transition Number of Steps | 6           | The number of Steps
     * Transition Step Resolution | 2           | The resolution of the Default Transition
     *                                          | Number of Steps field
     */
    uint8_t remaining_time;

    /** Flag: To represent if optional fields Target Level and Remaining Time are valid */
    uint8_t optional_fields_present;
} rm_mesh_generic_level_status_info_t;

/**
 * Generic Delta Set message parameters.
 */
typedef struct st_rm_mesh_generic_level_delta_set_info
{
    /** The Delta change of the Generic Level state */
    uint32_t delta_level;

    /** Transaction Identifier */
    uint8_t transaction_id;

    /**
     * Transition Time is a 1-octet value that consists of two fields:
     * - a 2-bit bit field representing the step resolution
     * - a 6-bit bit field representing the number of transition steps.
     *
     * Field                      | Size (bits) | Description
     * ---------------------------|-------------|----------------
     * Transition Number of Steps | 6           | The number of Steps
     * Transition Step Resolution | 2           | The resolution of the Default Transition
     *                                          | Number of Steps field
     */
    uint8_t transition_time;

    /** Message execution delay in 5 milliseconds steps */
    uint8_t delay;

    /** Flag: To represent if optional Transaction time and Delay fields are valid */
    uint8_t optional_fields_present;
} rm_mesh_generic_level_delta_set_info_t;

/**
 * Generic Move Set message parameters.
 */
typedef struct st_rm_mesh_generic_level_move_set_info
{
    /** The Delta Level step to calculate Move speed for the Generic Level state. */
    uint16_t delta_level;

    /** Transaction Identifier */
    uint8_t transaction_id;

    /**
     * Transition Time is a 1-octet value that consists of two fields:
     * - a 2-bit bit field representing the step resolution
     * - a 6-bit bit field representing the number of transition steps.
     *
     * Field                      | Size (bits) | Description
     * ---------------------------|-------------|----------------
     * Transition Number of Steps | 6           | The number of Steps
     * Transition Step Resolution | 2           | The resolution of the Default Transition
     *                                          | Number of Steps field
     */
    uint8_t transition_time;

    /** Message execution delay in 5 milliseconds steps */
    uint8_t delay;

    /** Flag: To represent if optional Transaction time and Delay fields are valid */
    uint8_t optional_fields_present;
} rm_mesh_generic_level_move_set_info_t;

/*******************************************************************************************************************//**
 * @addtogroup RM_MESH_GENERIC_LEVEL_CLT
 * @{
 **********************************************************************************************************************/

/** BLE mesh generic level instance control block. DO NOT INITIALIZE. Initialization occurs when RM_MESH_GENERIC_LEVEL_CLT_Open() is called. */
typedef struct st_rm_mesh_generic_level_clt_instance_ctrl
{
    uint32_t open;                                // To check whether api has been opened or not.
    rm_ble_mesh_model_client_cfg_t const * p_cfg; // Pointer to initial configurations.
    rm_ble_mesh_access_model_handle_t      model_handle;
} rm_mesh_generic_level_clt_instance_ctrl_t;

/**********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/

/** @cond INC_HEADER_DEFS_SEC */
/** Filled in Interface API structure for this Instance. */
extern const rm_ble_mesh_model_client_api_t g_rm_ble_mesh_model_client_on_rm_mesh_generic_level_clt;

/** @endcond */

/***********************************************************************************************************************
 * Public APIs
 **********************************************************************************************************************/
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_Open(rm_ble_mesh_model_client_ctrl_t * const      p_ctrl,
                                         rm_ble_mesh_model_client_cfg_t const * const p_cfg);
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_Close(rm_ble_mesh_model_client_ctrl_t * const p_ctrl);
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_GetModelHandle(rm_ble_mesh_model_client_ctrl_t * const   p_ctrl,
                                                   rm_ble_mesh_access_model_handle_t * const model_handle);
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_SendReliablePdu(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                                    uint32_t                                req_opcode,
                                                    void const * const                      parameter,
                                                    uint32_t                                rsp_opcode);

fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_Get(rm_ble_mesh_model_client_ctrl_t * const p_ctrl);
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_Set(rm_ble_mesh_model_client_ctrl_t * const p_ctrl, void const * const p_parameter);
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_SetUnacknowledged(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                                      void const * const                      p_parameter);
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_DeltaSet(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                             void const * const                      p_parameter);
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_DeltaSetUnacknowledged(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                                           void const * const                      p_parameter);
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_MoveSet(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                            void const * const                      p_parameter);
fsp_err_t RM_MESH_GENERIC_LEVEL_CLT_MoveSetUnacknowledged(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                                          void const * const                      p_parameter);

/*******************************************************************************************************************//**
 * @} (end addgroup RM_MESH_GENERIC_LEVEL_CLT)
 **********************************************************************************************************************/

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif                                 // RM_MESH_GENERIC_LEVEL_CLT_H
