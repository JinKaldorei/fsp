/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef RM_MESH_GENERIC_USER_PROP_SRV_H
#define RM_MESH_GENERIC_USER_PROP_SRV_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

#include "rm_ble_mesh_model_server_api.h"
#include "rm_ble_mesh_access.h"

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

/**********************************************************************************************************************
 * Macro definitions
 *********************************************************************************************************************/

/*********************************************************************************************************************
 * Typedef definitions
 *********************************************************************************************************************/

/*******************************************************************************************************************//**
 *
 * @addtogroup RM_MESH_GENERIC_USER_PROP_SRV
 * @{
 **********************************************************************************************************************/

/** Generic User Property is a state representing a device property of an element */
typedef struct st_rm_mesh_generic_user_prop_srv_info
{
    /**
     * User Property ID field is a 2-octet Assigned Number value referencing a
     * device property
     */
    uint16_t property_id;

    /**
     * User Access field is an enumeration indicating whether the device property
     * can be read or written as a Generic User Property
     */
    uint8_t user_access;

    /** User Property Value field is a conditional field */
    uint8_t * property_value;
    uint16_t  property_value_len;
} rm_mesh_generic_user_prop_srv_info_t;

/** BLE mesh generic user prop instance control block. DO NOT INITIALIZE. Initialization occurs when RM_MESH_GENERIC_USER_PROP_SRV_Open() is called. */
typedef struct st_rm_mesh_generic_user_prop_srv_instance_ctrl
{
    uint32_t open;                                // To check whether api has been opened or not.
    rm_ble_mesh_access_model_handle_t model_handle;
    rm_ble_mesh_access_model_handle_t setup_server_handle;

    rm_ble_mesh_model_server_cfg_t const * p_cfg; // Pointer to initial configurations.
} rm_mesh_generic_user_prop_srv_instance_ctrl_t;

/**********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/

/** @cond INC_HEADER_DEFS_SEC */
/** Filled in Interface API structure for this Instance. */
extern const rm_ble_mesh_model_server_api_t g_rm_ble_mesh_model_server_on_rm_mesh_generic_user_prop_srv;

/** @endcond */

/***********************************************************************************************************************
 * Public APIs
 **********************************************************************************************************************/
fsp_err_t RM_MESH_GENERIC_USER_PROP_SRV_Open(rm_ble_mesh_model_server_ctrl_t * const      p_ctrl,
                                             rm_ble_mesh_model_server_cfg_t const * const p_cfg);
fsp_err_t RM_MESH_GENERIC_USER_PROP_SRV_Close(rm_ble_mesh_model_server_ctrl_t * const p_ctrl);
fsp_err_t RM_MESH_GENERIC_USER_PROP_SRV_StateUpdate // MS_generic_user_prop_server_state_update
    (rm_ble_mesh_model_server_ctrl_t * const p_ctrl, rm_ble_mesh_access_server_state_t const * const p_state);

/*******************************************************************************************************************//**
 * @} (end addgroup RM_MESH_GENERIC_USER_PROP_SRV)
 **********************************************************************************************************************/

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif                                 // RM_MESH_GENERIC_USER_PROP_SRV_H
