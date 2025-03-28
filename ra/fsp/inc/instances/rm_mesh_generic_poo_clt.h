/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef RM_MESH_GENERIC_POO_CLT_H
#define RM_MESH_GENERIC_POO_CLT_H

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
 * Generic OnPowerUp Set message parameters.
 */
typedef struct st_rm_mesh_generic_poo_on_power_up_info
{
    /**
     * The Generic OnPowerUp state is an enumeration representing the behavior of an element when powered up.
     *
     * Value     | Description
     * ----------|------------
     * 0x00      | Off. After being powered up, the element is in an off state.
     * 0x01      | Default. After being powered up, the element is in an On state and uses default state values.
     * 0x02      | Restore. If a transition was in progress when powered down, the element restores the target
     *             state when powered up. Otherwise the element restores the state it was in when powered down.
     * 0x03-0xFF | Prohibited
     */
    uint8_t on_power_up;
} rm_mesh_generic_poo_on_power_up_info_t;

/*******************************************************************************************************************//**
 * @addtogroup RM_MESH_GENERIC_POO_CLT
 * @{
 **********************************************************************************************************************/

/** BLE mesh generic poo instance control block. DO NOT INITIALIZE. Initialization occurs when RM_MESH_GENERIC_POO_CLT_Open() is called. */
typedef struct st_rm_mesh_generic_poo_clt_instance_ctrl
{
    uint32_t open;                                // To check whether api has been opened or not.
    rm_ble_mesh_model_client_cfg_t const * p_cfg; // Pointer to initial configurations.
    rm_ble_mesh_access_model_handle_t      model_handle;
} rm_mesh_generic_poo_clt_instance_ctrl_t;

/**********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/

/** @cond INC_HEADER_DEFS_SEC */
/** Filled in Interface API structure for this Instance. */
extern const rm_ble_mesh_model_client_api_t g_rm_ble_mesh_model_client_on_rm_mesh_generic_poo_clt;

/** @endcond */

/***********************************************************************************************************************
 * Public APIs
 **********************************************************************************************************************/
fsp_err_t RM_MESH_GENERIC_POO_CLT_Open(rm_ble_mesh_model_client_ctrl_t * const      p_ctrl,
                                       rm_ble_mesh_model_client_cfg_t const * const p_cfg);
fsp_err_t RM_MESH_GENERIC_POO_CLT_Close(rm_ble_mesh_model_client_ctrl_t * const p_ctrl);
fsp_err_t RM_MESH_GENERIC_POO_CLT_GetModelHandle(rm_ble_mesh_model_client_ctrl_t * const   p_ctrl,
                                                 rm_ble_mesh_access_model_handle_t * const model_handle);
fsp_err_t RM_MESH_GENERIC_POO_CLT_SendReliablePdu(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                                  uint32_t                                req_opcode,
                                                  void const * const                      parameter,
                                                  uint32_t                                rsp_opcode);
fsp_err_t RM_MESH_GENERIC_POO_CLT_OnpowerupGet(rm_ble_mesh_model_client_ctrl_t * const p_ctrl);
fsp_err_t RM_MESH_GENERIC_POO_CLT_OnpowerupSet(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                               void const * const                      p_parameter);
fsp_err_t RM_MESH_GENERIC_POO_CLT_OnpowerupSetUnacknowledged(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                                             void const * const                      p_parameter);

/*******************************************************************************************************************//**
 * @} (end addgroup RM_MESH_GENERIC_POO_CLT)
 **********************************************************************************************************************/

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif                                 // RM_MESH_GENERIC_POO_CLT_H
