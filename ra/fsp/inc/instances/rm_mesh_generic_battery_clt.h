/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef RM_MESH_GENERIC_BATTERY_CLT_H
#define RM_MESH_GENERIC_BATTERY_CLT_H

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
 *  The Generic Battery state is a set of four values representing the state of a battery:
 *  - a charge level (Generic Battery Level)
 *  - remaining time to complete discharging (Generic Battery Time to Discharge)
 *  - remaining time to complete charging (Generic Battery Time to Charge)
 *  - flags bit field (Generic Battery Flags)
 */
typedef struct st_rm_mesh_generic_battery_status_info
{
    /**
     * Generic Battery Level.
     * The Generic Battery Level state is a value ranging from 0 percent through 100 percent.
     *
     * Value     | Description
     * ----------|------------
     * 0x00-0x64 | The percentage of the charge level. 100% represents fully charged. 0% represents fully discharged.
     * 0x65-0xFE | Prohibited
     * 0xFF      | The percentage of the charge level is unknown.
     */
    uint8_t battery_level;

    /**
     * The Generic Battery Time to Discharge state is a 24-bit unsigned value ranging from 0 through 0xFFFFFF.
     *
     * Value             | Description
     * ------------------|------------
     * 0x000000-0xFFFFFE | The remaining time (in minutes) of the discharging process
     * 0xFFFFFF          | The remaining time of the discharging process is not known.
     */
    uint32_t time_to_discharge;

    /**
     * The Generic Battery Time to Charge state is a 24-bit unsigned value ranging from 0 through 0xFFFFFF.
     *
     * Value             | Description
     * ------------------|------------
     * 0x000000-0xFFFFFE | The remaining time (in minutes) of the charging process
     * 0xFFFFFF          | The remaining time of the charging process is not known.
     */
    uint32_t time_to_charge;

    /**
     * The Generic Battery Flags state is a concatenation of four 2-bit bit fields: Presence, Indicator, Charging, and Serviceability
     *
     * Bit  | Description
     * -----|------------
     * 0-1  | Generic Battery Flags Presence
     * 2-3  | Generic Battery Flags Indicator
     * 4-5  | Generic Battery Flags Charging
     * 6-7  | Generic Battery Flags Serviceability
     */
    uint8_t flags;
} rm_mesh_generic_battery_status_info_t;

/*******************************************************************************************************************//**
 * @addtogroup RM_MESH_GENERIC_BATTERY_CLT
 * @{
 **********************************************************************************************************************/

/** BLE mesh generic battery instance control block. DO NOT INITIALIZE. Initialization occurs when RM_MESH_GENERIC_BATTERY_CLT_Open() is called. */
typedef struct st_rm_mesh_generic_battery_clt_instance_ctrl
{
    uint32_t open;                                // To check whether api has been opened or not.
    rm_ble_mesh_model_client_cfg_t const * p_cfg; // Pointer to initial configurations.
    rm_ble_mesh_access_model_handle_t      model_handle;
} rm_mesh_generic_battery_clt_instance_ctrl_t;

/**********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/

/** @cond INC_HEADER_DEFS_SEC */
/** Filled in Interface API structure for this Instance. */
extern const rm_ble_mesh_model_client_api_t g_rm_ble_mesh_model_client_on_rm_mesh_generic_battery_clt;

/** @endcond */

/***********************************************************************************************************************
 * Public APIs
 **********************************************************************************************************************/
fsp_err_t RM_MESH_GENERIC_BATTERY_CLT_Open(rm_ble_mesh_model_client_ctrl_t * const      p_ctrl,
                                           rm_ble_mesh_model_client_cfg_t const * const p_cfg);
fsp_err_t RM_MESH_GENERIC_BATTERY_CLT_Close(rm_ble_mesh_model_client_ctrl_t * const p_ctrl);
fsp_err_t RM_MESH_GENERIC_BATTERY_CLT_GetModelHandle(rm_ble_mesh_model_client_ctrl_t * const   p_ctrl,
                                                     rm_ble_mesh_access_model_handle_t * const model_handle);

fsp_err_t RM_MESH_GENERIC_BATTERY_CLT_SendReliablePdu(rm_ble_mesh_model_client_ctrl_t * const p_ctrl,
                                                      uint32_t                                req_opcode,
                                                      void const * const                      parameter,
                                                      uint32_t                                rsp_opcode);
fsp_err_t RM_MESH_GENERIC_BATTERY_CLT_Get(rm_ble_mesh_model_client_ctrl_t * const p_ctrl);

/*******************************************************************************************************************//**
 * @} (end addgroup RM_MESH_GENERIC_BATTERY_CLT)
 **********************************************************************************************************************/

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif                                 // RM_MESH_GENERIC_BATTERY_CLT_H
