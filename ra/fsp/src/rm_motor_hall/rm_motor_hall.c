/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include <math.h>
#include "rm_motor_hall.h"
#include "bsp_api.h"
#include "bsp_cfg.h"

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

#define MOTOR_HALL_OPEN                                  (('M' << 24U) | ('T' << 16U) | ('H' << 8U) | ('L' << 0U))

#define MOTOR_HALL_RAD2RPM                               (30.0F / 3.14159265359F)

#define MOTOR_HALL_FLG_CLR                               (0)
#define MOTOR_HALL_FLG_SET                               (1)

/* For statemachine */
#define MOTOR_HALL_STATEMACHINE_SIZE_STATE               (3)
#define MOTOR_HALL_STATEMACHINE_SIZE_EVENT               (4)

/* State machine error flags */
#define MOTOR_HALL_STATEMACHINE_ERROR_NONE               (0x00) /* None error flag */
#define MOTOR_HALL_STATEMACHINE_ERROR_EVENTOUTBOUND      (0x01) /* The event index is out of bound */
#define MOTOR_HALL_STATEMACHINE_ERROR_STATEOUTBOUND      (0x02) /* The state index is out of bound */
#define MOTOR_HALL_STATEMACHINE_ERROR_ACTIONEXCEPTION    (0x04) /* The action function returns failure */

#ifndef MOTOR_HALL_ERROR_RETURN

 #define  MOTOR_HALL_ERROR_RETURN(a, err)    FSP_ERROR_RETURN((a), (err))
#endif

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/

/* Speed control <=> Current control interface functions */
void rm_motor_hall_current_callback(motor_current_callback_args_t * p_args);
void rm_motor_hall_speed_callback(motor_speed_callback_args_t * p_args);

static uint16_t rm_motor_hall_error_check(motor_hall_instance_ctrl_t * p_ctrl,
                                          float                        f_iu,
                                          float                        f_iv,
                                          float                        f_iw,
                                          float                        f_vdc,
                                          float                        f_speed);
static void rm_motor_hall_init_speed_input(motor_speed_input_t * st_input);
static void rm_motor_hall_init_speed_output(motor_speed_output_t * st_output);
static void rm_motor_hall_copy_speed_current(motor_speed_output_t * st_output, motor_current_input_t * st_input);
static void rm_motor_hall_copy_current_speed(motor_current_output_t * st_output, motor_speed_input_t * st_input);

/* Action functions */
static uint8_t rm_motor_hall_active(motor_hall_instance_ctrl_t * p_ctrl);
static uint8_t rm_motor_hall_inactive(motor_hall_instance_ctrl_t * p_ctrl);
static uint8_t rm_motor_hall_reset(motor_hall_instance_ctrl_t * p_ctrl);
static uint8_t rm_motor_hall_error(motor_hall_instance_ctrl_t * p_ctrl);
static uint8_t rm_motor_hall_nowork(motor_hall_instance_ctrl_t * p_ctrl);

static const uint8_t state_transition_table[MOTOR_HALL_STATEMACHINE_SIZE_EVENT][
    MOTOR_HALL_STATEMACHINE_SIZE_STATE] =
{
/* State              0:MOTOR_HALL_CTRL_STATUS_STOP, 1:MOTOR_HALL_CTRL_STATUS_RUN, 2:MOTOR_HALL_CTRL_STATUS_ERROR */
/* Event */
/* 0:EVENT_STOP     */ {MOTOR_HALL_CTRL_STATUS_STOP,  MOTOR_HALL_CTRL_STATUS_STOP,   MOTOR_HALL_CTRL_STATUS_ERROR     },

/* 1:EVENT_RUN      */ {MOTOR_HALL_CTRL_STATUS_RUN,   MOTOR_HALL_CTRL_STATUS_RUN,    MOTOR_HALL_CTRL_STATUS_ERROR     },

/* 2:EVENT_ERROR    */ {MOTOR_HALL_CTRL_STATUS_ERROR, MOTOR_HALL_CTRL_STATUS_ERROR,  MOTOR_HALL_CTRL_STATUS_ERROR     },

/* 3:EVENT_RESET    */ {MOTOR_HALL_CTRL_STATUS_STOP,  MOTOR_HALL_CTRL_STATUS_ERROR,  MOTOR_HALL_CTRL_STATUS_STOP      },
};

typedef uint8_t (* motor_hall_action_t)(motor_hall_instance_ctrl_t * p_ctrl);

static const motor_hall_action_t motor_hall_action_table[MOTOR_HALL_STATEMACHINE_SIZE_EVENT][
    MOTOR_HALL_STATEMACHINE_SIZE_STATE] =
{
/* State                    0:STOP,                   1:RUN,                 2:ERROR */
/* Event */
/* 0:EVENT_STOP     */ {rm_motor_hall_inactive, rm_motor_hall_inactive, rm_motor_hall_nowork      },

/* 1:EVENT_RUN      */ {rm_motor_hall_active,   rm_motor_hall_nowork,   rm_motor_hall_nowork      },

/* 2:EVENT_ERROR    */ {rm_motor_hall_error,    rm_motor_hall_error,    rm_motor_hall_nowork      },

/* 3:EVENT_RESET    */ {rm_motor_hall_reset,    rm_motor_hall_error,    rm_motor_hall_reset       },
};

/* Statemachine functions */
static void     rm_motor_hall_statemachine_init(motor_hall_statemachine_t * p_state_machine);
static void     rm_motor_hall_statemachine_reset(motor_hall_statemachine_t * p_state_machine);
static uint16_t rm_motor_hall_statemachine_event(motor_hall_instance_ctrl_t * p_ctrl, motor_hall_ctrl_event_t u1_event);

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
const motor_api_t g_motor_on_motor_hall =
{
    .open            = RM_MOTOR_HALL_Open,
    .close           = RM_MOTOR_HALL_Close,
    .run             = RM_MOTOR_HALL_Run,
    .stop            = RM_MOTOR_HALL_Stop,
    .reset           = RM_MOTOR_HALL_Reset,
    .errorSet        = RM_MOTOR_HALL_ErrorSet,
    .speedSet        = RM_MOTOR_HALL_SpeedSet,
    .statusGet       = RM_MOTOR_HALL_StatusGet,
    .angleGet        = RM_MOTOR_HALL_AngleGet,
    .speedGet        = RM_MOTOR_HALL_SpeedGet,
    .errorCheck      = RM_MOTOR_HALL_ErrorCheck,
    .positionSet     = RM_MOTOR_HALL_PositionSet,
    .waitStopFlagGet = RM_MOTOR_HALL_WaitStopFlagGet,
    .functionSelect  = RM_MOTOR_HALL_FunctionSelect,
};

/*******************************************************************************************************************//**
 * @addtogroup MOTOR_HALL
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * Configure the MOTOR HALL in register start mode. Implements @ref motor_api_t::open.
 *
 * This function should only be called once as MOTOR configuration registers can only be written to once so subsequent
 * calls will have no effect.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_Open
 *
 * @retval FSP_SUCCESS              MOTOR HALL successfully configured.
 * @retval FSP_ERR_ASSERTION        Null pointer, or one or more configuration options is invalid.
 * @retval FSP_ERR_ALREADY_OPEN     Module is already open.  This module can only be opened once.
 * @retval FSP_ERR_INVALID_ARGUMENT Configuration parameter error.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_Open (motor_ctrl_t * const p_ctrl, motor_cfg_t const * const p_cfg)
{
    fsp_err_t err = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ASSERT(NULL != p_cfg);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN != p_instance_ctrl->open, FSP_ERR_ALREADY_OPEN);

    /* Using modules' instance check */
    MOTOR_HALL_ERROR_RETURN(NULL != p_cfg->p_motor_speed_instance, FSP_ERR_ASSERTION);
    MOTOR_HALL_ERROR_RETURN(NULL != p_cfg->p_motor_current_instance, FSP_ERR_ASSERTION);

    MOTOR_HALL_ERROR_RETURN(NULL != p_cfg->p_extend, FSP_ERR_ASSERTION);

    motor_hall_extended_cfg_t * p_extended_cfg = (motor_hall_extended_cfg_t *) p_cfg->p_extend;

    MOTOR_HALL_ERROR_RETURN(p_extended_cfg->f_overcurrent_limit >= 0.0F, FSP_ERR_INVALID_ARGUMENT);
    MOTOR_HALL_ERROR_RETURN(p_extended_cfg->f_overvoltage_limit >= 0.0F, FSP_ERR_INVALID_ARGUMENT);
    MOTOR_HALL_ERROR_RETURN(p_extended_cfg->f_overspeed_limit >= 0.0F, FSP_ERR_INVALID_ARGUMENT);
    MOTOR_HALL_ERROR_RETURN(p_extended_cfg->f_lowvoltage_limit >= 0.0F, FSP_ERR_INVALID_ARGUMENT);
#endif

    p_instance_ctrl->p_cfg = p_cfg;

    p_cfg->p_motor_current_instance->p_api->open(p_cfg->p_motor_current_instance->p_ctrl,
                                                 p_cfg->p_motor_current_instance->p_cfg);

    p_cfg->p_motor_speed_instance->p_api->open(p_cfg->p_motor_speed_instance->p_ctrl,
                                               p_cfg->p_motor_speed_instance->p_cfg);

    p_instance_ctrl->u2_error_info = MOTOR_ERROR_NONE;

    rm_motor_hall_statemachine_init(&(p_instance_ctrl->st_statem));

    rm_motor_hall_init_speed_input(&(p_instance_ctrl->st_speed_input));
    rm_motor_hall_init_speed_output(&(p_instance_ctrl->st_speed_output));

    /* Mark driver as open */
    p_instance_ctrl->open = MOTOR_HALL_OPEN;

    return err;
}

/*******************************************************************************************************************//**
 * @brief Disables specified Motor Hall Control block. Implements @ref motor_api_t::close.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_Close
 *
 * @retval FSP_SUCCESS              Successfully closed.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_Close (motor_ctrl_t * const p_ctrl)
{
    fsp_err_t err = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);

    /* Using modules' instance check */
    MOTOR_HALL_ERROR_RETURN(NULL != p_instance_ctrl->p_cfg->p_motor_speed_instance, FSP_ERR_ASSERTION);
    MOTOR_HALL_ERROR_RETURN(NULL != p_instance_ctrl->p_cfg->p_motor_current_instance, FSP_ERR_ASSERTION);
#endif

    /* Close using modules */
    p_instance_ctrl->p_cfg->p_motor_speed_instance->p_api->close(p_instance_ctrl->p_cfg->p_motor_speed_instance->p_ctrl);

    p_instance_ctrl->p_cfg->p_motor_current_instance->p_api->close(
        p_instance_ctrl->p_cfg->p_motor_current_instance->p_ctrl);

    rm_motor_hall_init_speed_input(&(p_instance_ctrl->st_speed_input));
    rm_motor_hall_init_speed_output(&(p_instance_ctrl->st_speed_output));

    p_instance_ctrl->open = 0;

    return err;
}

/*******************************************************************************************************************//**
 * @brief Reset Motor Hall Control block. Implements @ref motor_api_t::reset.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_Reset
 *
 * @retval FSP_SUCCESS              Successfully resetted.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_Reset (motor_ctrl_t * const p_ctrl)
{
    fsp_err_t err = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    /* Reset is valid only at "error" status */
    if (MOTOR_ERROR_NONE != p_instance_ctrl->u2_error_info)
    {
        rm_motor_hall_statemachine_event(p_instance_ctrl, MOTOR_HALL_CTRL_EVENT_RESET);

        rm_motor_hall_init_speed_input(&(p_instance_ctrl->st_speed_input));
        rm_motor_hall_init_speed_output(&(p_instance_ctrl->st_speed_output));

        /* Clear error information */
        p_instance_ctrl->u2_error_info = MOTOR_ERROR_NONE;
    }

    return err;
}

/*******************************************************************************************************************//**
 * @brief Run Motor (Start motor rotation). Implements @ref motor_api_t::run.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_Run
 *
 * @retval FSP_SUCCESS              Successfully resetted.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_Run (motor_ctrl_t * const p_ctrl)
{
    fsp_err_t err = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    rm_motor_hall_statemachine_event(p_instance_ctrl, MOTOR_HALL_CTRL_EVENT_RUN);

    return err;
}

/*******************************************************************************************************************//**
 * @brief Stop Motor (Stop motor rotation). Implements @ref motor_api_t::stop.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_Stop
 *
 * @retval FSP_SUCCESS              Successfully resetted.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_Stop (motor_ctrl_t * const p_ctrl)
{
    fsp_err_t err = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    rm_motor_hall_statemachine_event(p_instance_ctrl, MOTOR_HALL_CTRL_EVENT_STOP);

    return err;
}

/*******************************************************************************************************************//**
 * @brief Set error information. Implements @ref motor_api_t::errorSet.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_ErrorSet
 *
 * @retval FSP_SUCCESS              Successfully resetted.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_ErrorSet (motor_ctrl_t * const p_ctrl, motor_error_t const error)
{
    fsp_err_t err = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    p_instance_ctrl->u2_error_info |= (uint16_t) error;
    rm_motor_hall_statemachine_event(p_instance_ctrl, MOTOR_HALL_CTRL_EVENT_ERROR);

    return err;
}

/*******************************************************************************************************************//**
 * @brief Set speed reference[rpm]. Implements @ref motor_api_t::speedSet.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_SpeedSet
 *
 * @retval FSP_SUCCESS              Successfully resetted.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_SpeedSet (motor_ctrl_t * const p_ctrl, float const speed_rpm)
{
    fsp_err_t err = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    if (p_instance_ctrl->p_cfg->p_motor_speed_instance != NULL)
    {
        err = p_instance_ctrl->p_cfg->p_motor_speed_instance->p_api->speedReferenceSet(
            p_instance_ctrl->p_cfg->p_motor_speed_instance->p_ctrl,
            speed_rpm);
    }

    return err;
}

/*******************************************************************************************************************//**
 * @brief Get current control status. Implements @ref motor_api_t::statusGet.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_StatusGet
 *
 * @retval FSP_SUCCESS              Successfully resetted.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 * @retval FSP_ERR_INVALID_ARGUMENT Data received pointer is invalid..
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_StatusGet (motor_ctrl_t * const p_ctrl, uint8_t * const p_status)
{
    fsp_err_t err = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
    MOTOR_HALL_ERROR_RETURN(p_status != NULL, FSP_ERR_INVALID_ARGUMENT);
#endif

    *p_status = (uint8_t) (p_instance_ctrl->st_statem.status);

    return err;
}

/*******************************************************************************************************************//**
 * @brief Get current rotor angle. Implements @ref motor_api_t::angleGet.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_AngleGet
 *
 * @retval FSP_SUCCESS              Successfully resetted.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 * @retval FSP_ERR_INVALID_ARGUMENT Data received pointer is invalid..
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_AngleGet (motor_ctrl_t * const p_ctrl, float * const p_angle_rad)
{
    motor_current_output_t       temp_current_output;
    fsp_err_t                    err             = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;
    temp_current_output.f_rotor_angle = 0.0F;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
    MOTOR_HALL_ERROR_RETURN(p_angle_rad != NULL, FSP_ERR_INVALID_ARGUMENT);
#endif

    motor_current_instance_t const * p_current_instance = p_instance_ctrl->p_cfg->p_motor_current_instance;

    if (p_current_instance != NULL)
    {
        err = p_current_instance->p_api->parameterGet(p_current_instance->p_ctrl, &temp_current_output);
    }

    *p_angle_rad = temp_current_output.f_rotor_angle;

    return err;
}

/*******************************************************************************************************************//**
 * @brief Get rotational speed. Implements @ref motor_api_t::speedGet.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_SpeedGet
 *
 * @retval FSP_SUCCESS              Successfully resetted.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 * @retval FSP_ERR_INVALID_ARGUMENT Data received pointer is invalid..
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_SpeedGet (motor_ctrl_t * const p_ctrl, float * const p_speed_rpm)
{
    motor_current_output_t       temp_current_output;
    fsp_err_t                    err             = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;
    temp_current_output.f_speed_rpm = 0.0F;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
    MOTOR_HALL_ERROR_RETURN(p_speed_rpm != NULL, FSP_ERR_INVALID_ARGUMENT);
#endif

    motor_current_instance_t const * p_current_instance = p_instance_ctrl->p_cfg->p_motor_current_instance;

    if (p_current_instance != NULL)
    {
        err = p_current_instance->p_api->parameterGet(p_current_instance->p_ctrl, &temp_current_output);
    }

    *p_speed_rpm = temp_current_output.f_speed_rpm;

    return err;
}

/*******************************************************************************************************************//**
 * @brief Check the occurunce of Error. Implements @ref motor_api_t::errorCheck.
 *
 * Example:
 * @snippet rm_motor_hall_example.c RM_MOTOR_HALL_ErrorCheck
 *
 * @retval FSP_SUCCESS              Successfully resetted.
 * @retval FSP_ERR_ASSERTION        Null pointer.
 * @retval FSP_ERR_NOT_OPEN         Module is not open.
 * @retval FSP_ERR_INVALID_ARGUMENT Data received pointer is invalid..
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_ErrorCheck (motor_ctrl_t * const p_ctrl, uint16_t * const p_error)
{
    motor_current_output_t temp_current_output;
    float temp_iu = 0.0F;
    float temp_iv = 0.0F;
    float temp_iw = 0.0F;
    motor_driver_current_get_t temp_crnt_get;
    temp_crnt_get.vdc = 0.0F;

    fsp_err_t err = FSP_SUCCESS;
    motor_hall_instance_ctrl_t * p_instance_ctrl = (motor_hall_instance_ctrl_t *) p_ctrl;

#if MOTOR_HALL_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(p_instance_ctrl);
    MOTOR_HALL_ERROR_RETURN(MOTOR_HALL_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
    MOTOR_HALL_ERROR_RETURN(p_error != NULL, FSP_ERR_INVALID_ARGUMENT);
#endif

    motor_current_instance_t const * p_current_instance =
        p_instance_ctrl->p_cfg->p_motor_current_instance;
    motor_driver_instance_t const * p_driver_instance =
        p_instance_ctrl->p_cfg->p_motor_current_instance->p_cfg->p_motor_driver_instance;

    motor_current_instance_ctrl_t * p_current_instance_ctrl =
        (motor_current_instance_ctrl_t *) p_instance_ctrl->p_cfg->p_motor_current_instance->p_ctrl;

    if (p_current_instance != NULL)
    {
        err = p_current_instance->p_api->parameterGet(p_current_instance->p_ctrl, &temp_current_output);
    }

    if (p_driver_instance != NULL)
    {
        err = p_driver_instance->p_api->currentGet(p_driver_instance->p_ctrl, &temp_crnt_get);
        if (MOTOR_HALL_FLG_SET == p_current_instance_ctrl->u1_flag_crnt_offset)
        {
            temp_iu = temp_crnt_get.iu;
            temp_iw = temp_crnt_get.iw;
            temp_iv = -(temp_iu + temp_iw);
        }
    }

    p_instance_ctrl->u2_error_info |=
        rm_motor_hall_error_check(p_instance_ctrl,
                                  temp_iu,
                                  temp_iv,
                                  temp_iw,
                                  temp_crnt_get.vdc,
                                  temp_current_output.f_speed_rpm);

    *p_error = p_instance_ctrl->u2_error_info;

    if (MOTOR_ERROR_NONE != p_instance_ctrl->u2_error_info)
    {
        rm_motor_hall_statemachine_event(p_instance_ctrl, MOTOR_HALL_CTRL_EVENT_ERROR);
    }

    return err;
}

/*******************************************************************************************************************//**
 * @brief Set position reference. Implements @ref motor_api_t::positionSet.
 *
 * @retval FSP_ERR_UNSUPPORTED      Unsupported.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_PositionSet (motor_ctrl_t * const p_ctrl, motor_speed_position_data_t const * const p_position)
{
    FSP_PARAMETER_NOT_USED(p_ctrl);
    FSP_PARAMETER_NOT_USED(p_position);

    return FSP_ERR_UNSUPPORTED;
}

/*******************************************************************************************************************//**
 * @brief Get wait stop flag. Implements @ref motor_api_t::waitStopFlagGet.
 *
 * @retval FSP_ERR_UNSUPPORTED      Unsupported.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_WaitStopFlagGet (motor_ctrl_t * const p_ctrl, motor_wait_stop_flag_t * const p_flag)
{
    FSP_PARAMETER_NOT_USED(p_ctrl);
    FSP_PARAMETER_NOT_USED(p_flag);

    return FSP_ERR_UNSUPPORTED;
}

/*******************************************************************************************************************//**
 * @brief Select function. Implements @ref motor_api_t::functionSelect.
 *
 * @retval FSP_ERR_UNSUPPORTED      Unsupported.
 *
 * @note
 *
 **********************************************************************************************************************/
fsp_err_t RM_MOTOR_HALL_FunctionSelect (motor_ctrl_t * const p_ctrl, motor_function_select_t const function)
{
    FSP_PARAMETER_NOT_USED(p_ctrl);
    FSP_PARAMETER_NOT_USED(function);

    return FSP_ERR_UNSUPPORTED;
}

/*******************************************************************************************************************//**
 * @} (end addtogroup MOTOR_HALL)
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private functions
 **********************************************************************************************************************/

/*****  For status transition *****/

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_active
 * Description   : Activates the motor control system and enables PWM output
 * Arguments     : p_ctrl  - Pointer to Hall Motor control structure
 * Return Value  : error information
 **********************************************************************************************************************/
static uint8_t rm_motor_hall_active (motor_hall_instance_ctrl_t * p_ctrl)
{
    fsp_err_t err = FSP_SUCCESS;
    if (p_ctrl->p_cfg->p_motor_speed_instance != NULL)
    {
        err = p_ctrl->p_cfg->p_motor_speed_instance->p_api->run(p_ctrl->p_cfg->p_motor_speed_instance->p_ctrl);
    }

    if (p_ctrl->p_cfg->p_motor_current_instance != NULL)
    {
        err = p_ctrl->p_cfg->p_motor_current_instance->p_api->run(p_ctrl->p_cfg->p_motor_current_instance->p_ctrl);
    }

    return (uint8_t) err;
}                                      /* End of function rm_motor_hall_active() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_inactive
 * Description   : Deactivates the motor control system and disables PWM output
 * Arguments     : p_ctrl  - Pointer to Hall Motor control structure
 * Return Value  : error information
 **********************************************************************************************************************/
static uint8_t rm_motor_hall_inactive (motor_hall_instance_ctrl_t * p_ctrl)
{
    fsp_err_t err = FSP_SUCCESS;
    if (p_ctrl->p_cfg->p_motor_speed_instance != NULL)
    {
        err = p_ctrl->p_cfg->p_motor_speed_instance->p_api->reset(p_ctrl->p_cfg->p_motor_speed_instance->p_ctrl);
    }

    if (p_ctrl->p_cfg->p_motor_current_instance != NULL)
    {
        err = p_ctrl->p_cfg->p_motor_current_instance->p_api->reset(p_ctrl->p_cfg->p_motor_current_instance->p_ctrl);
    }

    return (uint8_t) err;
}                                      /* End of function rm_motor_hall_inactive() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_nowork
 * Description   : The empty dummy function used to fill the blank in action table
 * Arguments     : p_ctrl  - Pointer to Hall Motor control structure
 * Return Value  : Always success (0)
 **********************************************************************************************************************/
static uint8_t rm_motor_hall_nowork (motor_hall_instance_ctrl_t * p_ctrl)
{
    if (p_ctrl->p_cfg->p_motor_speed_instance != NULL)
    {
        /* Do nothing */
    }

    return 0;
}                                      /* End of function rm_motor_hall_nowork() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_reset
 * Description   : Resets the configurations to default and clear error status
 * Arguments     : p_ctrl  - Pointer to Hall Motor control structure
 * Return Value  : The result of action, Always success(0)
 **********************************************************************************************************************/
static uint8_t rm_motor_hall_reset (motor_hall_instance_ctrl_t * p_ctrl)
{
    if (p_ctrl->p_cfg->p_motor_speed_instance != NULL)
    {
        p_ctrl->p_cfg->p_motor_speed_instance->p_api->reset(p_ctrl->p_cfg->p_motor_speed_instance->p_ctrl);
    }

    if (p_ctrl->p_cfg->p_motor_current_instance != NULL)
    {
        p_ctrl->p_cfg->p_motor_current_instance->p_api->reset(p_ctrl->p_cfg->p_motor_current_instance->p_ctrl);
    }

    return 0;
}                                      /* End of function rm_motor_hall_reset() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_error
 * Description   : Executes the post-processing (include stopping the PWM output) when an error has been detected
 * Arguments     : p_ctrl  - Pointer to Hall Motor control structure
 * Return Value  : The result of action, Always success(0)
 **********************************************************************************************************************/
static uint8_t rm_motor_hall_error (motor_hall_instance_ctrl_t * p_ctrl)
{
    if (p_ctrl->p_cfg->p_motor_speed_instance != NULL)
    {
        p_ctrl->p_cfg->p_motor_speed_instance->p_api->reset(p_ctrl->p_cfg->p_motor_speed_instance->p_ctrl);
    }

    if (p_ctrl->p_cfg->p_motor_current_instance != NULL)
    {
        p_ctrl->p_cfg->p_motor_current_instance->p_api->reset(p_ctrl->p_cfg->p_motor_current_instance->p_ctrl);
    }

    return 0;
}                                      /* End of function rm_motor_hall_error() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_statemachine_init
 * Description   : Initializes state machine for motor drive system
 * Arguments     : p_state_machine   - the pointer to the state machine data structure
 * Return Value  : None
 **********************************************************************************************************************/
static void rm_motor_hall_statemachine_init (motor_hall_statemachine_t * p_state_machine)
{
    rm_motor_hall_statemachine_reset(p_state_machine);
}                                      /* End of function rm_motor_hall_statemachine_init() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_statemachine_reset
 * Description   : Resets state machine
 * Arguments     : p_state_machine   - the pointer to the state machine data structure
 * Return Value  : None
 **********************************************************************************************************************/
static void rm_motor_hall_statemachine_reset (motor_hall_statemachine_t * p_state_machine)
{
    p_state_machine->status          = MOTOR_HALL_CTRL_STATUS_STOP;
    p_state_machine->next_status     = MOTOR_HALL_CTRL_STATUS_STOP;
    p_state_machine->u2_error_status = MOTOR_HALL_STATEMACHINE_ERROR_NONE;
}                                      /* End of function rm_motor_hall_statemachine_reset() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_statemachine_event
 * Description   : Performs state transition and execute corresponding action when
 *                 specified event happen
 * Arguments     : p_ctrl  - the pointer to the structure of motor hall control
 *                 u1_event          - the event index to be executed
 *                 MOTOR_HALL_CTRL_EVENT_STOP    Stop the motor drive system
 *                 MOTOR_HALL_CTRL_EVENT_RUN     Activate the motor drive system
 *                 MOTOR_HALL_CTRL_EVENT_ERROR   Throw an error and stop driving
 *                 MOTOR_HALL_CTRL_EVENT_RESET   Reset the configurations of motor drive system
 * Return Value  : The error flags of the state machine
 *                 BIT0: Event index is out of bound, please check the u1_event
 *                 BIT1: State index is out of bound
 *                 BIT2: Action error (value other than 0 returned from action)
 **********************************************************************************************************************/
static uint16_t rm_motor_hall_statemachine_event (motor_hall_instance_ctrl_t * p_ctrl, motor_hall_ctrl_event_t u1_event)
{
    motor_hall_action_t func_action;
    uint8_t             action_ret;

    p_ctrl->st_statem.u2_error_status = MOTOR_HALL_STATEMACHINE_ERROR_NONE;

    /* Check if accessing state transition table out of bound */
    if (MOTOR_HALL_STATEMACHINE_SIZE_EVENT <= u1_event)
    {
        /* Event is out of bound */
        u1_event = MOTOR_HALL_CTRL_EVENT_ERROR;
        p_ctrl->st_statem.u2_error_status |= MOTOR_HALL_STATEMACHINE_ERROR_EVENTOUTBOUND;
    }

    if (MOTOR_HALL_STATEMACHINE_SIZE_STATE <= p_ctrl->st_statem.status)
    {
        /* State is out of bound */
        u1_event                           = MOTOR_HALL_CTRL_EVENT_ERROR;
        p_ctrl->st_statem.status           = MOTOR_HALL_CTRL_STATUS_STOP;
        p_ctrl->st_statem.u2_error_status |= MOTOR_HALL_STATEMACHINE_ERROR_STATEOUTBOUND;
    }

    /*
     * current_event : Event happening
     * status        : Current status
     * next_status   : Status after action executed
     */
    p_ctrl->st_statem.current_event = u1_event;
    p_ctrl->st_statem.next_status   =
        (motor_hall_ctrl_status_t) (state_transition_table[u1_event][p_ctrl->st_statem.status]);

    /* Get action function from action table and execute action */
    func_action = motor_hall_action_table[u1_event][p_ctrl->st_statem.status];
    action_ret  = func_action(p_ctrl);

    /* If return value is not zero, set the action exception flag */
    if (action_ret != 0U)
    {
        p_ctrl->st_statem.u2_error_status |= MOTOR_HALL_STATEMACHINE_ERROR_ACTIONEXCEPTION;
    }

    p_ctrl->st_statem.status = p_ctrl->st_statem.next_status;

    return p_ctrl->st_statem.u2_error_status;
}                                      /* End of function rm_motor_hall_statemachine_event() */

/***********************************************************************************************************************
 * Function Name : rm_motor_check_over_speed_error
 * Description   : Checks over-speed error
 * Arguments     : f4_speed - The electrical speed
 *                 f4_speed_limit - The speed threshold of the over-speed error, should be a positive value
 * Return Value  : The over-speed error flag
 **********************************************************************************************************************/
static inline uint16_t rm_motor_check_over_speed_error (float f4_speed, float f4_speed_limit)
{
    float    f4_temp0;
    uint16_t u2_temp0;

    u2_temp0 = MOTOR_ERROR_NONE;

    f4_temp0 = fabsf(f4_speed);
    if (f4_temp0 > f4_speed_limit)
    {
        u2_temp0 = MOTOR_ERROR_OVER_SPEED;
    }

    return u2_temp0;
}                                      /* End of function rm_motor_check_over_speed_error() */

/***********************************************************************************************************************
 * Function Name : rm_motor_check_over_voltage_error
 * Description   : Checks over-voltage error
 * Arguments     : f4_vdc - The actual Vdc value [V]
 *                 f4_overvoltage_limit - The threshold voltage[V] of over-voltage error
 * Return Value  : The flag of over-voltage error
 **********************************************************************************************************************/
static inline uint16_t rm_motor_check_over_voltage_error (float f4_vdc, float f4_overvoltage_limit)
{
    uint16_t u2_temp0;

    u2_temp0 = MOTOR_ERROR_NONE;

    if (f4_vdc > f4_overvoltage_limit)
    {
        u2_temp0 = MOTOR_ERROR_OVER_VOLTAGE;
    }

    return u2_temp0;
}                                      /* End of function rm_motor_check_over_voltage_error() */

/***********************************************************************************************************************
 * Function Name : rm_motor_check_low_voltage_error
 * Description   : Checks low-voltage error
 * Arguments     : f4_vdc - The actual Vdc value [V]
 *                 f4_lowvoltage_limit - The threshold voltage[V] of low-voltage error
 * Return Value  : The flag of low-voltage error
 **********************************************************************************************************************/
static inline uint16_t rm_motor_check_low_voltage_error (float f4_vdc, float f4_lowvoltage_limit)
{
    uint16_t u2_temp0;

    u2_temp0 = MOTOR_ERROR_NONE;

    if (f4_vdc < f4_lowvoltage_limit)
    {
        u2_temp0 = MOTOR_ERROR_LOW_VOLTAGE;
    }

    return u2_temp0;
}                                      /* End of function rm_motor_check_low_voltage_error() */

/***********************************************************************************************************************
 * Function Name : rm_motor_check_over_current_error
 * Description   : Checks over-current error
 * Arguments     : f4_iu - The actual U-phase current value [A]
 *                 f4_iv - The actual V-phase current value [A]
 *                 f4_iw - The actual W-phase current value [A]
 *                 f4_oc_limit - The threshold current[A] of over-current error
 * Return Value  : The flag of over-current error
 **********************************************************************************************************************/
static inline uint16_t rm_motor_check_over_current_error (float f4_iu, float f4_iv, float f4_iw, float f4_oc_limit)
{
    float    f4_temp0 = 0.0F;
    uint16_t u2_temp0 = 0U;

    u2_temp0 = MOTOR_ERROR_NONE;       /* Initialize */

    f4_temp0 = fabsf(f4_iu);
    if (f4_temp0 > f4_oc_limit)
    {
        u2_temp0 = 1U;
    }

    f4_temp0 = fabsf(f4_iv);
    if (f4_temp0 > f4_oc_limit)
    {
        u2_temp0 = 1U;
    }

    f4_temp0 = fabsf(f4_iw);
    if (f4_temp0 > f4_oc_limit)
    {
        u2_temp0 = 1U;
    }

    if (1U == u2_temp0)
    {
        u2_temp0 = MOTOR_ERROR_OVER_CURRENT_SW;
    }

    return u2_temp0;
}                                      /* End of function rm_motor_check_over_current_error() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_error_check
 * Description   : Checks the errors
 * Arguments     : p_ctrl - Pointer to Hall Motor control structure
 *               : f_iu  - U phase current
 *               : f_iv  - V phase current
 *               : f_iw  - W phase current
 *               : f_vdc - Main line voltage
 *               : f_speed - Motor rotational speed [rpm]
 * Return Value  : Detected error information
 **********************************************************************************************************************/
static uint16_t rm_motor_hall_error_check (motor_hall_instance_ctrl_t * p_ctrl,
                                           float                        f_iu,
                                           float                        f_iv,
                                           float                        f_iw,
                                           float                        f_vdc,
                                           float                        f_speed)
{
    uint16_t u2_error_flags = 0U;
    motor_hall_extended_cfg_t * p_extended_cfg = (motor_hall_extended_cfg_t *) p_ctrl->p_cfg->p_extend;

    /* Over current error check */
    u2_error_flags |= rm_motor_check_over_current_error(f_iu, f_iv, f_iw, p_extended_cfg->f_overcurrent_limit);

    /* Over voltage error check */
    u2_error_flags |= rm_motor_check_over_voltage_error(f_vdc, p_extended_cfg->f_overvoltage_limit);

    /* Low voltage error check */
    u2_error_flags |= rm_motor_check_low_voltage_error(f_vdc, p_extended_cfg->f_lowvoltage_limit);

    /* Over speed error check */
    u2_error_flags |= rm_motor_check_over_speed_error(f_speed, p_extended_cfg->f_overspeed_limit);

    return u2_error_flags;
}                                      /* End of function rm_motor_hall_error_check() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_init_speed_input
 * Description   : Initialize speed control input data(structure)
 * Arguments     : st_input - Pointer to the structure of speed control input
 * Return Value  : None
 **********************************************************************************************************************/
static void rm_motor_hall_init_speed_input (motor_speed_input_t * st_input)
{
    st_input->f_id            = 0.0F;
    st_input->f_iq            = 0.0F;
    st_input->f_vamax         = 0.0F;
    st_input->f_speed_rad     = 0.0F;
    st_input->f_ed            = 0.0F;
    st_input->f_eq            = 0.0F;
    st_input->f_phase_err_rad = 0.0F;

    st_input->u1_flag_get_iref = MOTOR_HALL_FLG_CLR;
}                                      /* End of function rm_motor_hall_init_speed_input() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_init_speed_output
 * Description   : Initialize speed control output data(structure)
 * Arguments     : st_output - Pointer to the structure of speed control output
 * Return Value  : None
 **********************************************************************************************************************/
static void rm_motor_hall_init_speed_output (motor_speed_output_t * st_output)
{
    st_output->f_id_ref             = 0.0F;
    st_output->f_iq_ref             = 0.0F;
    st_output->f_ref_speed_rad_ctrl = 0.0F;
    st_output->f_damp_comp_speed    = 0.0F;

    st_output->u1_flag_pi = MOTOR_HALL_FLG_CLR;
}                                      /* End of function rm_motor_hall_init_speed_output() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_copy_speed_current
 * Description   : Copy speed output data to current input data
 * Arguments     : st_output - Pointer to the structure of speed control output
 *                 st_input  - Pointer to the structure of current control input
 * Return Value  : None
 **********************************************************************************************************************/
static void rm_motor_hall_copy_speed_current (motor_speed_output_t * st_output, motor_current_input_t * st_input)
{
    st_input->f_id_ref             = st_output->f_id_ref;
    st_input->f_iq_ref             = st_output->f_iq_ref;
    st_input->f_ref_speed_rad_ctrl = st_output->f_ref_speed_rad_ctrl;
    st_input->f_damp_comp_speed    = st_output->f_damp_comp_speed;
    st_input->u1_flag_pi           = st_output->u1_flag_pi;
}                                      /* End of function rm_motor_hall_copy_speed_current() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_copy_current_speed
 * Description   : Copy current output data to speed input data
 * Arguments     : st_output - Pointer to the structure of current control output
 *                 st_input  - Pointer to the structure of speed control input
 * Return Value  : None
 **********************************************************************************************************************/
static void rm_motor_hall_copy_current_speed (motor_current_output_t * st_output, motor_speed_input_t * st_input)
{
    st_input->f_id             = st_output->f_id;
    st_input->f_iq             = st_output->f_iq;
    st_input->f_vamax          = st_output->f_vamax;
    st_input->f_speed_rad      = st_output->f_speed_rad;
    st_input->f_ed             = st_output->f_ed;
    st_input->f_eq             = st_output->f_eq;
    st_input->f_phase_err_rad  = st_output->f_phase_err_rad;
    st_input->u1_flag_get_iref = st_output->u1_flag_get_iref;
}                                      /* End of function rm_motor_hall_copy_current_speed() */

/*****  Callback function *****/

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_current_callback
 * Description   : Set speed control output data(structure) into current control
 * Arguments     : p_args - current control interface argument
 * Return Value  : None
 **********************************************************************************************************************/
void rm_motor_hall_current_callback (motor_current_callback_args_t * p_args)
{
    motor_instance_t           * p_instance = (motor_instance_t *) p_args->p_context;
    motor_hall_instance_ctrl_t * p_ctrl     = (motor_hall_instance_ctrl_t *) p_instance->p_ctrl;
    motor_callback_args_t        temp_args_t;

    switch (p_args->event)
    {
        case MOTOR_CURRENT_EVENT_FORWARD:
        {
            /* Invoke the callback function if it is set. */
            if (NULL != p_ctrl->p_cfg->p_callback)
            {
                temp_args_t.event     = MOTOR_CALLBACK_EVENT_CURRENT_FORWARD;
                temp_args_t.p_context = p_ctrl->p_cfg->p_context;
                (p_ctrl->p_cfg->p_callback)(&temp_args_t);
            }

            break;
        }

        case MOTOR_CURRENT_EVENT_DATA_SET:
        {
            rm_motor_hall_copy_speed_current(&(p_ctrl->st_speed_output), &(p_ctrl->st_current_input));
            p_ctrl->p_cfg->p_motor_current_instance->p_api->parameterSet(
                p_ctrl->p_cfg->p_motor_current_instance->p_ctrl,
                &(p_ctrl->st_current_input));
            break;
        }

        case MOTOR_CURRENT_EVENT_BACKWARD:
        {
            /* Invoke the callback function if it is set. */
            if (NULL != p_ctrl->p_cfg->p_callback)
            {
                temp_args_t.event     = MOTOR_CALLBACK_EVENT_CURRENT_BACKWARD;
                temp_args_t.p_context = p_ctrl->p_cfg->p_context;
                (p_ctrl->p_cfg->p_callback)(&temp_args_t);
            }

            break;
        }

        default:
        {
            break;
        }
    }
}                                      /* End of function rm_motor_hall_current_callback() */

/***********************************************************************************************************************
 * Function Name : rm_motor_hall_speed_callback
 * Description   : Speed control callback function (Perform at timer cyclic interrupt)
 * Arguments     : p_args - speed control interface argument
 * Return Value  : None
 **********************************************************************************************************************/
void rm_motor_hall_speed_callback (motor_speed_callback_args_t * p_args)
{
    motor_instance_t           * p_instance = (motor_instance_t *) p_args->p_context;
    motor_hall_instance_ctrl_t * p_ctrl     = (motor_hall_instance_ctrl_t *) p_instance->p_ctrl;
    motor_callback_args_t        temp_args_t;

    switch (p_args->event)
    {
        case MOTOR_SPEED_EVENT_FORWARD:
        {
            /* Get speed control input data from current control */
            p_ctrl->p_cfg->p_motor_current_instance->p_api->parameterGet(
                p_ctrl->p_cfg->p_motor_current_instance->p_ctrl,
                &(p_ctrl->st_current_output));
            rm_motor_hall_copy_current_speed(&(p_ctrl->st_current_output), &(p_ctrl->st_speed_input));

            /* Invoke the callback function if it is set. */
            if (NULL != p_ctrl->p_cfg->p_callback)
            {
                temp_args_t.event     = MOTOR_CALLBACK_EVENT_SPEED_FORWARD;
                temp_args_t.p_context = p_ctrl->p_cfg->p_context;
                (p_ctrl->p_cfg->p_callback)(&temp_args_t);
            }

            break;
        }

        case MOTOR_SPEED_EVENT_BACKWARD:
        {
            /* Invoke the callback function if it is set. */
            if (NULL != p_ctrl->p_cfg->p_callback)
            {
                temp_args_t.event     = MOTOR_CALLBACK_EVENT_SPEED_BACKWARD;
                temp_args_t.p_context = p_ctrl->p_cfg->p_context;
                (p_ctrl->p_cfg->p_callback)(&temp_args_t);
            }

            break;
        }

        default:
        {
            break;
        }
    }
}                                      /* End of function rm_motor_hall_speed_callback() */
