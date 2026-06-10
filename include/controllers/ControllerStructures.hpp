/*
 *  Pi3hat Hardware Interface for ROS2 control framework
 *  Copyright (C) 2024 KNR-Melson team
 *
 *  Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
 *  You may obtain a copy of the License at
 *  <http://www.gnu.org/licenses/>.
 * 
 */

/* Author: Bartłomiej Krajewski (https://github.com/BartlomiejK2) */


#ifndef _CONTROLLER_STRUCTURES_HPP_
#define _CONTROLLER_STRUCTURES_HPP_

#include <string>
#include <vector>

namespace controller_interface
{
    /* Structure for basic actuator command */
    struct ControllerCommand
    {
        double position_ = 0;           /* [radians] */
        double velocity_ = 0;           /* [radians/s] */
        double torque_ = 0;             /* [Nm] */
    };

    /* Structure for basic actuator state */
    struct ControllerState
    {
        double position_ = 0.0;           /* [radians] */
        double velocity_ = 0.0;           /* [radians/s] */
        double torque_ = 0.0;             /* [Nm] */
    };

    /* Structure for controller diagnotics */
    struct ControllerDiagnostics
    {
        double mode_ = 0.0;         /* Mode flag */
        double temperature_ = 0.0;  /* [Celcius] */
        double voltage_ = 0.0;      /* [Volt] */
        double current_ = 0.0;      /* [Ampere] */
        double power_ = 0.0;        /* [Wat] */
        double fault_ = 0.0;        /* [Fault flags] */
    };

    /* Structure for additional diagnostics */
    struct AdditionalDiagnostics
    {
        /* Joint diagnostics */
        double position_error_ = 0.0;     /* [radians] */
        double velocity_error_ = 0.0;     /* [radians/s] */
        double desired_position_ = 0.0;   /* [radians] */
        double desired_velocity_ = 0.0;   /* [radians/s] */
        double desired_effort_ = 0.0;     /* [Nm] */

        /* Motor diagnostics */
        double motor_position_error_ = 0.0;     /* [radians] */
        double motor_velocity_error_ = 0.0;     /* [radians/s] */
        double motor_desired_position_ = 0.0;   /* [radians] */
        double motor_desired_velocity_ = 0.0;   /* [radians/s] */
        double motor_desired_effort_ = 0.0;     /* [Nm] */
    };

    /* Structure for basic controller parameters */
    struct ControllerParameters
    {
        double position_max_ = 0.0;     /* [radians] */
        double position_min_ = 0.0;     /* [radians] */
        double position_offset_ = 0.0;  /* [radians] */
        double velocity_max_ = 0.0;     /* [radians/s] */
        double torque_max_ = 0.0;       /* [Nm] */
        int direction_ = 1;             /* 1 or -1 */
        int id_ = 0.0;                  /* Usage in your bridge (check moteus bridge) */
        int bus_ = 0.0;                 /* Usage in your bridge (check moteus bridge) */
    };

    /* Const expressions for all command and state interfaces */ 
    namespace hardware_interface_names
    {
        /* Main joint command and state hardware interfaces */
        constexpr char POSITION[]       = "position";
        constexpr char VELOCITY[]       = "velocity";
        constexpr char EFFORT[]         = "effort";

        /* Main motor state hardware interfaces */
        constexpr char MOTOR_POSITION[] = "motor_position";
        constexpr char MOTOR_VELOCITY[] = "motor_velocity";
        constexpr char MOTOR_EFFORT[]   = "motor_effort";

        /* Diagnostic controller state hardware interface */
        constexpr char MODE[]           = "mode";
        constexpr char TEMPERATURE[]    = "temperature";
        constexpr char VOLTAGE[]        = "voltage";
        constexpr char CURRENT[]        = "current";
        constexpr char POWER[]          = "power";
        constexpr char FAULT[]          = "fault";
        
        /* Diagnostic joint state hardware interfaces */
        constexpr char POSITION_ERROR[]     = "position_error";
        constexpr char VELOCITY_ERROR[]     = "velocity_error";
        constexpr char DESIRED_POSITION[]   = "desired_position";
        constexpr char DESIRED_VELOCITY[]   = "desired_velocity";
        constexpr char DESIRED_EFFORT[]     = "desired_effort";

        /* Diagnostic motor state hardware interfaces */
        constexpr char MOTOR_POSITION_ERROR[]   = "motor_position_error";
        constexpr char MOTOR_VELOCITY_ERROR[]   = "motor_velocity_error";
        constexpr char MOTOR_DESIRED_POSITION[] = "motor_desired_position";
        constexpr char MOTOR_DESIRED_VELOCITY[] = "motor_desired_velocity";
        constexpr char MOTOR_DESIRED_EFFORT[]   = "motor_desired_effort";
    }
}

#endif