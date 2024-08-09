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

namespace controller_interface
{

/* Structure for basic actuator command */
struct ControllerCommand
{
    double position_ = 0;           /* [radians] */
    double velocity_  = 0;          /* [radians/s] */
    double torque_ = 0;             /* [Nm] */
};

/* Structure for basic actuator state */
struct ControllerState
{
    double position_ = 0;           /* [radians] */
    double velocity_ = 0;           /* [radians/s] */
    double torque_ = 0;             /* [Nm] */
    double temperature_ = 0;        /* [Celcius] */
    bool fault = false;            
};

/* Structure for basic controller parameters */
struct ControllerParameters
{
    double position_max_ = 0;       /* [radians] */
    double position_min_ = 0;       /* [radians] */
    double position_offset_ = 0;    /* [radians] */
    double velocity_max_ = 0;       /* [radians/s] */
    double torque_max_ = 0;         /* [Nm] */
    int direction_ = 1;             /* 1 or -1 */
    int id_ = 0;                    /* Usage in your bridge (check moteus bridge) */
    int bus_ = 0;                   /* Usage in your bridge (check moteus bridge) */
};

}

#endif