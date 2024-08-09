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


#ifndef _IMU_
#define _IMU_

#include <array>
#include "3rd_libs/pi3hat/pi3hat.h"
#include <algorithm>
#include <cmath>

namespace IMU
{   

/* Class for basic transforming filtered IMU state from Pi3hat */
class IMUTransform
{
    private:

    constexpr static double degrees_to_radians = 2 * M_PI / 360.0;

    public:

    /* Function for transforming IMU data */
    void transform_attitude(mjbots::pi3hat::Attitude& attitude);
};

};


#endif