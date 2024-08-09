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


#include "imu_transform/IMUTransform.hpp"


using namespace IMU;

void IMUTransform::transform_attitude(mjbots::pi3hat::Attitude& attitude)
{
    /* Comment taken from: https://github.com/G-Levine/pi3hat_hardware_interface/blob/main/src/pi3hat_hardware_interface.cpp */ 
         /* 
            The quaternion returned by the pi3hat is the rotation from the gravity frame to the IMU frame.
                Gravity frame:
                    +x and +y are parallel to the ground
                    +z points towards the ground
                IMU frame:
                    +x points towards the side of the Pi with the USB-C port
                    +y points towards the side of the Pi with the USB-A ports
                    +z points up from the Pi
            However, we want the rotation from the world frame to the IMU frame.
                World frame:
                    +x and +y are parallel to the ground
                    +z points towards the sky
            Therefore, we need to rotate the quaternion returned by the pi3hat by 180 degrees about the x-axis or y-axis. We choose to rotate about the x-axis.
            Let the quaternion returned by the pi3hat be (x, y, z, w).
            After applying a 180-degree rotation about the x-axis, the new quaternion is:
                (w, -z, y, -x)
         */
    
    double temp_x = attitude.attitude.x;
    double temp_y = attitude.attitude.y;

    attitude.attitude.x = attitude.attitude.w;
    attitude.attitude.w = -temp_x;

    attitude.attitude.y = -attitude.attitude.z;
    attitude.attitude.z = temp_y;

    /* degrees/s to radians/s conversion */
    attitude.rate_dps.x *= degrees_to_radians;
    attitude.rate_dps.y *= degrees_to_radians;
    attitude.rate_dps.z *= degrees_to_radians;
}   