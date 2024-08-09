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


#ifndef _CONTROLLER_WRAPPER_HPP_
#define _CONTROLLER_WRAPPER_HPP_

#include "../ControllerStructures.hpp"
#include "3rd_libs/pi3hat/pi3hat.h"

namespace controller_interface
{

/* Interface class for wrapper that connects 3rd party lib for controller to connector 
   bridge interface. Check out MoteusWrapper */
class ControllerWrapper
{
    protected:
    
    public:
    using CanFrame = mjbots::pi3hat::CanFrame;

    ControllerWrapper() = default;
    ControllerWrapper(const ControllerWrapper& other) = default;
    ControllerWrapper(ControllerWrapper&& other) = default;
    ControllerWrapper& operator=(const ControllerWrapper& other) = default;
    ControllerWrapper& operator=(ControllerWrapper&& other) = default;


    virtual void command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) = 0;
    virtual void query_to_tx_frame(CanFrame& tx_frame) = 0;
    virtual void rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) = 0;
    virtual void init_to_tx_frame(CanFrame& tx_frame) = 0;
    virtual int get_id_from_rx_frame(const CanFrame& rx_frame) = 0;

    virtual ~ControllerWrapper() = default;

};

};

#endif