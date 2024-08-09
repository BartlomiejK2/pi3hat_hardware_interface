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


#ifndef _CONTROLLER_BRIDGE_HPP_
#define _CONTROLLER_BRIDGE_HPP_

#include "3rd_libs/pi3hat/pi3hat.h"
#include "wrappers/WrappersTypes.hpp"
#include "ControllerStructures.hpp"
#include <exception>
#include <memory>
#include <algorithm>

namespace controller_interface
{

/* Class for abstracting communication with diffrent type of controllers. 
   Class makes basic transformations for command and state structures before using wrapper */
class ControllerBridge
{
    private:

    using CanFrame = mjbots::pi3hat::CanFrame;

    std::unique_ptr<ControllerWrapper> wrapper_;
    ControllerParameters params_;


    public:
    ControllerBridge(std::string wrapper_type, 
     const ControllerParameters& params);

    ControllerBridge(const ControllerBridge& other_controller) = delete;
    ControllerBridge& operator=(const ControllerBridge& other_controller) = delete;
    ControllerBridge(ControllerBridge&& other_controller);
    ControllerBridge& operator=(ControllerBridge&& other_controller);


    /* Transform controller command to data in TX CAN frame */
    void make_command(CanFrame& tx_frame, ControllerCommand& command) const;

    /* Transform query to data in TX CAN frame */
    void make_query(CanFrame& tx_frame) const;

    /* Transform RX CAN frame to controller state */
    void get_state(const CanFrame& rx_frame, ControllerState& state) const;

    /* Initialize controller */
    void initialize(CanFrame& tx_frame) const;

    /* Get id from RX CAN frame (sometimes raw id from rx frame needs to be transformed)*/
    int get_id(const CanFrame& rx_frame);

    /* Get parameters that were set by user */
    ControllerParameters get_params();

    ~ControllerBridge() = default;
};

};
#endif