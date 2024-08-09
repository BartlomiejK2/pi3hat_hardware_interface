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


#ifndef _MOTEUS_WRAPPER_HPP_
#define _MOTEUS_WRAPPER_HPP_

#include "ControllerWrapper.hpp"
#include "3rd_libs/moteus/moteus.h"
#include <memory>

namespace controller_interface
{

class MoteusWrapper final: public ControllerWrapper 
{
    private:

    /* Const coefficients for easy radians - rotations transform */
    constexpr static double rotation_to_radians_ = 2 * M_PI;
    constexpr static double radians_to_rotation_ = 1 / (2 * M_PI); /* Multiplying is faster than dividing */

    /* Command structure for moteus object*/
    mjbots::moteus::PositionMode::Command position_command_;
    mjbots::moteus::Controller moteus_controller_;
    
    public:
    using CanFrame = mjbots::pi3hat::CanFrame;

    MoteusWrapper( 
        const mjbots::moteus::Controller::Options& options,
        const mjbots::moteus::PositionMode::Command& command);
    void command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) override;
    void query_to_tx_frame(CanFrame& tx_frame) override;
    void rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) override;
    void init_to_tx_frame(CanFrame& tx_frame) override;
    int get_id_from_rx_frame(const CanFrame& rx_frame) override;

};

/* Copying for Moteus class is deleted, prevents from making constructor for MoteusWrapper 
   with only ControllerParameter as argument :/ */
std::unique_ptr<MoteusWrapper> make_moteus_wrapper(const ControllerParameters& params);

};

#endif