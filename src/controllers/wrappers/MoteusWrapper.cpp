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


#include "controllers/wrappers/MoteusWrapper.hpp"

using namespace controller_interface;

MoteusWrapper::MoteusWrapper(
        const mjbots::moteus::Controller::Options& options,
        const mjbots::moteus::PositionMode::Command& command):

        ControllerWrapper(), 
        position_command_(command),
        moteus_controller_(mjbots::moteus::Controller(options)) {} 


void MoteusWrapper::command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) 
{
    /* Change command values */
    position_command_.position = command.position_ * radians_to_rotation_;
    position_command_.velocity = command.velocity_ * radians_to_rotation_;
    position_command_.feedforward_torque = command.torque_;

    /* Create CANFD frame */
    mjbots::moteus::CanFdFrame can_fd_frame = moteus_controller_.MakePosition(position_command_);
    
    /* Copy data from CANFD frame to CAN frame */
    tx_frame.id = can_fd_frame.arbitration_id;
    tx_frame.bus = can_fd_frame.bus;
    tx_frame.size = can_fd_frame.size;
    std::memcpy(tx_frame.data, can_fd_frame.data, can_fd_frame.size);
}

void MoteusWrapper::query_to_tx_frame(CanFrame& tx_frame)
{
    /* Create CANFD frame */
    mjbots::moteus::CanFdFrame can_fd_frame = moteus_controller_.MakeQuery();
    
    /* Copy data from CANFD frame to TX CANFD Pi3hat frame */
    tx_frame.id = can_fd_frame.arbitration_id;
    tx_frame.bus = can_fd_frame.bus;
    tx_frame.size = can_fd_frame.size;
    std::memcpy(tx_frame.data, can_fd_frame.data, can_fd_frame.size);
}

void MoteusWrapper::rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) 
{
    /* Parse data from RX CANFD Pi3hat frame to Result object */
    if(((rx_frame.id >> 8) & 0x7f) != (uint32_t) moteus_controller_.options().id) return; /* This should not happen! (map frame to wrapper first) */

    mjbots::moteus::Query::Result result = mjbots::moteus::Query::Parse(rx_frame.data, rx_frame.size);
    state.position_ = result.position * rotation_to_radians_;
    state.velocity_ = result.velocity * rotation_to_radians_;
    state.torque_ = result.torque;
    state.temperature_ = result.temperature;
    state.fault = result.fault;
}

void MoteusWrapper::init_to_tx_frame(CanFrame& tx_frame) 
{
    /* create CANFD frame*/
    mjbots::moteus::CanFdFrame can_fd_frame = moteus_controller_.MakeStop();

    /* Copy data from CANFD frame to TX CAN Pi3hat frame */
    tx_frame.id = can_fd_frame.arbitration_id;
    tx_frame.bus = can_fd_frame.bus;
    tx_frame.size = can_fd_frame.size;
    std::memcpy(tx_frame.data, can_fd_frame.data, can_fd_frame.size);
}

int MoteusWrapper::get_id_from_rx_frame(const CanFrame& rx_frame)
{
    /* Get real motor if from RX CAN Pi3hat frame */
    return ((rx_frame.id>> 8) & 0x7f);
}

std::unique_ptr<MoteusWrapper> controller_interface::make_moteus_wrapper(const ControllerParameters& params)
{
    /* Moteus options */ 
    using mjbots::moteus::Controller;
    using controller_interface::MoteusWrapper;
    Controller::Options moteus_options;
    moteus_options.bus = params.bus_;
    moteus_options.id = params.id_;

    /* Moteus command format (it will be copied to wrapper) */
    mjbots::moteus::PositionMode::Format format;
    format.feedforward_torque = mjbots::moteus::kFloat;
    format.maximum_torque = mjbots::moteus::kFloat;
    format.velocity_limit= mjbots::moteus::kFloat;
    moteus_options.position_format = format;

    /* Moteus command (it will be copied to wrapper) */
    mjbots::moteus::PositionMode::Command moteus_command;
    moteus_command.maximum_torque = params.torque_max_;
    //moteus_command.velocity_limit = params.velocity_max_; // Creates error, check later

    controller_interface::MoteusWrapper moteus_wrapper(moteus_options, moteus_command);
    std::unique_ptr<controller_interface::MoteusWrapper> moteus_wrapper_ptr = std::make_unique<controller_interface::MoteusWrapper>(moteus_wrapper);
    return moteus_wrapper_ptr;
}
