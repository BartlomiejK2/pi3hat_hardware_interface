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

#include <algorithm>

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
    position_command_.position = command.position_ * radians_to_rotation;
    position_command_.velocity = command.velocity_ * radians_to_rotation;
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

void MoteusWrapper::rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state, 
    ControllerDiagnostics& diagnostics) 
{
    /* Parse data from RX CANFD Pi3hat frame to Result object */
    if(((rx_frame.id >> 8) & 0x7f) != (uint32_t) moteus_controller_.options().id) return; /* This should not happen! (map frame to wrapper first) */

    mjbots::moteus::Query::Result result = mjbots::moteus::Query::Parse(rx_frame.data, rx_frame.size);
    state.position_ = result.position * rotation_to_radians;
    state.velocity_ = result.velocity * rotation_to_radians;
    state.torque_ = result.torque;
    diagnostics.mode_ = static_cast<double>(result.mode);
    diagnostics.fault_ = static_cast<double>(result.fault);
     diagnostics.temperature_ = result.temperature;
    diagnostics.voltage_ = static_cast<double>(result.voltage);
    diagnostics.power_ = result.power;
    diagnostics.current_ = result.power / static_cast<double>(result.voltage);
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

std::unique_ptr<MoteusWrapper> controller_interface::make_moteus_wrapper(const ControllerParameters& params, 
    const std::vector<std::string>& command_interfaces, 
    const std::vector<std::string>& state_interfaces)
{
    /* Moteus options */ 
    using mjbots::moteus::Controller;
    using controller_interface::MoteusWrapper;
    Controller::Options moteus_options;
    moteus_options.bus = params.bus_;
    moteus_options.id = params.id_;

    /* Moteus command format (it will be copied to wrapper) */
    mjbots::moteus::PositionMode::Format command_format;
    command_format.position = mjbots::moteus::kIgnore;
    command_format.velocity = mjbots::moteus::kIgnore;
    command_format.feedforward_torque= mjbots::moteus::kIgnore;

    for(const auto& command_interface: command_interfaces)
    {
        if(command_interface == hardware_interface_names::POSITION)
        {
            command_format.position = mjbots::moteus::kFloat;
        }
        else if(command_interface == hardware_interface_names::VELOCITY)
        {
            command_format.velocity = mjbots::moteus::kFloat;
        }
        else if(command_interface == hardware_interface_names::EFFORT)
        {
            command_format.feedforward_torque = mjbots::moteus::kFloat;
        }
        else
        {
            throw std::runtime_error("Wrong command interface: " + command_interface + " !");
        }
    
    }
    command_format.maximum_torque = mjbots::moteus::kFloat;
    command_format.velocity_limit = mjbots::moteus::kFloat;
    moteus_options.position_format = command_format;

    /* Moteus query format (it will be copied to wrapper) */
    
    mjbots::moteus::Query::Format query_format;
    query_format.position = mjbots::moteus::kIgnore;
    query_format.velocity = mjbots::moteus::kIgnore;
    query_format.torque = mjbots::moteus::kIgnore;
    query_format.mode = mjbots::moteus::kIgnore;
    query_format.fault = mjbots::moteus::kIgnore;
    query_format.d_current = mjbots::moteus::kIgnore;
    query_format.q_current = mjbots::moteus::kIgnore;
    query_format.temperature = mjbots::moteus::kIgnore;
    query_format.voltage = mjbots::moteus::kIgnore;
    query_format.power = mjbots::moteus::kIgnore;

    for(const auto& state_interface: state_interfaces)
    {
        if(state_interface == hardware_interface_names::POSITION)
        {
            query_format.position = mjbots::moteus::kFloat;
        }
        else if(state_interface == hardware_interface_names::VELOCITY)
        {
            query_format.velocity = mjbots::moteus::kFloat;
        }
        else if(state_interface == hardware_interface_names::EFFORT)
        {
            query_format.torque = mjbots::moteus::kFloat;
        }
        else if(state_interface == hardware_interface_names::MODE)
        {
            query_format.mode = mjbots::moteus::kInt8;
        }
        else if(state_interface == hardware_interface_names::FAULT)
        {
            query_format.fault = mjbots::moteus::kInt8;
        }
        else if(state_interface == hardware_interface_names::CURRENT)
        {
            // Not using this at the moment, current is given by power and voltage
            // query_format.d_current = mjbots::moteus::kFloat;
            // query_format.q_current = mjbots::moteus::kFloat;
            query_format.voltage = mjbots::moteus::kInt8;
            query_format.power = mjbots::moteus::kFloat;
        }
        else if(state_interface == hardware_interface_names::TEMPERATURE)
        {
            query_format.temperature = mjbots::moteus::kInt8;
        }
        else if(state_interface == hardware_interface_names::VOLTAGE)
        {
            query_format.voltage = mjbots::moteus::kInt8;
        }
        else if(state_interface == hardware_interface_names::POWER)
        {
            query_format.power = mjbots::moteus::kFloat;
        }
        else
        {
            throw std::runtime_error("Wrong state interface: " + state_interface + " !");
        }
    }
    moteus_options.query_format = query_format;

    /* Moteus command (it will be copied to wrapper) */
    mjbots::moteus::PositionMode::Command moteus_command;
    moteus_command.maximum_torque = params.torque_max_;
    moteus_command.velocity_limit = params.velocity_max_ * radians_to_rotation;

    controller_interface::MoteusWrapper moteus_wrapper(moteus_options, moteus_command);
    std::unique_ptr<controller_interface::MoteusWrapper> moteus_wrapper_ptr = std::make_unique<controller_interface::MoteusWrapper>(moteus_wrapper);
    return moteus_wrapper_ptr;
}
