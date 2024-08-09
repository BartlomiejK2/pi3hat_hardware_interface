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


#include "controllers/ControllerBridge.hpp"

using namespace controller_interface;
using mjbots::pi3hat::CanFrame;


ControllerBridge::ControllerBridge(
     std::string wrapper_type, 
     const ControllerParameters& params): 
     wrapper_(nullptr), params_(params)
{
    /* Here add your wrapper type as std::string (use "else if" after this "if") */
    if(wrapper_type == "moteus")
    {
         wrapper_ = make_moteus_wrapper(params);
    }

    
    else
    {
        throw std::invalid_argument("Wrong type of wrapper!");
    }
}
     
ControllerBridge::ControllerBridge(ControllerBridge&& other_controller):
    wrapper_(std::move(other_controller.wrapper_)), params_(other_controller.params_) {}

ControllerBridge& ControllerBridge::operator=(ControllerBridge&& other_controller)
{
    if (this != &other_controller)
    {
        this->wrapper_ = std::move(other_controller.wrapper_);
        this->params_ = other_controller.params_;
    }
    return *this;
}

void ControllerBridge::make_command(CanFrame& tx_frame, ControllerCommand& command) const
{
    /* Basic transformations before sending data to wrapper */
    command.position_ = params_.direction_* std::clamp(command.position_,
     params_.position_min_, params_.position_max_) + params_.position_offset_;
    command.velocity_ = params_.direction_* std::clamp(command.velocity_, -params_.velocity_max_, params_.velocity_max_);
    command.torque_ = params_.direction_* std::clamp(command.torque_, -params_.torque_max_, params_.torque_max_);

    tx_frame.expect_reply = true;
    
    wrapper_->command_to_tx_frame(tx_frame, command);
    
}

void ControllerBridge::make_query(CanFrame& tx_frame) const
{
    tx_frame.expect_reply = true;
    wrapper_->query_to_tx_frame(tx_frame);
}

void ControllerBridge::get_state(const CanFrame& rx_frame, ControllerState& state) const
{
    wrapper_->rx_frame_to_state(rx_frame, state);

    /* Basic transformations after getting data from wrapper */
    state.position_ = params_.direction_ * (state.position_ - params_.position_offset_);
    state.velocity_ = params_.direction_ * state.velocity_;
    state.torque_ = params_.direction_ * state.torque_; 
}

void ControllerBridge::initialize(CanFrame& tx_frame) const
{
    tx_frame.expect_reply = true;
    wrapper_->init_to_tx_frame(tx_frame);
}

int ControllerBridge::get_id(const CanFrame& rx_frame)
{
    return wrapper_->get_id_from_rx_frame(rx_frame);
}

ControllerParameters ControllerBridge::get_params()
{
    return params_;
}