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


#include "controllers/Controllers.hpp"
#include "3rd_libs/pi3hat/pi3hat.h"
#include "3rd_libs/pi3hat/realtime.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <string>



static double GetNow() 
{
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
};


int main(int argc, char** argv)
{
    // pi3hat 
    mjbots::pi3hat::Pi3Hat::Configuration pi3hat_configuration;
    pi3hat_configuration.attitude_rate_hz = 1000;


    std::vector<mjbots::pi3hat::CanFrame> tx_frame(2);
    std::vector<mjbots::pi3hat::CanFrame> rx_frame(2);
    tx_frame[0].expect_reply = true;
    tx_frame[1].expect_reply = true;

    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_span(tx_frame.data(), 2);
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_span(rx_frame.data(), 2);
    mjbots::pi3hat::Attitude attitude;

    mjbots::pi3hat::Pi3Hat::Input input;

    input.tx_can = tx_span;
    input.rx_can = rx_span;
    input.attitude = &attitude;
    input.request_attitude = true;

    mjbots::pi3hat::Pi3Hat pi3hat(pi3hat_configuration);
    
    // pi3hat output
    mjbots::pi3hat::Pi3Hat::Output pi3hat_output;

    // moteus wrapper
    controller_interface::ControllerParameters params_1;
    params_1.direction_ = 1;
    params_1.position_max_ = 30 * M_PI;
    params_1.position_min_ = -30 * M_PI;
    params_1.velocity_max_ = 10;
    params_1.torque_max_ = 1;
    params_1.bus_ = 1;
    params_1.id_ = 1;

    controller_interface::ControllerParameters params_2;
    params_2.direction_ = 1;
    params_2.position_max_ = 10 * M_PI;
    params_2.position_min_ = -10 * M_PI;
    params_2.velocity_max_ = 10;
    params_2.torque_max_ = 1;
    params_2.bus_ = 2;
    params_2.id_ = 2;

    // moteusues options
    using mjbots::moteus::Controller;
    Controller::Options moteus_1_options;
    moteus_1_options.bus = 1;
    moteus_1_options.id = 1;

    Controller::Options moteus_2_options;
    moteus_2_options.bus = 2;
    moteus_2_options.id = 2;

    // moteus command format (it will be copied to wrapper)
    mjbots::moteus::PositionMode::Format format;
    format.feedforward_torque = mjbots::moteus::kFloat;
    format.maximum_torque = mjbots::moteus::kFloat;
    moteus_1_options.position_format = format;
    moteus_2_options.position_format = format;

    //moteus command (it will be copied to wrapper)
    mjbots::moteus::PositionMode::Command moteus_1_command;
    moteus_1_command.maximum_torque = params_1.torque_max_;
    moteus_1_command.velocity_limit = params_1.velocity_max_;

    mjbots::moteus::PositionMode::Command moteus_2_command;
    moteus_2_command.maximum_torque = params_2.torque_max_;
    moteus_2_command.velocity_limit = params_2.velocity_max_;


    std::vector<controller_interface::ControllerBridge> controllers;
    std::vector<controller_interface::ControllerCommand> controller_commands;
    std::vector<controller_interface::ControllerState> controller_states;

   
    controller_interface::ControllerBridge controller_1("moteus", params_1); 
    controller_interface::ControllerBridge controller_2("moteus", params_2); 

    controllers.push_back(std::move(controller_1));
    controllers.push_back(std::move(controller_2));
    
    controller_commands.push_back(controller_interface::ControllerCommand());
    controller_commands.push_back(controller_interface::ControllerCommand());
    
    controller_states.push_back(controller_interface::ControllerState());
    controller_states.push_back(controller_interface::ControllerState());


    std::cout << "Options for controllers succesfully initialized!" << std::endl;


    mjbots::pi3hat::ConfigureRealtime(0);
    std::cout << "Realtime control activated!" << std::endl;

    // set stop to moteus
    for(int i = 0; i < controllers.size(); ++i)
    {
        controllers[i].initialize(tx_frame[i]);
    }
    pi3hat_output = pi3hat.Cycle(input);
    ::usleep(10000);

    for(int i = 0; i < controllers.size(); ++i)
    {
        controller_commands[i].position_ = 0;
        controllers[i].make_command(tx_frame[i], controller_commands[i]);
    }
    while(std::abs(controller_states[0].position_) > 0.1 && std::abs(controller_states[1].position_) > 0.1)
    {
        pi3hat_output = pi3hat.Cycle(input);
        ::usleep(2000);
        for(int i = 0; i < controllers.size(); ++i)
        {
            controllers[i].get_state(rx_frame[i], controller_states[i]);
        }
    }
    std::cout << "Controllers successfully started!" << std::endl;

    auto prev = GetNow();
    int frequency;
    while(true)
    {   
        auto now = GetNow();
        controller_commands[0].position_ = 20 * M_PI * cos(now - prev);
        controller_commands[1].position_ = 10 * M_PI *sin(now - prev);
        for(int i = 0; i < controllers.size(); ++i)
        {
            controllers[i].make_command(tx_frame[i], controller_commands[i]);
        }
        pi3hat_output = pi3hat.Cycle(input);
        ::usleep(500);
        auto mesaure_time = GetNow() - now;
        frequency = (int) 1/mesaure_time;
        for(int i = 0; i < controllers.size(); ++i)
        {
            controllers[i].get_state(rx_frame[i], controller_states[i]);
        }
        ::printf("f = %d\n pos_1_command = %7.3f, pos_1_state = %7.3f)\n pos_2_command = %7.3f, pos_2_state = %7.3f)\r",
        frequency, controller_commands[0].position_, controller_states[0].position_, controller_commands[1].position_, controller_states[1].position_);
        ::fflush(::stdout);
    }

    return 0;
}