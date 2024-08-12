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


    mjbots::pi3hat::CanFrame tx_frame;
    mjbots::pi3hat::CanFrame rx_frame;
    tx_frame.expect_reply = true;

    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_span(&tx_frame, 1);
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_span(&rx_frame, 1);
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
    pi3hat_controller_interface::ControllerParameters params;
    params.direction_ = 1;
    params.position_max_ = 10;
    params.position_min_ = -10;
    params.velocity_max_ = 4;
    params.torque_max_ = 1;
    params.bus_ = 1;
    params.id_ = 1;

    pi3hat_controller_interface::MoteusWrapper controller;
    controller.set_parameters(params);


    pi3hat_controller_interface::ControllerCommand controller_command;
    controller_command.velocity_ = 0;
    controller_command.torque_ = 0;

    pi3hat_controller_interface::ControllerState controller_state;

    std::cout << "Options for controller succesfully initialized!" << std::endl;


    mjbots::pi3hat::ConfigureRealtime(0);
    std::cout << "Realtime control activated!" << std::endl;

    // set stop to moteus
    controller.init_to_tx_frame(tx_frame);
    pi3hat_output = pi3hat.Cycle(input);
    ::usleep(10000);
    
    controller_command.position_ = 0;
    controller_state.position_ = 100000;
    while(std::abs(controller_state.position_) > 0.1)
    {
        controller.command_to_tx_frame(tx_frame, controller_command);
        pi3hat_output = pi3hat.Cycle(input);
        ::usleep(2000);
        controller.rx_frame_to_state(rx_frame, controller_state);
    }

    std::cout << "Controller successfully started!" << std::endl;

    auto prev = GetNow();
    int frequency;
    while(true)
    {   
        auto now = GetNow();
        controller_command.position_ = 5 * sin(now - prev);
        controller.command_to_tx_frame(tx_frame, controller_command);
        pi3hat_output = pi3hat.Cycle(input);
        ::usleep(1000);
        auto mesaure_time = GetNow() - now;
        frequency = (int) 1/mesaure_time;
        controller.rx_frame_to_state(rx_frame, controller_state);
        ::printf("f, pos_c, pos_s=(%d, %7.3f, %7.3f)\r",
        frequency, controller_command.position_, controller_state.position_);
        ::fflush(::stdout);
    }

    return 0;
}