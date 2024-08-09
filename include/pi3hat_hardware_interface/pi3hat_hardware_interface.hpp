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


#ifndef PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_
#define PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_


#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include<unistd.h> 

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "transmission_interface/transmission.hpp"
#include "transmission_interface/simple_transmission.hpp"
#include "transmission_interface/four_bar_linkage_transmission.hpp"
#include "transmission_interface/differential_transmission.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/four_bar_linkage_transmission_loader.hpp"
#include "transmission_interface/differential_transmission_loader.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

#include "controllers/Controllers.hpp"
#include "imu_transform/IMUTransform.hpp"
#include "3rd_libs/pi3hat/pi3hat.h"
#include "3rd_libs/pi3hat/realtime.h"

#include "visibility_control.hpp"

namespace pi3hat_hardware_interface
{
    class Pi3HatHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Pi3HatHardwareInterface)

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        
        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;


        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        ~Pi3HatHardwareInterface();
        

    private:
        
        /* UTILITY ROS2 OBJECTS: */
        std::unique_ptr<rclcpp::Logger> logger_;

        /* Number of controllers/joints */
        int joint_controller_number_;

        /* PART FOR COMMUNICATION WITH HARDWARE: */

        /* Pi3hat */
        std::shared_ptr<mjbots::pi3hat::Pi3Hat> pi3hat_;

        /* Pi3hat input structure */
        mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;

        /* IMU states */ 
        mjbots::pi3hat::Attitude attitude_;

        /* IMU transform */
        IMU::IMUTransform imu_transform_;

        /* TX CAN frames */
        std::vector<mjbots::pi3hat::CanFrame> tx_can_frames_;

        /* RX CAN frames */ 
        std::vector<mjbots::pi3hat::CanFrame> rx_can_frames_;

        /* Container for rx_frame_id (diffrent for diffrent controller type) to joint_index maping */
        std::unordered_map<int, int> controller_joint_map_;

        /* Controller states and commands */
        std::vector<controller_interface::ControllerState> controller_states_;
        std::vector<controller_interface::ControllerCommand> controller_commands_;

        /* For transmission interface */
        std::vector<controller_interface::ControllerCommand> controller_transmission_passthrough_;
         
        /* Controller Bridges */
        std::vector<controller_interface::ControllerBridge> controller_bridges_;


        using JointState = controller_interface::ControllerState;
        using JointCommand = controller_interface::ControllerCommand;

        /* Joint states and commands (for transmissions)*/
        std::vector<JointState> joint_states_;
        std::vector<JointCommand> joint_commands_;

        /* For transmission interface */
        std::vector<JointCommand> joint_transmission_passthrough_;


        /* FUNCTION FOR INITIALIZATION */
        controller_interface::ControllerParameters get_controller_parameters(const hardware_interface::ComponentInfo& joint_info);

        /* FUNCTION FOR CONTROLLERS */
        void controllers_init();

        void controllers_make_commands();

        void controllers_make_queries();

        void controllers_get_states();

        void create_controller_joint_map();

        /* FUNCTIONS FOR CREATING TRANSMISSION OBJECTS:*/

        /* Transmission interfaces*/
        std::vector<std::shared_ptr<transmission_interface::Transmission>> transmissions_;

        void joint_to_controller_transform();

        void controller_to_joint_transform();

        /* Function for creating all transmissions */
        void create_transmission_interface(const hardware_interface::HardwareInfo &info);

        /* Function for creating one transmission */
        void create_transmission(const hardware_interface::TransmissionInfo& transmission_info, std::string type, 
        transmission_interface::TransmissionLoader& loader, const std::vector<std::string>& joint_names);

        /* Functions for loading transmission data */
        void load_transmission_data(const hardware_interface::TransmissionInfo& transmission_info, 
            transmission_interface::TransmissionSharedPtr& transmission,
            transmission_interface::TransmissionLoader& loader);

        /* Functions for creating joint and actuator handels */
        void append_joint_handles(std::vector<transmission_interface::JointHandle>& joint_handles, 
         const std::string joint_name, const int joint_index);
        
        void append_actuator_handles(std::vector<transmission_interface::ActuatorHandle>& actuator_handles, 
         const std::string actuator_name, const int actuator_index);

        
        /* FUNCTIONS FOR INITIALIZING PI3HAT/CAN INTERFACE */
        


        /* UTILITY FUNCTIONS */
        bool string_to_bool(const std::string& str);
    }; 
};

#endif 
