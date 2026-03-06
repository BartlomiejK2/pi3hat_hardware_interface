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


#include "pi3hat_hardware_interface/pi3hat_hardware_interface.hpp"

using namespace pi3hat_hardware_interface;
using namespace controller_interface;

/* MAIN FUNCTIONS */

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    logger_ = std::make_unique<rclcpp::Logger>(
    rclcpp::get_logger("Pi3HatHardwareInterface"));

    joint_controller_number_ = info_.joints.size();

    controller_commands_.resize(joint_controller_number_);
    controller_states_.resize(joint_controller_number_);
    controller_transmission_passthrough_.resize(joint_controller_number_);


    joint_commands_.resize(joint_controller_number_);
    joint_states_.resize(joint_controller_number_);
    joint_transmission_passthrough_.resize(joint_controller_number_);
    
    controller_diagnostics_.resize(joint_controller_number_);
    additional_diagnostics_.resize(joint_controller_number_);

    /* Prepare controller bridges */
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        controller_interface::ControllerParameters params;
        std::string wrapper_type;
        try
        {
            params = get_controller_parameters(joint);
            wrapper_type = joint.parameters.at("controller_type");
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(*logger_, "Error reading motor/controller parameters!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        try
        {
            ControllerBridge controller_bridge(wrapper_type, params);
            controller_bridges_.push_back(std::move(controller_bridge));
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(*logger_, "Error creating motor controller!");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    /* Prepare transmissions */
    try
    {
        create_transmission_interface(info_);
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(*logger_, "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    /* Configure the IMU in Pi3hat */ 
    mjbots::pi3hat::Pi3Hat::Configuration config;
    
    /* Set the mounting orientation of the IMU */
    try
    {
        config.attitude_rate_hz = std::stoi(info_.hardware_parameters.at("imu_sampling_rate"));
        config.mounting_deg.yaw = std::stod(info_.hardware_parameters.at("imu_mounting_deg.yaw"));
        config.mounting_deg.pitch = std::stod(info_.hardware_parameters.at("imu_mounting_deg.pitch"));
        config.mounting_deg.roll = std::stod(info_.hardware_parameters.at("imu_mounting_deg.roll"));
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(*logger_, "Error reading IMU parameters!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    /* Initialize the Pi3Hat input */ 

    pi3hat_input_ = mjbots::pi3hat::Pi3Hat::Input();
    pi3hat_input_.request_attitude = true;
    pi3hat_input_.wait_for_attitude = string_to_bool(info_.hardware_parameters.at("wait_for_attitude"));
    pi3hat_input_.attitude = &attitude_;

    tx_can_frames_.resize(joint_controller_number_);
    rx_can_frames_.resize(joint_controller_number_);
    
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_can_frames_span_(&rx_can_frames_[0], joint_controller_number_); 
    pi3hat_input_.rx_can = rx_can_frames_span_;
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_can_frames_span_(&tx_can_frames_[0], joint_controller_number_); 
    pi3hat_input_.tx_can = tx_can_frames_span_;

    /* Configure each CAN bus */
    mjbots::pi3hat::Pi3Hat::CanConfiguration can_config;

    std::string fdcan_frame  = "fdcan_frame";
    std::string auto_retransmission = "automatic_retransmission";
    std::string bitrate_switch = "bitrate_switch";

    for(size_t i = 0; i < 5; ++i)
    {
        std::string can_channel = "can_" + std::to_string(i + 1) + "_";
        try
        {
            config.can[i].fdcan_frame = string_to_bool(info_.hardware_parameters.at(can_channel + fdcan_frame));
            config.can[i].automatic_retransmission = string_to_bool(info_.hardware_parameters.at(can_channel + auto_retransmission));
            config.can[i].bitrate_switch = string_to_bool(info_.hardware_parameters.at(can_channel + bitrate_switch));
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(*logger_, "Error reading CAN %d bus parameters!", i + 1);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    /* Initialize the Pi3Hat and realtime options */

    pi3hat_ =  std::make_shared<mjbots::pi3hat::Pi3Hat>(config);
    mjbots::pi3hat::ConfigureRealtime(0);
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{

    /* Initialize all motors/remove all flags and make query for state */

    using namespace std::literals::chrono_literals;

    const auto sleep_time = 10ms;

    controllers_init();
    auto result = pi3hat_->Cycle(pi3hat_input_);
    std::this_thread::sleep_for(sleep_time);

    if(result.error || result.rx_can_size <= 0)
    {
        RCLCPP_ERROR(*logger_, "Pi3Hat::Cycle() failed on \"on_configure()!\"");
        RCLCPP_ERROR(*logger_, "Error flag: %d, Amount of CAN frames:", 
            result.error, result.rx_can_size);
        return hardware_interface::CallbackReturn::ERROR;
    }

    /* Get all rx_frames ids (be sure there are no duplicates) */

    RCLCPP_INFO(*logger_, "Starting configure loop on \"on_configure()!\"");

    const auto configure_start_time = std::chrono::steady_clock::now();

    const auto max_configure_time = 10s;

    std::vector<uint32_t> rx_ids;
    do
    {
        controllers_make_queries();
        result = pi3hat_->Cycle(pi3hat_input_);

        if(result.error || result.rx_can_size <= 0)
        {
            RCLCPP_ERROR(*logger_, "Pi3Hat::Cycle() failed on \"on_configure()!\"");
            RCLCPP_ERROR(*logger_, "Error flag: %d, Amount of CAN frames:", 
            result.error, result.rx_can_size);
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::this_thread::sleep_for(sleep_time);
        for(size_t i = 0; i < result.rx_can_size; ++i)
        {
            if(std::find(rx_ids.begin(), rx_ids.end(), rx_can_frames_[i].id) == rx_ids.end())
            {
                RCLCPP_INFO(*logger_, "Configuration, new ID found: %d", 
                    rx_can_frames_[i].id);
                rx_ids.push_back(rx_can_frames_[i].id);
            }
        }

        if((std::chrono::steady_clock::now() - configure_start_time) > max_configure_time)
        {
            RCLCPP_ERROR(*logger_, "Configuration loop failed on timeout on \"on_configure()!\"");
            return hardware_interface::CallbackReturn::ERROR;
        }
    } 
    while(rx_ids.size() != joint_controller_number_);

    /* Create rx_frame.id -> joint map */
    try
    {
        create_controller_joint_map(rx_ids);
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(*logger_, "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{

    /* Lock motors in current place */
    RCLCPP_INFO(*logger_, "Locking motors in current position!");

    using namespace std::literals::chrono_literals;
    const auto sleep_time = 10ms;

    /* Get current position and set it for write() */

    const auto query_start_time = std::chrono::steady_clock::now();
    const auto max_query_time = 1s;

    while((std::chrono::steady_clock::now() - query_start_time) < max_query_time)
    {
        controllers_make_queries();

        const auto result = pi3hat_->Cycle(pi3hat_input_);

        if(result.error || result.rx_can_size <= 0)
        {
            RCLCPP_ERROR(*logger_, "Pi3Hat::Cycle() failed on \"on_activate()!\"");
            RCLCPP_ERROR(*logger_, "Error flag: %d, Amount of CAN frames:", 
                result.error, result.rx_can_size);
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(sleep_time);

        controllers_get_states(result.rx_can_size);
        if(result.rx_can_size == joint_controller_number_) break;
    }

    controller_to_joint_transform();

    reset_joint_data();
    
    RCLCPP_INFO(*logger_, "Locked motors in current position!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    
    /* Lock motors in current place */
    RCLCPP_INFO(*logger_, "Motors reaching starting position!");

    using namespace std::literals::chrono_literals;
    const auto sleep_time = 10ms;

    reset_joint_data();

    joint_to_controller_transform();

    controllers_make_commands();
    const auto result = pi3hat_->Cycle(pi3hat_input_);

    if(result.error || result.rx_can_size <= 0)
    {
        RCLCPP_ERROR(*logger_, "Pi3Hat::Cycle() failed on \"on_deactivate()!\"");
        RCLCPP_ERROR(*logger_, "Error flag: %d, Amount of CAN frames:", 
            result.error, result.rx_can_size);
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::this_thread::sleep_for(sleep_time);
    controllers_get_states(result.rx_can_size);

    controller_to_joint_transform();
    
    RCLCPP_INFO(*logger_, "Motors reached starting position!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{

    /* Deinitialize all motors/remove all flags */
    controllers_init();
    const auto result  = pi3hat_->Cycle(pi3hat_input_);

    if(result.error || result.rx_can_size <= 0)
    {
        RCLCPP_ERROR(*logger_, "Pi3Hat::Cycle() failed on \"on_cleanup()!\"");
        RCLCPP_ERROR(*logger_, "Error flag: %d, Amount of CAN frames:", 
            result.error, result.rx_can_size);
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> Pi3HatHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    /* Joint commands (before joint -> controller transformation)*/
    for (size_t i = 0; i < joint_controller_number_; i++)
    {   
        if(!(info_.joints[i].command_interfaces.size() > 0))
        {
            RCLCPP_WARN(*logger_, "Zero command interfaces for joint %s!", info_.joints[i].name.c_str());
        }
        for(const auto& command_interface: info_.joints[i].command_interfaces)
        {
            RCLCPP_INFO(*logger_, "%s joint has command interface: %s", info_.joints[i].name.c_str(), command_interface.name.c_str());
            if(command_interface.name == hardware_interface_names::POSITION)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface_names::POSITION, &(joint_commands_[i].position_)));
            }
            else if(command_interface.name == hardware_interface_names::VELOCITY)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface_names::VELOCITY, &(joint_commands_[i].velocity_)));
            }
            else if(command_interface.name == hardware_interface_names::EFFORT)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface_names::EFFORT, &(joint_commands_[i].torque_)));
            }
            else
            {
                RCLCPP_WARN(*logger_, "%s is wrong type of command interface, omitted!", command_interface.name.c_str());
            }
        }
    }

    return command_interfaces;
}

std::vector<hardware_interface::StateInterface> Pi3HatHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    /* Joint states (after controller -> joint transformation)*/
    for (size_t i = 0; i < joint_controller_number_; i++)
    {
        if(!(info_.joints[i].state_interfaces.size() > 0))
        {
            RCLCPP_WARN(*logger_, "Zero state interfaces for joint %s!", info_.joints[i].name.c_str());
        }
        for(const auto& state_interface: info_.joints[i].state_interfaces)
        {
            RCLCPP_INFO(*logger_, "%s joint has state interface: %s", info_.joints[i].name.c_str(), state_interface.name.c_str());
            if(state_interface.name == hardware_interface_names::POSITION)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::POSITION, &(joint_states_[i].position_)));
            }
            else if(state_interface.name == hardware_interface_names::VELOCITY)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::VELOCITY, &(joint_states_[i].velocity_)));
            }
            else if(state_interface.name == hardware_interface_names::EFFORT)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::EFFORT, &(joint_states_[i].torque_)));
            }

            if(state_interface.name == hardware_interface_names::MOTOR_POSITION)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::MOTOR_POSITION, &(controller_states_[i].position_)));
            }
            else if(state_interface.name == hardware_interface_names::MOTOR_VELOCITY)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::MOTOR_VELOCITY, &(controller_states_[i].velocity_)));
            }
            else if(state_interface.name == hardware_interface_names::MOTOR_EFFORT)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::MOTOR_EFFORT, &(controller_states_[i].torque_)));
            }

            else if(state_interface.name == hardware_interface_names::TEMPERATURE)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::TEMPERATURE, &(controller_diagnostics_[i].temperature_)));
            }
            else if(state_interface.name == hardware_interface_names::VOLTAGE)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::VOLTAGE, &(controller_diagnostics_[i].voltage_)));
            }
            else if(state_interface.name == hardware_interface_names::POWER)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::POWER, &(controller_diagnostics_[i].power_)));
            }
            else if(state_interface.name == hardware_interface_names::CURRENT)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::CURRENT, &(controller_diagnostics_[i].current_)));
            }
            else if(state_interface.name == hardware_interface_names::FAULT)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::FAULT, &(controller_diagnostics_[i].fault_)));
            }

            else if(state_interface.name == hardware_interface_names::POSITION_ERROR)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::POSITION_ERROR, &(additional_diagnostics_[i].position_error_)));
            }
            else if(state_interface.name == hardware_interface_names::VELOCITY_ERROR)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::VELOCITY_ERROR, &(additional_diagnostics_[i].velocity_error_)));
            }
            else if(state_interface.name == hardware_interface_names::DESIRED_POSITION)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::DESIRED_POSITION, &(additional_diagnostics_[i].desired_position_)));
            }
            else if(state_interface.name == hardware_interface_names::DESIRED_VELOCITY)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::DESIRED_VELOCITY, &(additional_diagnostics_[i].desired_velocity_)));
            }
            else if(state_interface.name == hardware_interface_names::DESIRED_EFFORT)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::DESIRED_EFFORT, &(additional_diagnostics_[i].desired_effort_)));
            }

            else if(state_interface.name == hardware_interface_names::MOTOR_POSITION_ERROR)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::MOTOR_POSITION_ERROR, &(additional_diagnostics_[i].motor_position_error_)));
            }
            else if(state_interface.name == hardware_interface_names::MOTOR_VELOCITY_ERROR)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::MOTOR_VELOCITY_ERROR, &(additional_diagnostics_[i].motor_velocity_error_)));
            }
            else if(state_interface.name == hardware_interface_names::MOTOR_DESIRED_POSITION)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::MOTOR_DESIRED_POSITION, &(additional_diagnostics_[i].motor_desired_position_)));
            }
            else if(state_interface.name == hardware_interface_names::MOTOR_DESIRED_VELOCITY)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::MOTOR_DESIRED_VELOCITY, &(additional_diagnostics_[i].motor_desired_velocity_)));
            }
            else if(state_interface.name == hardware_interface_names::MOTOR_DESIRED_EFFORT)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface_names::MOTOR_DESIRED_EFFORT, &(additional_diagnostics_[i].motor_desired_effort_)));
            }

            else
            {
                RCLCPP_WARN(*logger_, "%s is wrong type of state interface, omitted!", state_interface.name.c_str());
            }
        }
    }

    /* IMU states (after IMUTransform transformation) */
    if(info_.sensors.size() == 0)
    {
        RCLCPP_WARN(*logger_, "IMU: state interface was not configured!");
        return state_interfaces;
    }

    if(info_.sensors[0].state_interfaces.size() != 10)
    {
        RCLCPP_WARN(*logger_, "IMU: some states were not configured!");
    }
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.x", &attitude_.attitude.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.y", &attitude_.attitude.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.z", &attitude_.attitude.z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.w", &attitude_.attitude.w));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "angular_velocity.x", &attitude_.rate_dps.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "angular_velocity.y", &attitude_.rate_dps.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "angular_velocity.z", &attitude_.rate_dps.z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "linear_acceleration.x", &attitude_.accel_mps2.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "linear_acceleration.y", &attitude_.accel_mps2.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "linear_acceleration.z", &attitude_.accel_mps2.z));

    return state_interfaces;
}


hardware_interface::return_type Pi3HatHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    for (size_t i = 0; i < joint_controller_number_; ++i)
    {
        if (std::isnan(joint_commands_[i].position_) || std::isnan(joint_commands_[i].velocity_) || std::isnan(joint_commands_[i].torque_))
        {
            RCLCPP_WARN(*logger_, "NaN command for actuator");
            break;
        }
    }

    joint_to_controller_transform();

    controllers_make_commands();
    
    mjbots::pi3hat::Pi3Hat::Output result = pi3hat_->Cycle(pi3hat_input_);

    if(result.error)
    {
        RCLCPP_ERROR(*logger_, "Pi3Hat::Cycle() failed on \"write()\"!");
        RCLCPP_ERROR(*logger_, "Error flag: %d", result.error);
        return hardware_interface::return_type::ERROR;
    }

    if (result.attitude_present)
    {
        imu_transform_.transform_attitude(attitude_);
    }

    if(result.rx_can_size > 0)
    {
        controllers_get_states(result.rx_can_size);
    }

    controller_to_joint_transform();

    fill_diagnostics();
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pi3HatHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    return hardware_interface::return_type::OK;
}

void Pi3HatHardwareInterface::joint_to_controller_transform()
{
    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        joint_transmission_passthrough_[i].position_ = joint_commands_[i].position_;
        joint_transmission_passthrough_[i].velocity_ = joint_commands_[i].velocity_;
        joint_transmission_passthrough_[i].torque_ = joint_commands_[i].torque_;
    }

    std::for_each(transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->joint_to_actuator(); });

    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        controller_commands_[i].position_ = controller_transmission_passthrough_[i].position_;
        controller_commands_[i].velocity_ = controller_transmission_passthrough_[i].velocity_;
        controller_commands_[i].torque_ = controller_transmission_passthrough_[i].torque_;
    }
}

void Pi3HatHardwareInterface::controller_to_joint_transform()
{
    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        controller_transmission_passthrough_[i].position_ = controller_states_[i].position_;
        controller_transmission_passthrough_[i].velocity_ = controller_states_[i].velocity_;
        controller_transmission_passthrough_[i].torque_ = controller_states_[i].torque_;
    }

    std::for_each(transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->actuator_to_joint(); });

    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        joint_states_[i].position_ = joint_transmission_passthrough_[i].position_;
        joint_states_[i].velocity_ = joint_transmission_passthrough_[i].velocity_;
        joint_states_[i].torque_ = joint_transmission_passthrough_[i].torque_;
    }
}

/* TRANSMISSION FUNCTIONS */
void Pi3HatHardwareInterface::append_joint_handles(std::vector<transmission_interface::JointHandle>& joint_handles, const std::string joint_name, const int joint_index)
{
    transmission_interface::JointHandle joint_handle_position(joint_name, hardware_interface_names::POSITION, 
     &joint_transmission_passthrough_[joint_index].position_);
    joint_handles.push_back(joint_handle_position);

    transmission_interface::JointHandle joint_handle_velocity(joint_name, hardware_interface_names::VELOCITY, 
     &joint_transmission_passthrough_[joint_index].velocity_);
    joint_handles.push_back(joint_handle_velocity);

    transmission_interface::JointHandle joint_handle_torque(joint_name, hardware_interface_names::EFFORT,
     &joint_transmission_passthrough_[joint_index].torque_);
    joint_handles.push_back(joint_handle_torque);
}

void Pi3HatHardwareInterface::append_actuator_handles(std::vector<transmission_interface::ActuatorHandle>& actuator_handles, const std::string actuator_name, const int actuator_index)
{
    transmission_interface::ActuatorHandle actuator_handle_position(actuator_name, hardware_interface_names::POSITION,
     &controller_transmission_passthrough_[actuator_index].position_);
    actuator_handles.push_back(actuator_handle_position);

    transmission_interface::ActuatorHandle actuator_handle_velocity(actuator_name, hardware_interface_names::VELOCITY, 
     &controller_transmission_passthrough_[actuator_index].velocity_);
    actuator_handles.push_back(actuator_handle_velocity);

    transmission_interface::ActuatorHandle actuator_handle_torque(actuator_name, hardware_interface_names::EFFORT,
     &controller_transmission_passthrough_[actuator_index].torque_);
    actuator_handles.push_back(actuator_handle_torque);
}


void Pi3HatHardwareInterface::load_transmission_data(const hardware_interface::TransmissionInfo& transmission_info, 
    transmission_interface::TransmissionSharedPtr& transmission, transmission_interface::TransmissionLoader& loader)
{
    try
    {
        transmission = loader.load(transmission_info);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
        RCLCPP_FATAL(*logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
        return;
    }
}

void Pi3HatHardwareInterface::create_transmission_interface(const hardware_interface::HardwareInfo &info)
{
    if(info.transmissions.size() == 0) return;
    
    /* Prepare loaders */
    transmission_interface::SimpleTransmissionLoader simple_loader;
    transmission_interface::FourBarLinkageTransmissionLoader fbl_loader;
    transmission_interface::DifferentialTransmissionLoader diff_loader;

    /* Prepare joint names */
    std::vector<std::string> joint_names;
    for(const auto& joint: info.joints)
    {
        joint_names.push_back(joint.name);
    }

    /* For fast computation, transmissions will be sorted by type */

    /* Simple transmissions */
    for(const auto& transmission_info: info.transmissions)
    {
        if(transmission_info.type == "transmission_interface/SimpleTransmission")
        {
            try
            {
                create_transmission(transmission_info, "SimpleTransmission", simple_loader, joint_names);
            }
            catch(const transmission_interface::TransmissionInterfaceException& e)
            {
                throw;
            }
            
        }
    }

    /* FourBarLinkage transmissions */
    for(const auto& transmission_info: info.transmissions)
    {
        if(transmission_info.type == "transmission_interface/FourBarLinkageTransmission")
        {
            try
            {
                create_transmission(transmission_info, "FourBarLinkageTransmission", fbl_loader, joint_names);
            }
            catch(const transmission_interface::TransmissionInterfaceException& e)
            {
                throw;
            }
            
        }
    }

    /* Differential transmissions */
    for(const auto& transmission_info: info.transmissions)
    {
        if(transmission_info.type == "transmission_interface/DifferentialTransmission")
        {
            try
            {
                create_transmission(transmission_info, "DifferentialTransmission", diff_loader, joint_names);
            }
            catch(const transmission_interface::TransmissionInterfaceException& e)
            {
                throw;
            }
        }
    }
}

 void Pi3HatHardwareInterface::create_transmission(const hardware_interface::TransmissionInfo& transmission_info, std::string type, 
        transmission_interface::TransmissionLoader& loader, const std::vector<std::string>& joint_names)
{
    RCLCPP_INFO(*logger_, "%s initialization starting!", type.c_str());

    std::shared_ptr<transmission_interface::Transmission> transmission;

    if (transmission_info.type != ("transmission_interface/" + type))
    {
        RCLCPP_FATAL(*logger_, "This is not %s!", type.c_str());
        throw transmission_interface::TransmissionInterfaceException("This is not " + type + "!"); // this should not happen!
    }
    load_transmission_data(transmission_info, transmission, loader);

    int joint_for_transmission = 0;

    if(type == "SimpleTransmission")
    {
        joint_for_transmission = 1;
    }
    else if(type == "FourBarLinkageTransmission" || type == "DifferentialTransmission")
    {
        joint_for_transmission = 2;
    }

    if(transmission_info.joints.size() != joint_for_transmission)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of joints in %s!", type.c_str());
        throw transmission_interface::TransmissionInterfaceException("Invalid number of joints in " + type + "!"); // this should not happen!
    }

    if(transmission_info.actuators.size() != joint_for_transmission)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of actuators in %s!", type.c_str());
        throw transmission_interface::TransmissionInterfaceException("Invalid number of actuators in " + type + "!"); // this should not happen!
    }

    std::vector<transmission_interface::JointHandle> joint_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;

    for(size_t i = 0; i < joint_for_transmission; ++i)
    {
        std::vector<std::string>::const_iterator joint_it = std::find(joint_names.begin(), 
          joint_names.end(), transmission_info.joints[i].name);

        int joint_index = std::distance(joint_names.begin(), joint_it);

        append_joint_handles(joint_handles, transmission_info.joints[i].name, joint_index);
        append_actuator_handles(actuator_handles, transmission_info.actuators[i].name, joint_index);
    }

    try
    {
        transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
        RCLCPP_FATAL(*logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
        throw;
    }

    transmissions_.push_back(transmission);
    
    RCLCPP_INFO(*logger_, "%s initialized!", type.c_str());


}

ControllerParameters Pi3HatHardwareInterface::get_controller_parameters(const hardware_interface::ComponentInfo& joint_info)
{
    ControllerParameters params;
    try
    {
        params.bus_ = std::stoi(joint_info.parameters.at("controller_can_bus"));
        params.id_ = std::stoi(joint_info.parameters.at("controller_can_id"));
        params.direction_ = std::stoi(joint_info.parameters.at("motor_direction"));
        params.position_offset_ = std::stod(joint_info.parameters.at("motor_position_offset"));
        params.position_max_ = std::stod(joint_info.parameters.at("motor_position_max"));
        params.position_min_ = std::stod(joint_info.parameters.at("motor_position_min"));
        params.velocity_max_ = std::stod(joint_info.parameters.at("motor_velocity_max"));
        params.torque_max_ = std::stod(joint_info.parameters.at("motor_torque_max"));
    }
    catch(const std::exception& e)
    {
        throw;
    }

    return params;
}

void Pi3HatHardwareInterface::controllers_init()
{
    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges_[i].initialize(tx_can_frames_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_make_commands()
{
    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges_[i].make_command(tx_can_frames_[i], controller_commands_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_make_queries()
{
    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges_[i].make_query(tx_can_frames_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_get_states(size_t current_can_size)
{
    for(size_t i = 0; i < current_can_size; ++i)
    {
        int joint_id = controller_joint_map_.at(rx_can_frames_[i].id);
        controller_bridges_[joint_id].get_state(rx_can_frames_[i], controller_states_[joint_id], controller_diagnostics_[joint_id]);
    }
}

void Pi3HatHardwareInterface::create_controller_joint_map(std::vector<uint32_t>& can_ids)
{
    if(can_ids.size() != joint_controller_number_)
    {
        RCLCPP_ERROR(*logger_, "Can IDs vector have length %d, should have %d!", 
            can_ids.size(), joint_controller_number_);
        throw std::logic_error("Failed while creating joint -> controller map!");    
    }
    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        int joint_id = i;
        const std::string joint_name = info_.joints[i].name;
        int controller_id = controller_bridges_[i].get_params().id_;
        for(size_t j = 0; j < joint_controller_number_; ++j)
        {
            int id_from_rx_frame = controller_bridges_[i].get_id(rx_can_frames_[j]);
            if(controller_id == id_from_rx_frame)
            {
                RCLCPP_INFO(*logger_, "Joint name: %s, Joint ID: %d, Controller ID: %d, "
                    "Frame ID: %d, Frame bus: %d", joint_name.c_str(), joint_id, 
                    controller_id, rx_can_frames_[j].id, rx_can_frames_[j].bus);
                std::pair<int, int> controller_joint_pair(rx_can_frames_[j].id, joint_id);
                controller_joint_map_.emplace(controller_joint_pair);
                break;
            }
        }
    }
    if(controller_joint_map_.size() != joint_controller_number_)
    {
        RCLCPP_ERROR(*logger_, "Map vector have length %d, should have %d!", 
            controller_joint_map_.size(), joint_controller_number_);
        throw std::logic_error("Failed while creating joint -> controller map!"); 
    }
}

Pi3HatHardwareInterface::~Pi3HatHardwareInterface()
{
    //on_deactivate(rclcpp_lifecycle::State()); // motors dont reach starting position, better to just turn them off
    on_cleanup(rclcpp_lifecycle::State());
}


bool Pi3HatHardwareInterface::string_to_bool(const std::string& str)
{
    if(str == "true")
    {
        return true;
    }
    else if(str == "false")
    {
        return false;
    }
    else
    {
        throw std::invalid_argument("Wrong string value for boolean");
    }
}

void Pi3HatHardwareInterface::reset_joint_data()
{
    /* Set all commands, states and transmission passthrough to end state */

    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        controller_commands_[i].position_ = controller_states_[i].position_; // start and end with current position
        controller_commands_[i].velocity_ = 0.0;
        controller_commands_[i].torque_ = 0.0;

        controller_transmission_passthrough_[i].position_ = 0.0;
        controller_transmission_passthrough_[i].velocity_ = 0.0;
        controller_transmission_passthrough_[i].torque_ = 0.0;


        joint_commands_[i].position_ = joint_states_[i].position_; // start and end with current position
        joint_commands_[i].velocity_ = 0.0;
        joint_commands_[i].torque_ = 0.0;

        joint_transmission_passthrough_[i].position_ = 0.0;
        joint_transmission_passthrough_[i].velocity_ = 0.0;
        joint_transmission_passthrough_[i].torque_ = 0.0;

        additional_diagnostics_[i].position_error_ = 0.0; 
        additional_diagnostics_[i].velocity_error_ = 0.0;
        additional_diagnostics_[i].desired_position_ = joint_states_[i].position_;
        additional_diagnostics_[i].desired_velocity_ = 0.0;
        additional_diagnostics_[i].desired_effort_ = 0.0;
        additional_diagnostics_[i].motor_position_error_ = 0.0;
        additional_diagnostics_[i].motor_velocity_error_ = 0.0;
        additional_diagnostics_[i].motor_desired_position_ = controller_states_[i].position_;
        additional_diagnostics_[i].motor_desired_velocity_ = 0.0;
        additional_diagnostics_[i].motor_desired_effort_ = 0.0;
    }
}

void Pi3HatHardwareInterface::fill_diagnostics()
{
    for(size_t i = 0; i < joint_controller_number_; ++i)
    {
        additional_diagnostics_[i].position_error_ = joint_commands_[i].position_ - joint_states_[i].position_; 
        additional_diagnostics_[i].velocity_error_ = joint_commands_[i].velocity_ - joint_states_[i].velocity_; 
        additional_diagnostics_[i].desired_position_ = joint_commands_[i].position_;
        additional_diagnostics_[i].desired_velocity_ = joint_commands_[i].velocity_;
        additional_diagnostics_[i].desired_effort_ = joint_commands_[i].torque_;
        additional_diagnostics_[i].motor_position_error_ = controller_commands_[i].position_ - controller_states_[i].position_; 
        additional_diagnostics_[i].motor_velocity_error_ = controller_commands_[i].velocity_ - controller_states_[i].velocity_; 
        additional_diagnostics_[i].motor_desired_position_ = controller_commands_[i].position_;
        additional_diagnostics_[i].motor_desired_velocity_ = controller_commands_[i].velocity_;
        additional_diagnostics_[i].motor_desired_effort_ = controller_commands_[i].torque_;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pi3hat_hardware_interface::Pi3HatHardwareInterface,
  hardware_interface::SystemInterface)