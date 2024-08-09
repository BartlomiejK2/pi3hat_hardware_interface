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
    pi3hat_input_.wait_for_attitude = true;
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

    for(int i = 0; i < 5; ++i)
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

    controllers_init();
    auto result = pi3hat_->Cycle(pi3hat_input_);
    ::usleep(1000000);

    /* Get all rx_frames ids (be sure there are no duplicates) */

    std::vector<uint32_t> rx_ids;
    rx_ids.resize(joint_controller_number_);
    do
    {
        controllers_make_queries();
        result = pi3hat_->Cycle(pi3hat_input_);
        ::usleep(100000);
        for(int i = 0; i < joint_controller_number_; ++i)
        {
            rx_ids[i] = rx_can_frames_[i].id;
        }
    } 
    while(std::adjacent_find(rx_ids.begin(), rx_ids.end()) != rx_ids.end());

    /* Create rx_frame.id -> joint map */
    try
    {
        create_controller_joint_map();
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(*logger_, "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    /* Get states with prepared controller -> joint map */
    controllers_get_states();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{

    /* Make start from actual motor position to 0.0 (offset included in controller bridges) */
    RCLCPP_INFO(*logger_, "Motors reaching starting position!");

    controllers_make_commands();
    pi3hat_->Cycle(pi3hat_input_);
    ::usleep(1000000);
    controllers_get_states();
    
    RCLCPP_INFO(*logger_, "Motors reached starting position!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    /* Set all commands, states and transmission passthrough to end state */
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_commands_[i].position_ = 0;
        controller_commands_[i].velocity_ = 0;
        controller_commands_[i].torque_ = 0;

        controller_transmission_passthrough_[i].position_ = 0;
        controller_transmission_passthrough_[i].velocity_ = 0;
        controller_transmission_passthrough_[i].torque_ = 0;


        joint_commands_[i].position_ = 0;
        joint_commands_[i].velocity_ = 0;
        joint_commands_[i].torque_ = 0;

        joint_transmission_passthrough_[i].position_ = 0;
        joint_transmission_passthrough_[i].velocity_ = 0;
        joint_transmission_passthrough_[i].torque_ = 0;
    }

    
    /* Make slow start from actual motor position to 0.0 for 10 seconds 
        (offset included in controller bridges) */
    RCLCPP_INFO(*logger_, "Motors reaching starting position!");

    controllers_make_commands();
    pi3hat_->Cycle(pi3hat_input_);
    ::usleep(1000000);
    controllers_get_states();
    
    RCLCPP_INFO(*logger_, "Motors reached starting position!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{

    /* Deinitialize all motors/remove all flags */
    controllers_init();
    pi3hat_->Cycle(pi3hat_input_);
    ::usleep(1000000);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> Pi3HatHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    /* Joint commands (before joint -> controller transformation)*/
    for (int i = 0; i < joint_controller_number_; i++)
    {   
        if(!(info_.joints[i].command_interfaces.size() > 0))
        {
            RCLCPP_WARN(*logger_, "Zero command interfaces for joint %s!", info_.joints[i].name.c_str());
        }
        for(const auto& command_interface: info_.joints[i].command_interfaces)
        {
            if(command_interface.name == hardware_interface::HW_IF_POSITION)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &(joint_commands_[i].position_)));
            }
            else if(command_interface.name == hardware_interface::HW_IF_VELOCITY)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &(joint_commands_[i].velocity_)));
            }
            else if(command_interface.name == hardware_interface::HW_IF_EFFORT)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &(joint_commands_[i].torque_)));
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
    for (int i = 0; i < joint_controller_number_; i++)
    {
        if(!(info_.joints[i].state_interfaces.size() > 0))
        {
            RCLCPP_WARN(*logger_, "Zero state interfaces for joint %s!", info_.joints[i].name.c_str());
        }
        for(const auto& state_interface: info_.joints[i].state_interfaces)
        {
            if(state_interface.name == hardware_interface::HW_IF_POSITION)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &(joint_states_[i].position_)));
            }
            else if(state_interface.name == hardware_interface::HW_IF_VELOCITY)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &(joint_states_[i].velocity_)));
            }
            else if(state_interface.name == hardware_interface::HW_IF_EFFORT)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &(joint_states_[i].torque_)));
            }
            else if(state_interface.name == "temperature")
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, "temperature", &(joint_states_[i].temperature_)));
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
    for (int i = 0; i < joint_controller_number_; ++i)
    {
        if (std::isnan(joint_commands_[i].position_) || std::isnan(joint_commands_[i].velocity_) || std::isnan(joint_commands_[i].torque_))
        {
            RCLCPP_WARN(rclcpp::get_logger("Pi3HatHardwareInterface"), "NaN command for actuator");
            break;
        }
    }

    joint_to_controller_transform();

    controllers_make_commands();
    
    mjbots::pi3hat::Pi3Hat::Output result = pi3hat_->Cycle(pi3hat_input_);

    if (result.error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Pi3Hat::Cycle() failed!");
        return hardware_interface::return_type::ERROR;
    }

    if (result.attitude_present)
    {
        imu_transform_.transform_attitude(attitude_);
    }

    if(result.rx_can_size > 0)
    {
        controllers_get_states();
    }

    controller_to_joint_transform();
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pi3HatHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    return hardware_interface::return_type::OK;
}

void Pi3HatHardwareInterface::joint_to_controller_transform()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        joint_transmission_passthrough_[i].position_ = joint_commands_[i].position_;
        joint_transmission_passthrough_[i].velocity_ = joint_commands_[i].velocity_;
        joint_transmission_passthrough_[i].torque_ = joint_commands_[i].torque_;
    }

    std::for_each(
    transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->joint_to_actuator(); });

    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_commands_[i].position_ = controller_transmission_passthrough_[i].position_;
        controller_commands_[i].velocity_ = controller_transmission_passthrough_[i].velocity_;
        controller_commands_[i].torque_ = controller_transmission_passthrough_[i].torque_;
    }
}

void Pi3HatHardwareInterface::controller_to_joint_transform()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_transmission_passthrough_[i].position_ = controller_states_[i].position_;
        controller_transmission_passthrough_[i].velocity_ = controller_states_[i].velocity_;
        controller_transmission_passthrough_[i].torque_ = controller_states_[i].torque_;
    }

    std::for_each(
    transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->actuator_to_joint(); });

    for(int i = 0; i < joint_controller_number_; ++i)
    {
        joint_states_[i].position_ = joint_transmission_passthrough_[i].position_;
        joint_states_[i].velocity_ = joint_transmission_passthrough_[i].velocity_;
        joint_states_[i].torque_ = joint_transmission_passthrough_[i].torque_;
    }
}

/* TRANSMISSION FUNCTIONS */
void Pi3HatHardwareInterface::append_joint_handles(std::vector<transmission_interface::JointHandle>& joint_handles, const std::string joint_name, const int joint_index)
{
    transmission_interface::JointHandle joint_handle_position(joint_name, hardware_interface::HW_IF_POSITION, 
     &joint_transmission_passthrough_[joint_index].position_);
    joint_handles.push_back(joint_handle_position);

    transmission_interface::JointHandle joint_handle_velocity(joint_name, hardware_interface::HW_IF_VELOCITY, 
     &joint_transmission_passthrough_[joint_index].velocity_);
    joint_handles.push_back(joint_handle_velocity);

    transmission_interface::JointHandle joint_handle_torque(joint_name, hardware_interface::HW_IF_EFFORT,
     &joint_transmission_passthrough_[joint_index].torque_);
    joint_handles.push_back(joint_handle_torque);
}

void Pi3HatHardwareInterface::append_actuator_handles(std::vector<transmission_interface::ActuatorHandle>& actuator_handles, const std::string actuator_name, const int actuator_index)
{
    transmission_interface::ActuatorHandle actuator_handle_position(actuator_name, hardware_interface::HW_IF_POSITION,
     &controller_transmission_passthrough_[actuator_index].position_);
    actuator_handles.push_back(actuator_handle_position);

    transmission_interface::ActuatorHandle actuator_handle_velocity(actuator_name, hardware_interface::HW_IF_VELOCITY, 
     &controller_transmission_passthrough_[actuator_index].velocity_);
    actuator_handles.push_back(actuator_handle_velocity);

    transmission_interface::ActuatorHandle actuator_handle_torque(actuator_name, hardware_interface::HW_IF_EFFORT,
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
        throw transmission_interface::TransmissionInterfaceException("Invalid number of joints in SimpleTransmission!"); // this should not happen!
    }

    if(transmission_info.actuators.size() != joint_for_transmission)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of actuators in %s!", type.c_str());
        throw transmission_interface::TransmissionInterfaceException("Invalid number of actuators in SimpleTransmission!"); // this should not happen!
    }

    std::vector<transmission_interface::JointHandle> joint_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;

    for(int i = 0; i < joint_for_transmission; ++i)
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
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges_[i].initialize(tx_can_frames_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_make_commands()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges_[i].make_command(tx_can_frames_[i], controller_commands_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_make_queries()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges_[i].make_query(tx_can_frames_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_get_states()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        int joint_id = controller_joint_map_.at(rx_can_frames_[i].id);
        controller_bridges_[joint_id].get_state(rx_can_frames_[i], controller_states_[joint_id]);
    }
}

void Pi3HatHardwareInterface::create_controller_joint_map()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        int joint_id = i;
        int controller_id = controller_bridges_[i].get_params().id_;
        for(int j = 0; j < joint_controller_number_; ++j)
        {
            int id_from_rx_frame = controller_bridges_[i].get_id(rx_can_frames_[j]);
            RCLCPP_INFO(*logger_, "Joint: %d, Controller: %d, Frame id: %d, Frame bus: %d", joint_id, controller_id, rx_can_frames_[j].id, rx_can_frames_[j].bus);
            if(controller_id == id_from_rx_frame)
            {
                std::pair<int, int> controller_joint_pair(rx_can_frames_[j].id, joint_id);
                controller_joint_map_.emplace(controller_joint_pair);
            }
        }
    }
    if(controller_joint_map_.size() != joint_controller_number_)
    {
        throw std::logic_error("Controller -> Joint map has diffrent length!");
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



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pi3hat_hardware_interface::Pi3HatHardwareInterface,
  hardware_interface::SystemInterface)