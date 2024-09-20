# pi3hat_hardware_interface

## Introduction

This project provides a `ros2_control` `SystemInterface` for mjbots [pi3hat](https://mjbots.com/products/mjbots-pi3hat-r4-5).\
Huge thanks to [Gabrael Levine](https://github.com/G-Levine) and his version of [pi3hat_hardware_interface](https://github.com/G-Levine/pi3hat_hardware_interface/tree/main)
for inspiration and making this work much more easier!

### Software supports:
- :ballot_box_with_check: [Moteus](https://mjbots.com/products/moteus-r4-11) (for adding your controller check below)
- :ballot_box_with_check: [Transmission interface](http://docs.ros.org/en/jade/api/transmission_interface/html/c++/classtransmission__interface_1_1Transmission.html) (`SimpleTransmission`,
`FourBarLinkageTransmission` and `DifferentialTransmission`)
- :ballot_box_with_check: Simple transformation for IMU (`z` axis now points "up")
- :ballot_box_with_check: 3 joint command interfaces:
  - position [`radians]`
  - velocity [`radians/s`]
  - effort [`Nm`]
- :ballot_box_with_check: 4 joint state interfaces:
  -  position [`radians/s`]
  -  velocity [`radians/s`]
  -  effort [`Nm`]
  -  temperature [`Celcius`]
- :ballot_box_with_check: 10 IMU state interfaces (ready to use for [IMU Sensor Broadcaster](https://control.ros.org/master/doc/ros2_controllers/imu_sensor_broadcaster/doc/userdoc.html)):
  - orientation (`x`, `y`, `z` and `w`)
  - angular velocity (`x`, `y`, `z`) [`radians/s`]
  - linear acceleration (`x`, `y`, `z`) [`m/s^2`]

#### :warning: IMPORTANT: User don't have to set up every command or state interface in `xml` file, but remeber to set up your controller accordingly e.g. in order to control velocity in `moteus` controller, u need to set up `kp` coefficient to 0.0. 

## Hardware
- Raspberry Pi 4 (more RAM the better)
- pi3hat 
  
## Operating System
- [Ubuntu 22.04 with real-time kernel](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.3_v5.15.98-rt62-raspi_ros2_humble)

## Dependencies (all for humble)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [pi3hat](https://github.com/mjbots/pi3hat) (already in project)
- [moteus](https://github.com/mjbots/moteus) (already in project)

## Installation 
1. Clone repo to your workspace:
```bash
git clone https://github.com/BartlomiejK2/pi3hat_hardware_interface.git
```
2. Create link to library `bcm_host`:
```bash
sudo ln /usr/lib/aarch64-linux-gnu/libbcm_host.so /usr/lib/libbcm_host.so.0
```
3. Install dependencies:
```bash
rosdep install --ignore-src --from-paths . -y -r
```
5. Build (only works on Raspberry Pi):
```bash
colcon build --packages-select pi3hat_hardware_interface
```

## Hardware parameters

### Pi3hat options:
```xml
<ros2_control name="pi3hat_hardware_interface" type="system">
  <hardware>
    <plugin>pi3hat_hardware_interface/Pi3HatHardwareInterface</plugin>
    <param name="imu_mounting_deg.yaw">0</param>
    <param name="imu_mounting_deg.pitch">0</param>
    <param name="imu_mounting_deg.roll">0</param>
    <param name="imu_sampling_rate">1000</param>

    <param name="can_1_fdcan_frame">true</param>
    <param name="can_1_automatic_retransmission">true</param>
    <param name="can_1_bitrate_switch">true</param>

    <param name="can_2_fdcan_frame">true</param>
    <param name="can_2_automatic_retransmission">true</param>
    <param name="can_2_bitrate_switch">true</param>

    <param name="can_3_fdcan_frame">true</param>
    <param name="can_3_automatic_retransmission">true</param>
    <param name="can_3_bitrate_switch">true</param>

    <param name="can_4_fdcan_frame">true</param>
    <param name="can_4_automatic_retransmission">true</param>
    <param name="can_4_bitrate_switch">true</param>

    <param name="can_5_fdcan_frame">true</param>
    <param name="can_5_automatic_retransmission">true</param>
    <param name="can_5_bitrate_switch">true</param>
  </hardware>
  ...
</ros2_control>
```
`imu_mounting_deg.*` - IMU RPY mouting relative to fixed link [`degrees`]\
`imu_sampling_rate` - IMU rate for attitude sampling (400 or 1000 are the best) [`Hz`]\
`can_X_fdcan_frame` - Using FDCAN frame in X CAN bus [`bool`]\
`can_X_automatic_retransmission` - Using automatic retransmission in X CAN bus [`bool`]\
`can_X_bitrate_switch` - Using bitrate switch in X CAN bus [`bool`]

### Controller/Motor options:

```xml
...
<joint name="joint_1">
  <param name="controller_type">moteus</param>
  <param name="controller_can_bus">1</param>
  <param name="controller_can_id">1</param>


  <param name="motor_direction">1</param>
  <param name="motor_position_offset">0.0</param>
  <param name="motor_position_max">500.0</param>
  <param name="motor_position_min">-500.0</param>
  <param name="motor_velocity_max">10.0</param>
  <param name="motor_torque_max">1.0</param>


  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <command_interface name="effort"/>

  <state_interface name = "position"/>
  <state_interface name = "velocity"/>
  <state_interface name = "effort"/>
  <state_interface name = "temperature"/>
</joint>
...
```
#### Controller:
`controller_type` - Type of controller (now only moteus supported)\
`controller_can_bus` - CAN bus which controller is connected\
`controller_id` - Controller Id
#### :warning: IMPORTANT: All controller id's must be unique.

#### Motor (not joint):
`motor_direction` - Motor direction (1 or -1)\
`motor_position_offset` - Motor position offset, will be added to commanded position before sending to controller [`radians`]\
`motor_position_max/min` - Motor max/min position [`radians`]\
`motor_velocity_max` - Motor maximal velocity [`radians`]\
`motor_torque_max` - Motor maximal torque [`Nm`]

## Testing 

### Testing controller wrappers:
1. Go to `test/controllers`
2. Choose controller type directory
3. Compile tests:
```bash
bash "type"_wrapper_tests_compile.sh
```
4. Run tests:
```bash
cd exe/
sudo simple_"type"_test
sudo single_"type"_wrapper_test
sudo double_"type"_wrapper_test
```

### Testing hardware interface
1. Go to `test/urdf`
2. Choose urdfs with your controller type
3. Edit urdf with your options
4. Build project
5. Source project
6. Run:
```bash
  ros2 launch pi3hat_hardware_interface test_"amount"_"type".launch.py
```

## Troubleshooting
#### Check out Gabrael Levine [Troubleshooting](https://github.com/G-Levine/pi3hat_hardware_interface/tree/main?tab=readme-ov-file#troubleshooting). 

## Contributing

### Adding your controller
1. Check `include/controllers/wrappers/MoteusWrapper.hpp` and `src/controllers/wrappers/MoteusWrapper.cpp`.
2. Add third-party library to `include/3rd_libs/`.
3. Add third-party party library to `CMakelist.txt` as `add_library` (like `pi3hat` and `moteus`).
4. Create `"Type"Wrapper.hpp` file for your controller wrapper in `include/controllers/wrappers`.
5. Add include path to `"Type"Wrapper.hpp` file in `include/controllers/Controllers.hpp`.
6. Create `"Type"Wrapper.cpp` file with implementation in `src/controllers/wrappers`. 
7. Export your wrapper as `pluginlib` class.
8. Add your plugin in `controller_wrapppers.xml`.
9. Add comments to your code (check other source and header files).
10. Add wrapper tests for wrapper in `test/controllers/"type"` (check tests for `moteus`).
11. Add tests for pi3hat with your wrapper in `test/urdf` and `test/bringup/launch` (check tests for `moteus`).
12. Run all tests.
13. Create pull request with your code. Write description of your changes.

#### :warning: IMPORTANT: value for `controller_type` parameter must be `"Type"` (like in `"Type"Wrapper`). 

### Other changes
1. Change code.
2. Run all tests. 
1. If all passed, create pull request with your code. Write description of your changes.
