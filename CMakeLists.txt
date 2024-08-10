cmake_minimum_required(VERSION 3.8)
project(pi3hat_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  hardware_interface
  rclcpp 
  rclcpp_lifecycle
  transmission_interface

)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## Hardware Interface
add_library(
  pi3hat_hardware_interface 
  SHARED
  src/pi3hat_hardware_interface.cpp
)
# add include directory
target_include_directories(pi3hat_hardware_interface
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_compile_features(
      pi3hat_hardware_interface PUBLIC cxx_std_17)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "PI3HAT_HARDWARE_INTERFACE_BUILDING_DLL")


## Pi3hat library by mjbots
add_library(
  pi3hat
  SHARED
  include/3rd_libs/pi3hat/pi3hat.cc
)

# add include directory
target_include_directories(pi3hat
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    
target_compile_features(
      pi3hat PUBLIC cxx_std_17)


## Custom controller bridge library
## Here add your wrappers!
add_library(controllers SHARED
  src/controllers/ControllerBridge.cpp
  src/controllers/wrappers/ControllerWrapper.cpp
  src/controllers/wrappers/MoteusWrapper.cpp
)

# add include directory
target_include_directories(controllers 
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    
target_compile_features(
  controllers  PUBLIC cxx_std_17)


## Custom library for IMU transformations
add_library(
  imu_transform 
  SHARED
  src/imu_transform/IMUTransform.cpp
)

# add include directory
target_include_directories(imu_transform 
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    
target_compile_features(
  imu_transform  PUBLIC cxx_std_17)


target_link_libraries(
  pi3hat_hardware_interface 
  PUBLIC pi3hat
  PUBLIC controllers
  PUBLIC imu_transform
  PUBLIC bcm_host
)

ament_target_dependencies(
  pi3hat_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


## Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface pi3hat_hardware_interface.xml)


## Install
install(TARGETS 
  ${PROJECT_NAME}
  pi3hat
  controllers
  imu_transform
  
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY test/urdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY test/bringup/launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY test/bringup/config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# exports
ament_export_include_directories(
  include
)
ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
) 

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
