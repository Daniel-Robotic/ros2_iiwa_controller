cmake_minimum_required(VERSION 3.8)
project(iiwa_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_definitions(-DHAVE_SOCKLEN_T
                -DPB_SYSTEM_HEADER="pb_syshdr.h"
                -DPB_FIELD_16BIT
                -DHAVE_STDINT_H
                -DHAVE_STDDEF_H
                -DHAVE_STDBOOL_H
                -DHAVE_STDLIB_H
                -DHAVE_STRING_H
                -fPIC)

set(FRI_HEADER
    external/libFRI/include
    external/libFRI/src/protobuf_gen
    external/libFRI/src/nanopb-0.2.8
    external/libFRI/src/protobuf
    external/libFRI/src/connection
    external/libFRI/src/client_lbr
    external/libFRI/src/base
  )

set(FRI_SRC
    external/libFRI/src/base/friClientApplication.cpp
    external/libFRI/src/client_lbr/friLBRClient.cpp
    external/libFRI/src/client_lbr/friLBRCommand.cpp
    external/libFRI/src/client_lbr/friLBRState.cpp
    external/libFRI/src/connection/friUdpConnection.cpp
    external/libFRI/src/protobuf/friCommandMessageEncoder.cpp
    external/libFRI/src/protobuf/friMonitoringMessageDecoder.cpp
    external/libFRI/src/protobuf/pb_frimessages_callbacks.c
    external/libFRI/src/protobuf_gen/FRIMessages.pb.c
    external/libFRI/src/nanopb-0.2.8/pb_decode.c
    external/libFRI/src/nanopb-0.2.8/pb_encode.c
    external/libFRI/src/client_trafo/friTransformationClient.cpp
  )

add_library(${PROJECT_NAME}
  SHARED
  src/IIWAHardwareInterface.cpp
  ${FRI_SRC}
  src/FRIClient.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
  include
  ${FRI_HEADER}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
)

pluginlib_export_plugin_description_file(hardware_interface iiwa_controller_plugin.xml)


install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
