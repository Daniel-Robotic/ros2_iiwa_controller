cmake_minimum_required(VERSION 3.8)
project(iiwa_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

install(
  DIRECTORY urdf meshes config python
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
