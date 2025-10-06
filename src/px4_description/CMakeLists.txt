cmake_minimum_required(VERSION 3.8)
project(px4_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(
  PROGRAMS
    src/dynamic_joint_state.py
    src/move_drone.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
