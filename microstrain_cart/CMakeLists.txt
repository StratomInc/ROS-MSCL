cmake_minimum_required(VERSION 3.5)
project(microstrain_cart)

find_package(ament_cmake REQUIRED)

# Install the launch and config files
install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()