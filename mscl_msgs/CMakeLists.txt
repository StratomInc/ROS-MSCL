cmake_minimum_required(VERSION 3.5)
project(mscl_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

FILE(GLOB rosMsgFiles RELATIVE "${${PROJECT_NAME}_SOURCE_DIR}"
    "${${PROJECT_NAME}_SOURCE_DIR}/msg/*.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${rosMsgFiles}
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


