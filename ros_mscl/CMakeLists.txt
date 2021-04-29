cmake_minimum_required(VERSION 3.5)
project(ros_mscl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev -fconcepts)
endif()

set(CMAKE_C_FLAGS "-Wno-implicit-function-declaration -Wno-incompatible-pointer-types -Wno-format -fno-builtin-memcpy")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(mscl_msgs REQUIRED)

find_library(MSCL_LIB_PATH NAMES libmscl.so PATHS "/usr/share/c++-mscl" DOC "MSCL Library" NO_DEFAULT_PATH)
find_library(BOOST_LIB_PATH NAMES libboost_chrono.so PATHS "/usr/share/c++-mscl/Boost/lib" NO_DEFAULT_PATH)
set(MSCL_INC_PATH "/usr/share/c++-mscl/source")
set(BOOST_INC_PATH "/usr/share/c++-mscl/Boost/include")
#find_path(MSCL_INC_PATH NAMES mscl.h PATHS "/usr/share/c++-mscl/source/mscl" DOC "MSCL Includes" NO_DEFAULT_PATH)

set(MSCL_LIB "usr/share/c++-mscl")


FILE(GLOB SRV_FILES RELATIVE "${${PROJECT_NAME}_SOURCE_DIR}"
    "${${PROJECT_NAME}_SOURCE_DIR}/srv/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
)

include_directories(include/${PROJECT_NAME} ${MIPLIBINC} ${MIPUTILINC} ${MSCL_INC_PATH} ${MSCL_LIB} ${MSCL_INC} )

add_library(microstrain_3dm SHARED
  src/microstrain_3dm.cpp
)

target_link_libraries(microstrain_3dm   ${MSCL_LIB_PATH})

ament_target_dependencies(microstrain_3dm
  rclcpp
  mscl_msgs
  tf2_geometry_msgs
  sensor_msgs  
)

target_include_directories(microstrain_3dm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

rosidl_target_interfaces(microstrain_3dm
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(microstrain_diagnostic_updater SHARED
  src/microstrain_diagnostic_updater.cpp
)
ament_target_dependencies(microstrain_diagnostic_updater
  rclcpp
  mscl_msgs
  diagnostic_updater
)
add_executable(set_filter_euler_client src/set_filter_euler_client.cpp)

add_dependencies(set_filter_euler_client ros_mscl_gencpp)


rosidl_target_interfaces(microstrain_diagnostic_updater
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(microstrain_3dm_node src/microstrain_3dm_node.cpp)

rosidl_target_interfaces(microstrain_3dm_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(microstrain_3dm_node microstrain_3dm microstrain_diagnostic_updater)

ament_target_dependencies(microstrain_3dm_node rclcpp mscl_msgs)

install(TARGETS microstrain_3dm microstrain_diagnostic_updater
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install( TARGETS microstrain_3dm_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_libraries(${PROJECT_NAME}_3dm microstrain_diagnostic_updater)
ament_export_include_directories(include)
ament_export_dependencies(ament_cmake rosidl_default_runtime)

ament_package()
