/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord GX5-Series Driver Definition File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2020, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MICROSTRAIN_3DM_H
#define _MICROSTRAIN_3DM_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


//MSCL
#include "mscl/mscl.h"
#include "mscl_msgs/msg/status.hpp"
#include "mscl_msgs/msg/rtk_status.hpp"
#include "mscl_msgs/msg/filter_status.hpp"
#include "mscl_msgs/msg/filter_heading.hpp"
#include "mscl_msgs/msg/filter_heading_state.hpp"
#include "mscl_msgs/msg/gps_correlation_timestamp_stamped.hpp"


#include "ros_mscl/srv/set_accel_bias.hpp"
#include "ros_mscl/srv/get_accel_bias.hpp"
#include "ros_mscl/srv/set_gyro_bias.hpp"
#include "ros_mscl/srv/get_gyro_bias.hpp"
#include "ros_mscl/srv/set_hard_iron_values.hpp"
#include "ros_mscl/srv/get_hard_iron_values.hpp"
#include "ros_mscl/srv/set_soft_iron_matrix.hpp"
#include "ros_mscl/srv/get_soft_iron_matrix.hpp"
#include "ros_mscl/srv/set_complementary_filter.hpp"
#include "ros_mscl/srv/get_complementary_filter.hpp"
#include "ros_mscl/srv/init_filter_euler.hpp"
#include "ros_mscl/srv/init_filter_heading.hpp"
#include "ros_mscl/srv/device_settings.hpp"
#include "ros_mscl/srv/set_accel_bias_model.hpp"
#include "ros_mscl/srv/get_accel_bias_model.hpp"
#include "ros_mscl/srv/set_gravity_adaptive_vals.hpp"
#include "ros_mscl/srv/get_gravity_adaptive_vals.hpp"
#include "ros_mscl/srv/set_sensor2_vehicle_rotation.hpp"
#include "ros_mscl/srv/get_sensor2_vehicle_rotation.hpp"
#include "ros_mscl/srv/set_sensor2_vehicle_offset.hpp"
#include "ros_mscl/srv/get_sensor2_vehicle_offset.hpp"
#include "ros_mscl/srv/set_reference_position.hpp"
#include "ros_mscl/srv/get_reference_position.hpp"
#include "ros_mscl/srv/set_coning_sculling_comp.hpp"
#include "ros_mscl/srv/get_coning_sculling_comp.hpp"
#include "ros_mscl/srv/set_estimation_control_flags.hpp"
#include "ros_mscl/srv/get_estimation_control_flags.hpp"
#include "ros_mscl/srv/set_dynamics_mode.hpp"
#include "ros_mscl/srv/get_dynamics_mode.hpp"
#include "ros_mscl/srv/set_zero_angle_update_threshold.hpp"
#include "ros_mscl/srv/get_zero_angle_update_threshold.hpp"
#include "ros_mscl/srv/set_zero_velocity_update_threshold.hpp"
#include "ros_mscl/srv/get_zero_velocity_update_threshold.hpp"
#include "ros_mscl/srv/set_tare_orientation.hpp"
#include "ros_mscl/srv/set_accel_noise.hpp"
#include "ros_mscl/srv/get_accel_noise.hpp"
#include "ros_mscl/srv/set_gyro_noise.hpp"
#include "ros_mscl/srv/get_gyro_noise.hpp"
#include "ros_mscl/srv/set_mag_noise.hpp"
#include "ros_mscl/srv/get_mag_noise.hpp"
#include "ros_mscl/srv/set_gyro_bias_model.hpp"
#include "ros_mscl/srv/get_gyro_bias_model.hpp"
#include "ros_mscl/srv/set_mag_adaptive_vals.hpp"
#include "ros_mscl/srv/get_mag_adaptive_vals.hpp"
#include "ros_mscl/srv/set_mag_dip_adaptive_vals.hpp"
#include "ros_mscl/srv/get_mag_dip_adaptive_vals.hpp"
#include "ros_mscl/srv/set_heading_source.hpp"
#include "ros_mscl/srv/get_heading_source.hpp"
#include "ros_mscl/srv/get_sensor2_vehicle_transformation.hpp"
#include "ros_mscl/srv/external_heading_update.hpp"
#include "ros_mscl/srv/set_relative_position_reference.hpp"
#include "ros_mscl/srv/get_relative_position_reference.hpp"





/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

#define SECS_PER_WEEK (60L*60*24*7)
#define UTC_GPS_EPOCH_DUR (315964800)

#define USTRAIN_G 9.80665  // from section 5.1.1 in https://www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf

//Macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x*1000.0)

#define GNSS1_ID 0
#define GNSS2_ID 1
#define NUM_GNSS 2


/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Microstrain
{
///
/// \brief Microstrain class
///

class Microstrain : public rclcpp::Node
{
public:

    Microstrain();
    ~Microstrain() = default;

    void setup();
    void process();
    void cleanup();

    void parse_mip_packet(const mscl::MipDataPacket& packet);
    void parse_imu_packet(const mscl::MipDataPacket& packet);
    void parse_filter_packet(const mscl::MipDataPacket& packet);
    void parse_gnss_packet(const mscl::MipDataPacket& packet, int gnss_id);
    void parse_rtk_packet(const mscl::MipDataPacket& packet);

    void device_status_callback();

    void device_report(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void get_basic_status(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void get_diagnostic_report(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

    void set_accel_bias(ros_mscl::srv::SetAccelBias::Request::SharedPtr req, ros_mscl::srv::SetAccelBias::Response::SharedPtr res);
    void get_accel_bias(ros_mscl::srv::GetAccelBias::Request::SharedPtr req, ros_mscl::srv::GetAccelBias::Response::SharedPtr res);

    void set_gyro_bias(ros_mscl::srv::SetGyroBias::Request::SharedPtr req, ros_mscl::srv::SetGyroBias::Response::SharedPtr res);
    void get_gyro_bias(ros_mscl::srv::GetGyroBias::Request::SharedPtr req, ros_mscl::srv::GetGyroBias::Response::SharedPtr res);

    void gyro_bias_capture(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

    void set_hard_iron_values(ros_mscl::srv::SetHardIronValues::Request::SharedPtr req, ros_mscl::srv::SetHardIronValues::Response::SharedPtr res);
    void get_hard_iron_values(ros_mscl::srv::GetHardIronValues::Request::SharedPtr req, ros_mscl::srv::GetHardIronValues::Response::SharedPtr res);

    void set_soft_iron_matrix(ros_mscl::srv::SetSoftIronMatrix::Request::SharedPtr req, ros_mscl::srv::SetSoftIronMatrix::Response::SharedPtr res);
    void get_soft_iron_matrix(ros_mscl::srv::GetSoftIronMatrix::Request::SharedPtr req, ros_mscl::srv::GetSoftIronMatrix::Response::SharedPtr res);

    void set_complementary_filter(ros_mscl::srv::SetComplementaryFilter::Request::SharedPtr req, ros_mscl::srv::SetComplementaryFilter::Response::SharedPtr res);
    void get_complementary_filter(ros_mscl::srv::GetComplementaryFilter::Request::SharedPtr req, ros_mscl::srv::GetComplementaryFilter::Response::SharedPtr res);

    void set_coning_sculling_comp(ros_mscl::srv::SetConingScullingComp::Request::SharedPtr req, ros_mscl::srv::SetConingScullingComp::Response::SharedPtr res);
    void get_coning_sculling_comp(ros_mscl::srv::GetConingScullingComp::Request::SharedPtr req, ros_mscl::srv::GetConingScullingComp::Response::SharedPtr res);

    void set_sensor2vehicle_rotation(ros_mscl::srv::SetSensor2VehicleRotation::Request::SharedPtr req, ros_mscl::srv::SetSensor2VehicleRotation::Response::SharedPtr res);
    void get_sensor2vehicle_rotation(ros_mscl::srv::GetSensor2VehicleRotation::Request::SharedPtr req, ros_mscl::srv::GetSensor2VehicleRotation::Response::SharedPtr res);

    void set_sensor2vehicle_offset(ros_mscl::srv::SetSensor2VehicleOffset::Request::SharedPtr req, ros_mscl::srv::SetSensor2VehicleOffset::Response::SharedPtr res);
    void get_sensor2vehicle_offset(ros_mscl::srv::GetSensor2VehicleOffset::Request::SharedPtr req, ros_mscl::srv::GetSensor2VehicleOffset::Response::SharedPtr res);

    void get_sensor2vehicle_transformation(ros_mscl::srv::GetSensor2VehicleTransformation::Request::SharedPtr req, ros_mscl::srv::GetSensor2VehicleTransformation::Response::SharedPtr res);

    void reset_filter(std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr resp);

    void init_filter_euler(ros_mscl::srv::InitFilterEuler::Request::SharedPtr req, ros_mscl::srv::InitFilterEuler::Response::SharedPtr res);
    void init_filter_heading(ros_mscl::srv::InitFilterHeading::Request::SharedPtr req, ros_mscl::srv::InitFilterHeading::Response::SharedPtr res);

    void set_heading_source(ros_mscl::srv::SetHeadingSource::Request::SharedPtr req, ros_mscl::srv::SetHeadingSource::Response::SharedPtr res);
    void get_heading_source(ros_mscl::srv::GetHeadingSource::Request::SharedPtr req, ros_mscl::srv::GetHeadingSource::Response::SharedPtr res);

    void set_reference_position(ros_mscl::srv::SetReferencePosition::Request::SharedPtr req, ros_mscl::srv::SetReferencePosition::Response::SharedPtr res);
    void get_reference_position(ros_mscl::srv::GetReferencePosition::Request::SharedPtr req, ros_mscl::srv::GetReferencePosition::Response::SharedPtr res);

    void set_estimation_control_flags(ros_mscl::srv::SetEstimationControlFlags::Request::SharedPtr req, ros_mscl::srv::SetEstimationControlFlags::Response::SharedPtr res);
    void get_estimation_control_flags(ros_mscl::srv::GetEstimationControlFlags::Request::SharedPtr req, ros_mscl::srv::GetEstimationControlFlags::Response::SharedPtr res);

    void set_dynamics_mode(ros_mscl::srv::SetDynamicsMode::Request::SharedPtr req, ros_mscl::srv::SetDynamicsMode::Response::SharedPtr res);
    void get_dynamics_mode(ros_mscl::srv::GetDynamicsMode::Request::SharedPtr req, ros_mscl::srv::GetDynamicsMode::Response::SharedPtr res);

    void set_zero_angle_update_threshold(ros_mscl::srv::SetZeroAngleUpdateThreshold::Request::SharedPtr req, ros_mscl::srv::SetZeroAngleUpdateThreshold::Response::SharedPtr res);
    void get_zero_angle_update_threshold(ros_mscl::srv::GetZeroAngleUpdateThreshold::Request::SharedPtr req, ros_mscl::srv::GetZeroAngleUpdateThreshold::Response::SharedPtr res);

    void set_zero_velocity_update_threshold(ros_mscl::srv::SetZeroVelocityUpdateThreshold::Request::SharedPtr req, ros_mscl::srv::SetZeroVelocityUpdateThreshold::Response::SharedPtr res);
    void get_zero_velocity_update_threshold(ros_mscl::srv::GetZeroVelocityUpdateThreshold::Request::SharedPtr req, ros_mscl::srv::GetZeroVelocityUpdateThreshold::Response::SharedPtr res);

    void set_tare_orientation(ros_mscl::srv::SetTareOrientation::Request::SharedPtr req, ros_mscl::srv::SetTareOrientation::Response::SharedPtr res);

    void commanded_vel_zupt(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void commanded_ang_rate_zupt(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

    void set_accel_noise(ros_mscl::srv::SetAccelNoise::Request::SharedPtr req, ros_mscl::srv::SetAccelNoise::Response::SharedPtr res);
    void get_accel_noise(ros_mscl::srv::GetAccelNoise::Request::SharedPtr req, ros_mscl::srv::GetAccelNoise::Response::SharedPtr res);

    void set_gyro_noise(ros_mscl::srv::SetGyroNoise::Request::SharedPtr req, ros_mscl::srv::SetGyroNoise::Response::SharedPtr res);
    void get_gyro_noise(ros_mscl::srv::GetGyroNoise::Request::SharedPtr req, ros_mscl::srv::GetGyroNoise::Response::SharedPtr res);

    void set_mag_noise(ros_mscl::srv::SetMagNoise::Request::SharedPtr req, ros_mscl::srv::SetMagNoise::Response::SharedPtr res);
    void get_mag_noise(ros_mscl::srv::GetMagNoise::Request::SharedPtr req, ros_mscl::srv::GetMagNoise::Response::SharedPtr res);

    void set_gyro_bias_model(ros_mscl::srv::SetGyroBiasModel::Request::SharedPtr req, ros_mscl::srv::SetGyroBiasModel::Response::SharedPtr res);
    void get_gyro_bias_model(ros_mscl::srv::GetGyroBiasModel::Request::SharedPtr req, ros_mscl::srv::GetGyroBiasModel::Response::SharedPtr res);

    void set_accel_bias_model(ros_mscl::srv::SetAccelBiasModel::Request::SharedPtr req, ros_mscl::srv::SetAccelBiasModel::Response::SharedPtr res);
    void get_accel_bias_model(ros_mscl::srv::GetAccelBiasModel::Request::SharedPtr req, ros_mscl::srv::GetAccelBiasModel::Response::SharedPtr res);

    void set_gravity_adaptive_vals(ros_mscl::srv::SetGravityAdaptiveVals::Request::SharedPtr req, ros_mscl::srv::SetGravityAdaptiveVals::Response::SharedPtr res);
    void get_gravity_adaptive_vals(ros_mscl::srv::GetGravityAdaptiveVals::Request::SharedPtr req, ros_mscl::srv::GetGravityAdaptiveVals::Response::SharedPtr res);

    void set_mag_adaptive_vals(ros_mscl::srv::SetMagAdaptiveVals::Request::SharedPtr req, ros_mscl::srv::SetMagAdaptiveVals::Response::SharedPtr res);
    void get_mag_adaptive_vals(ros_mscl::srv::GetMagAdaptiveVals::Request::SharedPtr req, ros_mscl::srv::GetMagAdaptiveVals::Response::SharedPtr res);

    void set_mag_dip_adaptive_vals(ros_mscl::srv::SetMagDipAdaptiveVals::Request::SharedPtr req, ros_mscl::srv::SetMagDipAdaptiveVals::Response::SharedPtr res);
    void get_mag_dip_adaptive_vals(ros_mscl::srv::GetMagDipAdaptiveVals::Request::SharedPtr req, ros_mscl::srv::GetMagDipAdaptiveVals::Response::SharedPtr res);

    void external_heading_update(ros_mscl::srv::ExternalHeadingUpdate::Request::SharedPtr req, ros_mscl::srv::ExternalHeadingUpdate::Response::SharedPtr res);

    void set_relative_position_reference(ros_mscl::srv::SetRelativePositionReference::Request::SharedPtr req, ros_mscl::srv::SetRelativePositionReference::Response::SharedPtr res);
    void get_relative_position_reference(ros_mscl::srv::GetRelativePositionReference::Request::SharedPtr req, ros_mscl::srv::GetRelativePositionReference::Response::SharedPtr res);

    void device_settings(ros_mscl::srv::DeviceSettings::Request::SharedPtr req, ros_mscl::srv::DeviceSettings::Response::SharedPtr res);

    void velocity_zupt_callback(const std_msgs::msg::Bool::SharedPtr state);
    void vel_zupt();

    void ang_zupt_callback(const std_msgs::msg::Bool::SharedPtr state);
    void ang_zupt();

    void external_gps_time_callback(const sensor_msgs::msg::TimeReference::SharedPtr time);

    int m_spin_rate;

private:

    //Convience for printing packet stats
    void print_packet_stats();

    //Variables/fields
    std::unique_ptr<mscl::InertialNode> m_inertial_device;

    //Info for converting to the ENU frame
    bool m_use_enu_frame;
    tf2::Matrix3x3 m_t_ned2enu;

    //Flag for using device timestamp instead of PC received time
    bool m_use_device_timestamp;

    //Packet Counters (valid, timeout, and checksum errors)
    uint32_t m_imu_valid_packet_count;
    uint32_t m_gnss_valid_packet_count[NUM_GNSS];
    uint32_t m_filter_valid_packet_count;
    uint32_t m_rtk_valid_packet_count;

    uint32_t m_imu_timeout_packet_count;
    uint32_t m_gnss_timeout_packet_count[NUM_GNSS];
    uint32_t m_filter_timeout_packet_count;

    uint32_t m_imu_checksum_error_packet_count;
    uint32_t m_gnss_checksum_error_packet_count[NUM_GNSS];
    uint32_t m_filter_checksum_error_packet_count;


    //Data field storage
    //IMU
    float m_curr_imu_mag_x;
    float m_curr_imu_mag_y;
    float m_curr_imu_mag_z;

    mscl::Vector m_curr_ahrs_quaternion;

    //FILTER
    double m_gps_leap_seconds;

    double m_curr_filter_pos_lat;
    double m_curr_filter_pos_long;
    double m_curr_filter_pos_height;

    float m_curr_filter_vel_north;
    float m_curr_filter_vel_east;
    float m_curr_filter_vel_down;

    mscl::Vector m_curr_filter_quaternion;

    float m_curr_filter_roll;
    float m_curr_filter_pitch;
    float m_curr_filter_yaw;

    float m_curr_filter_angular_rate_x;
    float m_curr_filter_angular_rate_y;
    float m_curr_filter_angular_rate_z;

    float m_curr_filter_pos_uncert_north;
    float m_curr_filter_pos_uncert_east;
    float m_curr_filter_pos_uncert_down;

    float m_curr_filter_vel_uncert_north;
    float m_curr_filter_vel_uncert_east;
    float m_curr_filter_vel_uncert_down;

    float m_curr_filter_att_uncert_roll;
    float m_curr_filter_att_uncert_pitch;
    float m_curr_filter_att_uncert_yaw;

    //IMU Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_mag_pub;
    rclcpp::Publisher<mscl_msgs::msg::GPSCorrelationTimestampStamped>::SharedPtr m_gps_corr_pub;

    //GNSS Publishers
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_gnss_pub[NUM_GNSS];
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_gnss_odom_pub[NUM_GNSS];
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr m_gnss_time_pub[NUM_GNSS];

    //RTK Data publisher
    rclcpp::Publisher<mscl_msgs::msg::RTKStatus>::SharedPtr m_rtk_pub;

    //Filter Publishers
    rclcpp::Publisher<mscl_msgs::msg::FilterStatus>::SharedPtr m_filter_status_pub;
    rclcpp::Publisher<mscl_msgs::msg::FilterHeading>::SharedPtr m_filter_heading_pub;
    rclcpp::Publisher<mscl_msgs::msg::FilterHeadingState>::SharedPtr m_filter_heading_state_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_filter_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_filtered_imu_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_filter_relative_pos_pub;

    //Device Status Publisher
    rclcpp::Publisher<mscl_msgs::msg::Status>::SharedPtr m_device_status_pub;

    //ZUPT subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_filter_vel_state_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_filter_ang_state_sub;

    //External GNSS subscriber
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr m_external_gps_time_sub;

    //IMU Messages
    sensor_msgs::msg::Imu           m_imu_msg;
    sensor_msgs::msg::MagneticField m_mag_msg;
    mscl_msgs::msg::GPSCorrelationTimestampStamped m_gps_corr_msg;

    //GNSS Messages
    sensor_msgs::msg::NavSatFix     m_gnss_msg[NUM_GNSS];
    nav_msgs::msg::Odometry         m_gnss_odom_msg[NUM_GNSS];
    sensor_msgs::msg::TimeReference m_gnss_time_msg[NUM_GNSS];

    //RTK Messages
    mscl_msgs::msg::RTKStatus   m_rtk_msg;

    //Filter Messages
    nav_msgs::msg::Odometry                 m_filter_msg;
    sensor_msgs::msg::Imu                   m_filtered_imu_msg;
    nav_msgs::msg::Odometry                 m_filter_relative_pos_msg;
    mscl_msgs::msg::FilterStatus            m_filter_status_msg;
    mscl_msgs::msg::FilterHeadingState      m_filter_heading_state_msg;
    mscl_msgs::msg::FilterHeading           m_filter_heading_msg;

    //Device Status Message
    mscl_msgs::msg::Status m_device_status_msg;

    //Frame ids
    std::string m_imu_frame_id;
    std::string m_gnss_frame_id[NUM_GNSS];
    std::string m_filter_frame_id;
    std::string m_filter_child_frame_id;

    //Topic strings
    std::string m_velocity_zupt_topic;
    std::string m_angular_zupt_topic;
    std::string m_external_gps_time_topic;

    //Publish data flags
    bool m_publish_imu;
    bool m_publish_gps_corr;
    bool m_publish_gnss[NUM_GNSS];
    bool m_publish_filter;
    bool m_publish_filter_relative_pos;
    bool m_publish_rtk;

    //ZUPT, angular ZUPT topic listener variables
    bool m_angular_zupt;
    bool m_velocity_zupt;

    bool m_vel_still;
    bool m_ang_still;

    //Static covariance vectors
    std::vector<double> m_imu_linear_cov;
    std::vector<double> m_imu_angular_cov;
    std::vector<double> m_imu_orientation_cov;

    // Update rates
    int m_imu_data_rate;
    int m_gnss_data_rate[NUM_GNSS];
    int m_filter_data_rate;

    //Gnss antenna offsets
    std::vector<double> m_gnss_antenna_offset[NUM_GNSS];

    //Various settings variables
    clock_t m_start;
    uint8_t m_com_mode;
    float   m_field_data[3];
    float   m_soft_iron[9];
    float   m_soft_iron_readback[9];
    float   m_angles[3];
    float   m_heading_angle;
    float   m_readback_angles[3];
    float   m_noise[3];
    float   m_beta[3];
    float   m_readback_beta[3];
    float   m_readback_noise[3];
    float   m_offset[3];
    float   m_readback_offset[3];
    double  m_reference_position_command[3];
    double  m_reference_position_readback[3];
    uint8_t m_dynamics_mode;
    int m_max_rate;
    uint32_t m_status_counter;

    //Raw data file parameters
    bool          m_raw_file_enable;
    bool          m_raw_file_include_support_data;
    std::ofstream m_raw_file;
}; //Microstrain class


// Define wrapper functions that call the Microstrain member functions
#ifdef __cplusplus
extern "C"
#endif
{

#ifdef __cplusplus
}
#endif

} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
