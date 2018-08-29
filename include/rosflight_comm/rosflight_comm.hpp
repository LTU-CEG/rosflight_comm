#pragma once

// System includes
#include <string>

// Ros includes
#include <ros/ros.h>

// Messages
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Status.h>

#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/RCRaw.h>

class rosflight_communication
{
private:
  // Variables
  mav_msgs::Status status_msg_;

  // ROS parts
  ros::NodeHandle priv_nh_;
  ros::NodeHandle public_nh_;

  ros::Subscriber rosflight_imu_sub_;
  ros::Publisher mav_imu_pub_;

  ros::Subscriber mav_rpyrt_sub_;
  ros::Publisher rosflight_rpyrt_pub_;

  ros::Subscriber rosflight_rc_sub_;
  ros::Publisher mav_rc_pub_;

  ros::Subscriber rosflight_status_sub_;
  ros::Publisher mav_status_pub_;

  // Callback functions for subscribed messages
  void callback_imu(const sensor_msgs::ImuConstPtr& msg);

  void callback_roll_pitch_yawrate_thrust(
      const mav_msgs::RollPitchYawrateThrustConstPtr& msg);

  void callback_rc_input(const rosflight_msgs::RCRawConstPtr& msg);

  void callback_status(const rosflight_msgs::StatusConstPtr& msg);

public:
  rosflight_communication(ros::NodeHandle& pub_nh, ros::NodeHandle& priv_nh);

  void ros_loop();
};
