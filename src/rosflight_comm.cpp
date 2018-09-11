// User includes
#include "rosflight_comm/rosflight_comm.hpp"

// Callback functions for subscribed messages
void rosflight_communication::callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
  sensor_msgs::Imu imu = *msg;

  // rosflight is in NED frame, we use NWU
  imu.angular_velocity.y *= -1;
  imu.angular_velocity.z *= -1;

  imu.linear_acceleration.y *= -1;
  imu.linear_acceleration.z *= -1;

  mav_imu_pub_.publish(imu);
}

void rosflight_communication::callback_roll_pitch_yawrate_thrust(
    const mav_msgs::RollPitchYawrateThrustConstPtr& msg)
{
  rosflight_msgs::Command cmd;

  cmd.header.stamp = msg->header.stamp;
  cmd.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  cmd.ignore = rosflight_msgs::Command::IGNORE_NONE;

  // rosflight is in NED frame, we use NWU
  // TODO: Check signs on all these
  cmd.x = msg->roll;
  cmd.y = -msg->pitch;
  cmd.z = -msg->yaw_rate;
  cmd.F = msg->thrust.z;

  rosflight_rpyrt_pub_.publish(cmd);
}

struct bound
{
  bound(float min_, float max_)
      : min(min), max(max), center(0), has_center(false)
  {
  }

  bound(float min_, float max_, float center_)
      : min(min), max(max), center(center_), has_center(true)
  {
  }

  bound() = delete;

  bool has_center;
  float min;
  float center;
  float max;
};

float bound_and_scale(const bound& output_bound, const bound& input_bound,
                      const float input)
{
  const auto bounded_input = [&]() {
    if (input < input_bound.min)
      return input_bound.min;
    if (input > input_bound.max)
      return input_bound.max;
    else
      return input;
  }();

  if (input_bound.has_center)
  {
    if (bounded_input > input_bound.center)
    {
      const auto i = (bounded_input - input_bound.center) /
                     (input_bound.max - input_bound.center);

      return output_bound.center + (output_bound.max - output_bound.center) * i;
    }
    else if (bounded_input < input_bound.center)
    {
      const auto i = (bounded_input - input_bound.min) /
                     (input_bound.center - input_bound.min);

      return output_bound.min + (output_bound.center - output_bound.min) * i;
    }
    else
    {
      return output_bound.center;
    }
  }
  else
  {
    const auto i =
        (bounded_input - input_bound.min) / (input_bound.max - input_bound.min);

    return output_bound.min + (output_bound.max - output_bound.min) * i;
  }
}

void rosflight_communication::callback_rc_input(
    const rosflight_msgs::RCRawConstPtr& msg)
{
  // TODO: Add calibration and scaling
  sensor_msgs::Joy joy;

  joy.header.stamp = msg->header.stamp;

  // Calibration and scaling
  const bound ib(1000, 2000, 1500);
  const bound ib_thr(1000, 2000);
  const bound ob(0, 1, 0.5);
  const bound ob_thr(0, 1);

  for (int i = 0; i < 8; i++)
    joy.axes.push_back(bound_and_scale(ib, ob, msg->values[i]));

  joy.axes[2] = bound_and_scale(ib_thr, ob_thr, msg->values[2]);

  mav_rc_pub_.publish(joy);
}

void rosflight_communication::callback_status(
    const rosflight_msgs::StatusConstPtr& msg)
{
  status_msg_.header.stamp = ros::Time::now();

  status_msg_.command_interface_enabled = msg->offboard;

  if (msg->armed == true)
  {
    status_msg_.in_air = true;
    status_msg_.motor_status = mav_msgs::Status::MOTOR_STATUS_RUNNING;
  }
  else
  {
    status_msg_.in_air = false;
    status_msg_.motor_status = mav_msgs::Status::MOTOR_STATUS_STOPPED;
  }

  mav_status_pub_.publish(status_msg_);
}

rosflight_communication::rosflight_communication(ros::NodeHandle& pub_nh,
                                                 ros::NodeHandle& priv_nh)
    : public_nh_(pub_nh), priv_nh_(priv_nh)
{
  std::string mav_name_;

  if (!priv_nh.getParam("mav_name", mav_name_))
  {
    mav_name_ = "no name";
  }

  //
  // Local variables
  //
  status_msg_.battery_voltage = 0;
  status_msg_.command_interface_enabled = false;
  status_msg_.cpu_load = 0;
  status_msg_.flight_time = 0;
  status_msg_.gps_num_satellites = 0;
  status_msg_.gps_status = mav_msgs::Status::GPS_STATUS_NO_LOCK;
  status_msg_.in_air = false;
  status_msg_.motor_status = mav_msgs::Status::MOTOR_STATUS_STOPPED;
  status_msg_.rc_command_mode = mav_msgs::Status::RC_COMMAND_ATTITUDE;
  status_msg_.system_uptime = 0;
  status_msg_.vehicle_name = mav_name_;
  status_msg_.vehicle_type = "quadrotor";
  status_msg_.header.stamp = ros::Time::now();

  rosflight_imu_sub_ = public_nh_.subscribe(
      "rosflight/imu", 5, &rosflight_communication::callback_imu, this,
      ros::TransportHints().tcpNoDelay());

  mav_imu_pub_ = public_nh_.advertise< sensor_msgs::Imu >(
      mav_msgs::default_topics::IMU, 1);

  mav_rpyrt_sub_ = public_nh_.subscribe(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 5,
      &rosflight_communication::callback_roll_pitch_yawrate_thrust, this,
      ros::TransportHints().tcpNoDelay());

  rosflight_rpyrt_pub_ =
      public_nh_.advertise< rosflight_msgs::Command >("command", 1);

  rosflight_rc_sub_ = public_nh_.subscribe(
      "rc_raw", 5, &rosflight_communication::callback_rc_input, this,
      ros::TransportHints().tcpNoDelay());

  mav_status_pub_ = public_nh_.advertise< mav_msgs::Status >(
      mav_msgs::default_topics::STATUS, 1);

  rosflight_status_sub_ = public_nh_.subscribe(
      "rosflight/status", 5, &rosflight_communication::callback_status, this,
      ros::TransportHints().tcpNoDelay());

  mav_rc_pub_ =
      public_nh_.advertise< sensor_msgs::Joy >(mav_msgs::default_topics::RC, 1);
}

void rosflight_communication::ros_loop()
{
  ros::spin();
}
