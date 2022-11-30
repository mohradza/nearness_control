#include <nearness_control/data_collector.h>

namespace data_collector {
dataCollector::dataCollector(const ros::NodeHandle &node_handle,
                             const ros::NodeHandle &private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle) {
  this->init();
}

void dataCollector::init() {
  // SUBSCRIBERS
  sub_cmds_ = nh_.subscribe("cmd_vel", 1, &dataCollector::cmdCb, this);
  sub_odom_ = nh_.subscribe("odometry", 20, &dataCollector::odomCb, this);
  sub_imu_ = nh_.subscribe("imu", 10, &dataCollector::imuCb, this);

  // PUBLISHERS
  pub_synced_odom_ = nh_.advertise<nav_msgs::Odometry>("synced_odom", 100);
  pub_synced_cmds_ =
      nh_.advertise<geometry_msgs::TwistStamped>("synced_cmds", 100);

  publish_rate_ = 125.0;
  publish_period_s_ = 1.0 / publish_rate_;
  last_publish_time_ = ros::Time::now();
  first_pass_ = true;
}

void dataCollector::odomCb(const nav_msgs::OdometryConstPtr &odom_msg) {

  sim_time_ = odom_msg->header.stamp;
  pub_dt_s_ = (sim_time_ - last_publish_time_).toSec();

  if (pub_dt_s_ >= publish_period_s_) {
    tf::Quaternion quat(
        odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    double p, q, r;
    double roll_dot, pitch_dot, yaw_dot;
    m.getRPY(roll, pitch, yaw);
    Matrix3f mat_yaw(3, 3), mat_pitch(3, 3), mat_roll(3, 3), mat_rot(3, 3);

    mat_yaw << cos(yaw), sin(yaw), 0.0, -sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;
    mat_pitch << cos(pitch), 0.0, -sin(pitch), 0.0, 1.0, 0.0, sin(pitch), 0.0,
        cos(pitch);
    mat_roll << 1.0, 0.0, 0.0, 0.0, cos(roll), sin(roll), 0.0, -sin(roll),
        cos(roll);
    mat_rot = mat_yaw * mat_pitch * mat_roll;

    Vector3f world_vels(odom_msg->twist.twist.linear.x,
                        odom_msg->twist.twist.linear.y,
                        odom_msg->twist.twist.linear.z);

    Vector3f body_vels_vec(0.0, 0.0, 0.0);
    body_vels_vec = mat_rot * world_vels;

    current_odom_ = *odom_msg;
    current_cmds_.header.stamp = current_odom_.header.stamp;

    pub_synced_odom_.publish(current_odom_);
    pub_synced_cmds_.publish(current_cmds_);

    ROS_INFO_THROTTLE(
        1.0,
        "Control Commands: u_u: %0.3f, u_v: %0.3f, u_w: %0.3f, u_r: %0.3f,",
        current_cmds_.twist.linear.x, current_cmds_.twist.linear.y,
        current_cmds_.twist.linear.z, current_cmds_.twist.angular.z);
    ROS_INFO_THROTTLE(
        1.0,
        "Body Velocities :   u: %0.3f,   v: %0.3f,   w: %0.3f,   r: %0.3f ",
        body_vels_vec(0), body_vels_vec(1), body_vels_vec(2),
        ang_vels_.angular.z);
    ROS_INFO_THROTTLE(1.0, " ");

    last_publish_time_ = sim_time_;
  }
}

void dataCollector::cmdCb(const geometry_msgs::TwistConstPtr &cmd_msg) {
  current_cmds_.twist = *cmd_msg;
}

void dataCollector::imuCb(const sensor_msgs::ImuConstPtr &imu_msg) {

  imu_orientation_ = imu_msg->orientation;
  ang_vels_.angular.x = imu_msg->angular_velocity.x;
  ang_vels_.angular.y = imu_msg->angular_velocity.y;
  ang_vels_.angular.z = imu_msg->angular_velocity.z;
}

bool dataCollector::cmpf(float A, float B, float epsilon = 0.000001) {
  return (fabs(A - B) < epsilon);
}

} // namespace data_collector
