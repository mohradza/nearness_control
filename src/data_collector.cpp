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

    current_odom_ = *odom_msg;
    current_cmds_.header.stamp = current_odom_.header.stamp;

    pub_synced_odom_.publish(current_odom_);
    pub_synced_cmds_.publish(current_cmds_);

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
