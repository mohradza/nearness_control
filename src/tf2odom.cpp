#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

using namespace Eigen;

bool new_cmd_;
geometry_msgs::TwistStamped cmd_;
void cmdCallback(const geometry_msgs::TwistStampedConstPtr& cmd_msg){
  new_cmd_ = true;
  cmd_ = *cmd_msg;
}

nav_msgs::Odometry odom_;
bool new_odom_;
ros::Time last_time;
void odomCallback(const nav_msgs::OdometryConstPtr& msg){
  odom_ = *msg;
}

Vector3f angular_velocities_;
Vector3f linear_accelerations_;
void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg){

  Matrix3f mat_pitch(3,3), mat_roll(3,3), mat_rot(3,3);

  float pitch = M_PI;
  float roll = M_PI;

  mat_pitch << cos(pitch), 0.0, -sin(pitch), 0.0, 1.0, 0.0, sin(pitch), 0.0, cos(pitch);
  mat_roll << 1.0, 0.0, 0.0, 0.0, cos(roll), sin(roll), 0.0, -sin(roll), cos(roll);

  // Do a 3-2 rotation to correct sensor signal signs
  mat_rot = mat_pitch*mat_roll;

  Vector3f ang_vels(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
  angular_velocities_ = mat_rot*ang_vels;
  Vector3f lin_accels(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
  linear_accelerations_ = mat_rot*lin_accels;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2odom");

  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Publisher robot_odom_pub =
    node.advertise<nav_msgs::Odometry>("odometry/ground_truth", 10);
  ros::Publisher body_vel_pub =
        node.advertise<geometry_msgs::Twist>("body_velocity/ground_truth", 10);
  ros::Publisher cmd_vel_pub =
    node.advertise<geometry_msgs::TwistStamped>("cmd_vel/ground_truth", 10);

    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 1, cmdCallback);
    // ros::Subscriber odom_sub = nh.subscribe("ground_truth/odom", 1, odomCallback);

  ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, imuCallback);


  tf::TransformListener listener;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped tf_msg;
  geometry_msgs::Quaternion quat_msg;

  geometry_msgs::PointStamped point_msg;
  std::string frame_name;
  nh.param<std::string>("OHRAD_X3", frame_name, "default_value");
  odom_msg.header.frame_id = "world";
  new_cmd_ = false;

  //geometry_msgs::Twist cmd;

  float last_x_pos = 0.0;
  float last_y_pos = 0.0;
  float last_z_pos = 0.0;

  float last_x_vel_filt = 0.0;
  float last_y_vel_filt = 0.0;
  float last_z_vel_filt = 0.0;
  float alpha_vel = .2;

  float last_roll = 0.0;
  float last_pitch = 0.0;
  float last_yaw = 0.0;

  float dt = 0.0;
  float diff = 0.0;

  geometry_msgs::Twist body_vels;

  ros::Time last_odom_time = ros::Time::now();

  // tf::StampedTransform transform;
  tf::StampedTransform transform;

  ros::Rate rate(500);
  while (node.ok()){
    // try{
    //   listener.lookupTransform("/simple_tunnel_01", "/OHRAD_X3",
    //      ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex){
    //   //ROS_ERROR("%s",ex.what());
    //   // ros::Duration(0.01).sleep();
    // }

    // odom_msg.header.stamp = ros::Time::now();
    odom_msg = odom_;
    // odom_msg.header.stamp = ros::Time::now();
    // dt = (odom_msg.header.stamp - last_odom_time).toSec();

    // dt = 0.02;

    // if(dt < 0.0099){
    //   ROS_INFO("dt: %f", dt);
    // }

    // last_odom_time = odom_msg.header.stamp;

      // odom_msg.pose.pose.position.x = transform.getOrigin().x();
      // odom_msg.pose.pose.position.y = transform.getOrigin().y();
      // odom_msg.pose.pose.position.z = transform.getOrigin().z();
      //
      // float vx, vy, vz, vx_filt, vy_filt, vz_filt;
      // // ROS_INFO("last_x_pos: %f, current_pos: %f, dt: %f", last_x_pos,odom_msg.pose.pose.position.x, dt);
      //
      // vx = (odom_msg.pose.pose.position.x - last_x_pos)/dt;
      // vy = (odom_msg.pose.pose.position.y - last_y_pos)/dt;
      // vz = (odom_msg.pose.pose.position.z - last_z_pos)/dt;
      //
      // last_x_pos = odom_msg.pose.pose.position.x;
      // last_y_pos = odom_msg.pose.pose.position.y;
      // last_z_pos = odom_msg.pose.pose.position.z;
      //
      // odom_msg.twist.twist.linear.x = vx;
      // odom_msg.twist.twist.linear.y = vy;
      // odom_msg.twist.twist.linear.z = vz;
      // ROS_INFO("vx: %f, last_xv_filt: %f", vx, last_x_vel_filt);

      // vx_filt = (1.0-alpha_vel)*vx + alpha_vel*last_x_vel_filt;
      // vy_filt = (1.0-alpha_vel)*vy + alpha_vel*last_y_vel_filt;
      // vz_filt = (1.0-alpha_vel)*vz + alpha_vel*last_z_vel_filt;
      //
      // last_x_vel_filt = vx_filt;
      // last_y_vel_filt = vy_filt;
      // last_z_vel_filt = vz_filt;

      // ROS_INFO("%f", vx_filt);
      // odom_msg.twist.twist.linear.x = vx_filt;
      // odom_msg.twist.twist.linear.y = vy_filt;
      // odom_msg.twist.twist.linear.z = vz_filt;



      // tf::quaternionTFToMsg(transform.getRotation().normalize(), odom_msg.pose.pose.orientation);
      tf::Quaternion quat(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
      tf::Matrix3x3 m(quat);
      double roll, pitch, yaw;
      double p, q, r;
      double roll_dot, pitch_dot, yaw_dot;
      m.getRPY(roll, pitch, yaw);

      // ROS_INFO_THROTTLE(0.5,"roll: %f, pitch: %f, heading: %f", roll, pitch, yaw);

      if (roll > M_PI/2){
        roll -= M_PI;
      } else if( roll < -M_PI/2){
        roll += M_PI;
      }

      // roll_dot = angular_velocities_[0];
      // pitch_dot = angular_velocities_[1];
      // yaw_dot = angular_velocities_[2];

      // ROS_INFO_THROTTLE(0.5,"roll: %f, pitch: %f, heading: %f", roll_dot, pitch_dot, yaw_dot);

      // p = roll_dot - sin(pitch)*yaw_dot;
      // q = cos(roll)*pitch_dot + sin(roll)*cos(pitch)*yaw_dot;
      // r = -sin(roll)*pitch_dot + cos(roll)*cos(pitch)*yaw_dot;

      // last_roll = roll;
      // last_pitch = pitch;
      // last_yaw = yaw;

      // odom_msg.twist.twist.angular.x = p;
      // odom_msg.twist.twist.angular.y = q;
      // odom_msg.twist.twist.angular.z = r;
      //
      // odom_msg.twist.covariance[0] = angular_velocities_[0];
      // odom_msg.twist.covariance[1] = angular_velocities_[1];
      // odom_msg.twist.covariance[2] = angular_velocities_[2];


    // body_vels.linear.x = 0.0;
    // body_vels.linear.y = 0.0;
    // body_vels.linear.z = 0.0;
    //
    // Vector3f world_vels(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);
    Matrix3f mat_yaw(3,3), mat_pitch(3,3), mat_roll(3,3), mat_rot(3,3);
    //
    // mat_yaw << cos(yaw), sin(yaw), 0.0, -sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;
    // mat_pitch << cos(pitch), 0.0, -sin(pitch), 0.0, 1.0, 0.0, sin(pitch), 0.0, cos(pitch);
    // mat_roll << 1.0, 0.0, 0.0, 0.0, cos(roll), sin(roll), 0.0, -sin(roll), cos(roll);
    //
    // mat_rot = mat_yaw*mat_pitch*mat_roll;
    // Vector3f body_vels_vec(0.0, 0.0, 0.0);
    // body_vels_vec = mat_rot*world_vels;

    mat_pitch << cos(-pitch), 0.0, -sin(-pitch), 0.0, 1.0, 0.0, sin(-pitch), 0.0, cos(-pitch);
    mat_roll << 1.0, 0.0, 0.0, 0.0, cos(-roll), sin(-roll), 0.0, -sin(-roll), cos(-roll);
    mat_rot = mat_roll*mat_pitch;

    Vector3f v1_accel_vec(0.0, 0.0, 0.0);
    Vector3f body_accels(linear_accelerations_[0], linear_accelerations_[1], linear_accelerations_[2]);
    // ROS_INFO_THROTTLE(0.5,"x: %f, y: %f, z: %f", body_accels[0], body_accels[1], body_accels[2]);
    v1_accel_vec = mat_rot*body_accels;
    // ROS_INFO_THROTTLE(0.5,"x: %f, y: %f, z: %f", v1_accel_vec[0], v1_accel_vec[1], v1_accel_vec[2]);

    odom_msg.twist.covariance[3] = v1_accel_vec[0];
    odom_msg.twist.covariance[4] = v1_accel_vec[1];
    odom_msg.twist.covariance[5] = v1_accel_vec[2];

    if(new_odom_){
      robot_odom_pub.publish(odom_msg);

      new_odom_ = false;
    }
    cmd_.header.stamp = odom_msg.header.stamp;
    cmd_vel_pub.publish(cmd_);
    // body_vels.linear.x = body_vels_vec[0];
    // body_vels.linear.y = body_vels_vec[1];
    // body_vels.linear.z = body_vels_vec[2];
    //
    // // odom_msg.twist.twist.linear.x = body_vels_vec[0];
    // // odom_msg.twist.twist.linear.y = body_vels_vec[1];
    // // odom_msg.twist.twist.linear.z = body_vels_vec[2];
    //
    // body_vels.angular.x = p;
    // body_vels.angular.y = q;
    // body_vels.angular.z = r;
    //
    // body_vel_pub.publish(body_vels);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
