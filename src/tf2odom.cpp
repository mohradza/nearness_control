#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

using namespace Eigen;

bool new_cmd_;
geometry_msgs::Twist cmd_;
void cmdCallback(const geometry_msgs::TwistConstPtr& cmd_msg){
     new_cmd_ = true;
     cmd_ = *cmd_msg;
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
    node.advertise<geometry_msgs::Twist>("cmd_vel/ground_truth", 10);

  ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10, cmdCallback);

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
  float alpha_vel = .95;

  float last_roll = 0.0;
  float last_pitch = 0.0;
  float last_yaw = 0.0;

  float dt = 0.0;
  float diff = 0.0;

  geometry_msgs::Twist body_vels;

  ros::Time last_odom_time = ros::Time::now();

  ros::Rate rate(25);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/world", "/OHRAD_X3",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
    }
    diff = abs(last_x_pos - transform.getOrigin().x());
    //ROS_INFO_THROTTLE(1.0,"diff: %f", diff);
    ROS_INFO_THROTTLE(1.0,"Publishing odometry...");
    // if( diff > .0001){
    odom_msg.header.stamp = ros::Time::now();
    dt = (odom_msg.header.stamp - last_odom_time).toSec();
    last_odom_time = odom_msg.header.stamp;

    odom_msg.pose.pose.position.x = transform.getOrigin().x();
    odom_msg.pose.pose.position.y = transform.getOrigin().y();
    odom_msg.pose.pose.position.z = transform.getOrigin().z();

    odom_msg.twist.twist.linear.x = (odom_msg.pose.pose.position.x - last_x_pos)/dt;
    odom_msg.twist.twist.linear.y = (odom_msg.pose.pose.position.y - last_y_pos)/dt;
    odom_msg.twist.twist.linear.z = (odom_msg.pose.pose.position.z - last_z_pos)/dt;

    last_x_pos = odom_msg.pose.pose.position.x;
    last_y_pos = odom_msg.pose.pose.position.y;
    last_z_pos = odom_msg.pose.pose.position.z;

    tf::quaternionTFToMsg(transform.getRotation().normalize(), odom_msg.pose.pose.orientation);
    tf::Quaternion quat(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    double p, q, r;
    double roll_dot, pitch_dot, yaw_dot;
    m.getRPY(roll, pitch, yaw);

    if (roll > M_PI/2){
      roll -= M_PI;
    } else if( roll < -M_PI/2){
      roll += M_PI;
    }

    roll_dot = (roll - last_roll)/dt;
    pitch_dot = (pitch - last_pitch)/dt;
    yaw_dot = (yaw - last_yaw)/dt;
    p = roll_dot - sin(pitch)*yaw_dot;
    q = cos(roll)*pitch_dot + sin(roll)*cos(pitch)*yaw_dot;
    r = -sin(roll)*pitch_dot + cos(roll)*cos(pitch)*yaw_dot;

    last_roll = roll;
    last_pitch = pitch;
    last_yaw = yaw;

    odom_msg.twist.twist.angular.x = p;
    odom_msg.twist.twist.angular.y = q;
    odom_msg.twist.twist.angular.z = r;

    robot_odom_pub.publish(odom_msg);
    cmd_vel_pub.publish(cmd_);

    // body_vels.linear.x = ;
    // body_vels.linear.y = ;
    // body_vels.linear.z = ;

    Vector3f world_vels(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);
    Matrix3f mat_yaw(3,3), mat_pitch(3,3), mat_roll(3,3), mat_rot(3,3);

    mat_yaw << cos(yaw), sin(yaw), 0.0, -sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;
    mat_pitch << cos(pitch), 0.0, -sin(pitch), 0.0, 1.0, 0.0, sin(pitch), 0.0, cos(pitch);
    mat_roll << 1.0, 0.0, 0.0, 0.0, cos(roll), sin(roll), 0.0, -sin(roll), cos(roll);

    mat_rot = mat_yaw*mat_pitch*mat_roll;

    Vector3f body_vels_vec(0.0, 0.0, 0.0);
    body_vels_vec = mat_rot*world_vels;

    body_vels.linear.x = body_vels_vec[0];
    body_vels.linear.y = body_vels_vec[1];
    body_vels.linear.z = body_vels_vec[2];

    odom_msg.twist.twist.linear.x = body_vels_vec[0];
    odom_msg.twist.twist.linear.y = body_vels_vec[1];
    odom_msg.twist.twist.linear.z = body_vels_vec[2];

    body_vels.angular.x = p;
    body_vels.angular.y = q;
    body_vels.angular.z = r;

    body_vel_pub.publish(body_vels);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
