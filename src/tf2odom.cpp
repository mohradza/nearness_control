#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_odom_publisher");

  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Publisher robot_odom_pub =
    node.advertise<nav_msgs::Odometry>("odometry/ground_truth", 10);

  tf::TransformListener listener;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped tf_msg;
  geometry_msgs::Quaternion quat_msg;

  geometry_msgs::PointStamped point_msg;
  std::string frame_name;
  nh.param<std::string>("OHRAD_X3", frame_name, "default_value");
  odom_msg.header.frame_id = "world";
  ros::Rate rate(20);
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
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose.pose.position.x = transform.getOrigin().x();
    odom_msg.pose.pose.position.y = transform.getOrigin().y();
    odom_msg.pose.pose.position.z = transform.getOrigin().z();
    tf::quaternionTFToMsg(transform.getRotation().normalize(), odom_msg.pose.pose.orientation);
    robot_odom_pub.publish(odom_msg);
/*
    point_msg.header.stamp = ros::Time::now();
    point_msg.point.x = odom_msg.pose.pose.position.x;
    point_msg.point.y = odom_msg.pose.pose.position.y;
    point_msg.point.z = odom_msg.pose.pose.position.z;

    robot_point_pub.publish(point_msg);
*/
    rate.sleep();
  }
  return 0;
};
