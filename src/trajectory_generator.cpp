#include <nearness_control/trajectory_generator.h>

using namespace std;
namespace trajectory_generator{
trajectoryGenerator::trajectoryGenerator(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle){
      this->init();
  }

void trajectoryGenerator::init() {

    sub_odom_ = nh_.subscribe("odometry_map", 1, &trajectoryGenerator::odomCb, this);

    pub_traj_ = nh_.advertise<nearness_control_msgs::TrajList>("ground_truth_trajectory", 10);

    pnh_.param("trajectory_point_distance_thresh", traj_point_dist_thresh_, 0.1);

    traj_point_dist_thresh_ = .1;
    initialized_ = false;
    count_ = 0;

}

void trajectoryGenerator::odomCb(const nav_msgs::OdometryConstPtr& odom_msg)
{
    ROS_INFO_THROTTLE(1,"Received odom");
    odom_ = *odom_msg;
    odom_point_ =  odom_.pose.pose.position;

    if(!initialized_){
      traj_list_points_.push_back(odom_point_);
      initialized_ = true;
    }

    //ROS_INFO_THROTTLE(1,"x: %f, y: %f", odom_point_.x, odom_point_.y);
    float distance = dist(odom_point_, traj_list_points_[count_]);
    if(distance > traj_point_dist_thresh_){
      traj_list_points_.push_back(odom_point_);
      count_++;
    }

}

float trajectoryGenerator::dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    float distance = sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2));
    return distance;
}

void trajectoryGenerator::publishTrajectory()
{
    traj_list_msg_.traj_points.clear();
    traj_list_msg_.traj_list_size = count_;
    traj_list_msg_.traj_points = traj_list_points_;
    pub_traj_.publish(traj_list_msg_);
}



 // end of class
} // End of namespace nearness
