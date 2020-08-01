#include <nearness_control/trajectory_follower.h>

using namespace std;
namespace trajectory_follower{
trajectoryFollower::trajectoryFollower(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle){
      this->init();
  }

void trajectoryFollower::init() {

    //priv_nh.param("desired_altitude", ref_h_);
    //priv_nh.param("k_height", k_height_);
    //ROS_INFO("alt: %f, k: %f",  ref_h_, k_height_);

    sub_odom_ = nh_.subscribe("odometry", 1, &trajectoryFollower::odomCb, this);
    sub_traj_ = nh_.subscribe("trajectory", 1, &trajectoryFollower::trajCb, this);
    sub_task_ = nh_.subscribe("task", 1, &trajectoryFollower::taskCb, this);

    pub_lookahead_ = nh_.advertise<geometry_msgs::PointStamped>("traj_lookahead", 10);
    last_lookahead_index_ = 0;
    lookahead_dist_short_ = 1.25;
    lookahead_dist_long_ = 1.5;
    enable_lookahead_lookup_ = false;
    have_current_traj_home_ = false;
    lookahead_point_.header.frame_id = "world";
}

void trajectoryFollower::odomCb(const nav_msgs::OdometryConstPtr& odom_msg)
{
    //ROS_INFO_THROTTLE(1,"Received odom");
    odom_ = *odom_msg;
    odom_point_ =  odom_.pose.pose.position;
    //ROS_INFO_THROTTLE(1,"x: %f, y: %f", odom_point_.x, odom_point_.y);
}

void trajectoryFollower::trajCb(const visualization_msgs::MarkerArrayConstPtr& msg)
{
    //ROS_INFO_THROTTLE(1,"Received traj");
    // e only need to do this when we can't plan home
    if(enable_lookahead_lookup_ && !have_current_traj_home_){
        uint32_t traj_list_size_ = msg->markers[1].points.size();
        if(traj_list_size_ > 2){
            traj_list_points_.clear();
            ROS_INFO_THROTTLE(1, "traj_list_size: %d", traj_list_size_);
            last_lookahead_index_ = traj_list_size_-2;
            // Import trajectory list
            for (int i = 0; i < traj_list_size_; i++){
                traj_list_points_.push_back(msg->markers[1].points[i]);
                //traj_list_points_[i].x = msg[i].pose.position.x;
                //traj_list_points_[i].y = msg[i].pose.position.y;
                //traj_list_points_[i].z = msg[i].pose.position.z;
                //ROS_INFO("i: %d, x: %f, y: %f", i, traj_list_points_[i].x, traj_list_points_[i].y);
            }
            //ROS_INFO("%d", last_lookahead_index_);
            have_current_traj_home_ = true;
          }
    }

}

void trajectoryFollower::findNextLookahead(){
    // Parse through the list for the next lookahead
    ROS_INFO_THROTTLE(5,"last_lookahead_index: %d", last_lookahead_index_);
    if(have_current_traj_home_ && (last_lookahead_index_ != 0)){
        // while(traj_list_points_[last_lookahead_index_].x < .01){
        //     last_lookahead_index_ -= 1;
        // }
        ROS_INFO_THROTTLE(5,"last_lookahead_index #2: %d", last_lookahead_index_);
        for (int i = last_lookahead_index_; i > 0; i--){
            float dist_err = dist(odom_point_, traj_list_points_[i]);
          //  ROS_INFO("%f, %d", dist_err, i);
            if((dist_err > lookahead_dist_short_) && (dist_err < lookahead_dist_long_)){
                lookahead_point_.point = traj_list_points_[i];
                last_lookahead_index_ = i + 1;
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, lookahead_point_.point.x, lookahead_point_.point.y);
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, traj_list_points_[i].x, traj_list_points_[i].y);
                //break;
            }
            // if(i==1){
            //     //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", 1, traj_list_points_[1].x, traj_list_points_[1].y);
            //     //lookahead_point_.point = traj_list_points_[1];
            // }
        }
        ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, traj_list_points_[last_lookahead_index_].x, traj_list_points_[last_lookahead_index_].y);
    } else {
        lookahead_point_.point = odom_point_;
    }
}

void trajectoryFollower::publishLookahead(){
    ROS_INFO_THROTTLE(2,"Publishing trajectory lookahead...");
    findNextLookahead();
    lookahead_point_.header.stamp = ros::Time::now();
    pub_lookahead_.publish(lookahead_point_);
}

void trajectoryFollower::taskCb(const std_msgs::String task_msg)
{
    // Check if we are unable to plan
    //ROS_INFO("%s",task_msg.data.c_str());
    std::string unable_to_plan_home_str("Unable to plan home");
    std::string unable_to_plan_str("Unable to plan");
    //if((task_msg.data.c_str() == "Unable to plan home") || (task_msg.data.c_str() == "Unable to plan")){
    if((unable_to_plan_home_str.compare(task_msg.data.c_str()) == 0)||(unable_to_plan_str.compare(task_msg.data.c_str())== 0)){
        enable_lookahead_lookup_ = true;
        //ROS_INFO("Unable to plan home!!!!!");
    } else {
        enable_lookahead_lookup_ = false;
        last_lookahead_index_ = 0;
        have_current_traj_home_ = false;
    }

}


float trajectoryFollower::dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    float distance = sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2));
    return distance;
}

bool trajectoryFollower::doLookup(){
    return enable_lookahead_lookup_;
}



 // end of class
} // End of namespace nearness
