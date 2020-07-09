#include <nearness_control/trajectory_follower.h>

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
    geometry_msgs::PointStamped
    sub_odom_ = nh_.subscribe("odometry", 1, &trajectoryFollower::odomCb, this);
    sub_traj_ = nh_.subscribe("trajectory", 1, &trajectoryFollower::trajCb, this);
    sub_task_ = nh_.subscribe("task", 1, &trajectoryFollower::taskCb, this);

    pub_lookahead_ = nh_.advertise<geometry_msgs::PointStamped>("traj_lookahead", 10);
    last_lookahead_index_ = 0;
}

void trajectoryFollower::odomCb(const nav_msgs::OdometryConstPtr& odom_msg)
{
    odom_ = *odom_msg;
    odom_point_ =  odom_.pose.pose.position;
}

void trajectoryFollower::trajCb(const vector<geometry_msgs::PoseStamped>& msg)
{
    // We only need to do this when we can't plan home
    if(enable_lookahead_lookup_ && !have_current_traj_home_){
        traj_list_points_.clear();
        uint32_t traj_list_size = msg.size();

        // Import trajectory list
        for (int i = 0; i < traj_list_size; i++){
            traj_list_points_.push_back(msg[i].pose.position);
            //traj_list_points_[i].x = msg[i].pose.position.x;
            //traj_list_points_[i].y = msg[i].pose.position.y;
            //traj_list_points_[i].z = msg[i].pose.position.z;
        }
        have_current_traj_home_ = true;
    }

}

void trajectoryFollower::findNextLookahead(){
    // Parse through the list for the next lookahead
    for (int i = last_lookahead_index_; i < msg.size() - last_lookahead_index_; i++){
        if(dist(odom_point_, traj_list_points[i]) > lookahead_dist_){
            lookahead_point_ = traj_list_points[i];
            last_lookahead_index_ = i;
            break;
        }
    }
}

void trajectoryFollower::publishLookahead(){
    findNextLookahead();
    pub_lookahead_.publish(lookahead_point_);
}

void trajectoryFollower::taskCb(const std_msgs::String task_msg)
{
    // Check if we are unable to plan
    if((task_msg.task.c_str() == "Unable to plan home") || (task_msg.task.c_str() == "Unable to plan")){
        enable_lookahead_lookup_ = true;
    } else {
        enable_lookahead_lookup_ = false;
        last_lookahead_index_ = 0;
        have_current_traj_home_ = false;
    }

}


float trajectoryFollower::dist(const geometry_msgs::Point p1, const geomtry_msgs::Point p2)
{
    float distance = math.sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2));
    return distance;
}

bool trajectoryFollower::doLookup(){
    return enable_lookahead_lookup_;
}



 // end of class
} // End of namespace nearness
