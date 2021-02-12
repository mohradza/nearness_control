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

    sub_odom_ = nh_.subscribe("odometry_map", 1, &trajectoryFollower::odomCb, this);
    sub_carto_traj_ = nh_.subscribe("cartographer_trajectory", 1, &trajectoryFollower::cartoTrajCb, this);
    sub_liosam_traj_ = nh_.subscribe("liosam_trajectory", 1, &trajectoryFollower::liosamTrajCb, this);
    sub_gt_traj_ = nh_.subscribe("ground_truth_trajectory", 1, &trajectoryFollower::gtTrajCb, this);
    //sub_task_ = nh_.subscribe("task", 1, &trajectoryFollower::taskCb, this);
    sub_follow_traj_ = nh_.subscribe("follow_traj", 1, &trajectoryFollower::followTrajCb, this);

    pub_lookahead_ = nh_.advertise<geometry_msgs::PointStamped>("traj_lookahead", 10);

    std::string vehicle_name;
    pnh_.param<std::string>("vehicle_name", vehicle_name,"X1");
    pnh_.param("enable_ground_truth", enable_ground_truth_, false);

    last_lookahead_index_ = 0;
    lookahead_dist_short_ = .5;
    lookahead_dist_long_ = 2.0;
    enable_lookahead_lookup_ = false;
    have_current_traj_home_ = false;
    have_current_gt_traj_home_ = false;
    last_gt_lookahead_index_ = 0;

    string lookahead_frame = vehicle_name + "/map";
    lookahead_point_.header.frame_id = lookahead_frame;

}

void trajectoryFollower::odomCb(const nav_msgs::OdometryConstPtr& odom_msg)
{
    //ROS_INFO_THROTTLE(1,"Received odom");
    odom_ = *odom_msg;
    odom_point_ =  odom_.pose.pose.position;
    ROS_INFO_THROTTLE(1,"x: %f, y: %f", odom_point_.x, odom_point_.y);
}

void trajectoryFollower::cartoTrajCb(const lcd_pkg::PoseGraphConstPtr& msg)
{
    //ROS_INFO_THROTTLE(1,"Received traj");
    // e only need to do this when we can't plan home
    uint32_t traj_list_size_ = msg->poseArray.size();

    //ROS_INFO_THROTTLE(1, "Traj list size: %d,", traj_list_size_);
    if(enable_lookahead_lookup_ && !have_current_traj_home_){
        traj_list_points_.clear();
        //ROS_INFO_THROTTLE(1, "traj_list_size: %d", traj_list_size_);
        last_lookahead_index_ = traj_list_size_-1;
        // Import trajectory list
        for (int i = 0; i < traj_list_size_; i++){
            traj_list_points_.push_back(msg->poseArray[i].pose.position);
        }
        //ROS_INFO("%d", last_lookahead_index_);
        have_current_traj_home_ = true;
      }
}

void trajectoryFollower::liosamTrajCb(const nav_msgs::PathConstPtr& msg)
{
    ROS_INFO_THROTTLE(1,"Received traj");
    // e only need to do this when we can't plan home
    uint32_t traj_list_size_ = msg->poses.size();

    //ROS_INFO_THROTTLE(1, "Traj list size: %d,", traj_list_size_);
    if(enable_lookahead_lookup_ && !have_current_traj_home_){
        traj_list_points_.clear();
        //ROS_INFO_THROTTLE(1, "traj_list_size: %d", traj_list_size_);
        last_lookahead_index_ = traj_list_size_-1;
        // Import trajectory list
        for (int i = 0; i < traj_list_size_; i++){
            traj_list_points_.push_back(msg->poses[i].pose.position);
        }
        ROS_INFO("%d", last_lookahead_index_);
        have_current_traj_home_ = true;
      }
}

void trajectoryFollower::gtTrajCb(const nearness_control_msgs::TrajListConstPtr& msg)
{
    ROS_INFO_THROTTLE(1,"Received gt traj");
    // e only need to do this when we can't plan home
    uint32_t gt_traj_list_size_ = msg->traj_list_size;

    //ROS_INFO_THROTTLE(1, "Traj list size: %d,", traj_list_size_);
    if(enable_lookahead_lookup_ && !have_current_gt_traj_home_){
        traj_list_points_.clear();
        //ROS_INFO_THROTTLE(1, "traj_list_size: %d", traj_list_size_);
        last_gt_lookahead_index_ = gt_traj_list_size_-1;
        // Import trajectory list

        gt_traj_list_points_ = msg->traj_points;

        //ROS_INFO("%d", last_lookahead_index_);
        have_current_gt_traj_home_ = true;
      }
}

void trajectoryFollower::findNextLookahead(){
    // Parse through the list for the next lookahead
    ROS_INFO_THROTTLE(1,"last_lookahead_index: %d", last_lookahead_index_);
    if(have_current_traj_home_ && (last_lookahead_index_ != 0)){
        //ROS_INFO_THROTTLE(1,"last_lookahead_index #2: %d", last_lookahead_index_);
        for (int i = last_lookahead_index_; i > 0; i--){
            float dist_err = dist(odom_point_, traj_list_points_[i]);
            //sROS_INFO("%f, %d", dist_err, i);
            if((dist_err > lookahead_dist_short_) && (dist_err < lookahead_dist_long_)){
                lookahead_point_.point = traj_list_points_[i];
                last_lookahead_index_ = i + 1;
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, lookahead_point_.point.x, lookahead_point_.point.y);
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, traj_list_points_[i].x, traj_list_points_[i].y);
                //break;
            }

        }
        //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, traj_list_points_[last_lookahead_index_].x, traj_list_points_[last_lookahead_index_].y);
    } else {
        lookahead_point_.point = odom_point_;
    }
}

void trajectoryFollower::findNextGTLookahead(){
    // Parse through the list for the next lookahead
    //ROS_INFO_THROTTLE(1,"last_gt_lookahead_index: %d", last_gt_lookahead_index_);
    if(have_current_gt_traj_home_ && (last_gt_lookahead_index_ != 0)){
        //ROS_INFO('Test1');
        //ROS_INFO_THROTTLE(1,"last_lookahead_index #2: %d", last_lookahead_index_);
        for (int i = last_gt_lookahead_index_; i > 0; i--){
            //ROS_INFO('Test2');

            float dist_err = dist(odom_point_, gt_traj_list_points_[i]);
            //ROS_INFO('Test3');

          //  ROS_INFO("%f, %d", dist_err, i);
            if((dist_err > lookahead_dist_short_) && (dist_err < lookahead_dist_long_)){
                lookahead_point_.point = gt_traj_list_points_[i];
                last_gt_lookahead_index_ = i + 1;
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, lookahead_point_.point.x, lookahead_point_.point.y);
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, traj_list_points_[i].x, traj_list_points_[i].y);
                //break;
            }

        }
        ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_gt_lookahead_index_, gt_traj_list_points_[last_gt_lookahead_index_].x, gt_traj_list_points_[last_gt_lookahead_index_].y);
    } else {
        lookahead_point_.point = odom_point_;
    }
}

void trajectoryFollower::publishLookahead(){
    ROS_INFO_THROTTLE(2,"Publishing trajectory lookahead...");
    if(enable_ground_truth_){
      findNextGTLookahead();
    } else {
      findNextLookahead();
    }
    lookahead_point_.header.stamp = ros::Time::now();
    pub_lookahead_.publish(lookahead_point_);
}

// void trajectoryFollower::taskCb(const std_msgs::String task_msg)
// {
//     // Check if we are unable to plan
//     //ROS_INFO("%s",task_msg.data.c_str());
//     std::string unable_to_plan_home_str("Unable to plan home");
//     std::string unable_to_plan_str("Unable to plan");
//     //if((task_msg.data.c_str() == "Unable to plan home") || (task_msg.data.c_str() == "Unable to plan")){
//     if((unable_to_plan_home_str.compare(task_msg.data.c_str()) == 0)||(unable_to_plan_str.compare(task_msg.data.c_str())== 0)){
//         enable_lookahead_lookup_ = true;
//         //ROS_INFO("Unable to plan home!!!!!");
//     } else {
//         enable_lookahead_lookup_ = false;
//         last_lookahead_index_ = 0;
//         have_current_traj_home_ = false;
//     }
//
// }

void trajectoryFollower::followTrajCb(const std_msgs::BoolConstPtr& follow_traj_msg){
    ROS_INFO("Follow traj cb");
    if(follow_traj_msg->data){
        enable_lookahead_lookup_ = true;
    } else {
      enable_lookahead_lookup_ = false;
      last_lookahead_index_ = 0;
      have_current_traj_home_ = false;
      have_current_gt_traj_home_ = false;
      last_gt_lookahead_index_ = 0;
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
