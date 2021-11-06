/*
 * goal_manager.cpp
 * Copyright (C) Taishi Ueda 2021  <taichi.ueda@gmal.com>
 *
 */

#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include "goal_manager/goal_manager_node.h"

GoalManager::GoalManager(){
    unif_ptr_.reset(new std::uniform_real_distribution<double>(-M_PI, M_PI));
    bool is_ok = init();
    ROS_ASSERT(is_ok);
}

GoalManager::~GoalManager(){
}

bool GoalManager::init(){

    //--- create initial pose ---
    // header
    init_pose_.header.seq = 0;
    nh_.param("map_frame", init_pose_.header.frame_id, std::string("map"));
    // timeout
    nh_.param("timeout_s", timeout_s_, 30.0);
    // position
    nh_.param("initial_pose_x_m", init_pose_.pose.pose.position.x, -2.0);
    nh_.param("initial_pose_y_m", init_pose_.pose.pose.position.y, -0.5);
    init_pose_.pose.pose.position.z = 0.0;
    // orientation
    init_w_deg_ = 0.0;
    nh_.param("initial_pose_w_deg", init_w_deg_, 0.0);
    init_pose_.pose.pose.orientation.x = 0.0;
    init_pose_.pose.pose.orientation.y = 0.0;
    init_pose_.pose.pose.orientation.z = sin(init_w_deg_/180.0*M_PI/2.0);
    init_pose_.pose.pose.orientation.w = cos(init_w_deg_/180.0*M_PI/2.0);
    // covariance. same as values published by rviz.
    init_pose_.pose.covariance = {
	0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853};

    // parameters for generateTarget.
    nh_.param("target_center_x_m", target_center_x_m_, 0.0);
    nh_.param("target_center_y_m", target_center_y_m_, 0.0);
    nh_.param("target_radius_m", target_radius_m_, 2.0);

    // create instants
    pub_init_pose_  = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
	    "/initialpose", 10);
    pub_target_  = nh_.advertise<geometry_msgs::PoseStamped>(
	    "/move_base_simple/goal", 10);
    sub_status_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>(
	    "/move_base/status", 10, &GoalManager::navStatusCallBack, this);

    // start timer
    double interval_s = 0.0;
    nh_.param("interval_s", interval_s, 5.0);
    timer_ = nh_.createTimer(
	    ros::Duration(interval_s),
	    &GoalManager::timerCallback,
	    this);

    return true;
}

void GoalManager::generateTarget(geometry_msgs::PoseStamped& target){
    target.header.seq = 0;
    target.header.stamp.sec = ros::Time::now().sec;
    target.header.stamp.nsec = ros::Time::now().nsec;
    target.header.frame_id = init_pose_.header.frame_id;
    double theta_rad = (*unif_ptr_)(rnd_engine_);
    target.pose.position.x = target_center_x_m_ + target_radius_m_*cos(theta_rad);
    target.pose.position.y = target_center_y_m_ + target_radius_m_*sin(theta_rad);
    target.pose.position.z = 0.0;
    target.pose.orientation.x = 0.0;
    target.pose.orientation.y = 0.0;
    target.pose.orientation.z = sin(theta_rad/2.0);
    target.pose.orientation.w = cos(theta_rad/2.0);
}

void GoalManager::timerCallback(const ros::TimerEvent& e){

    // set initial pose of amcl
    if (is_first_){
	init_pose_.header.stamp.sec = ros::Time::now().sec;
	init_pose_.header.stamp.nsec = ros::Time::now().nsec;
	pub_init_pose_.publish(init_pose_);
        ++init_pose_.header.seq;
	ROS_INFO("Initial pose published.");
        ROS_INFO("[x, y, w] = [%lf, %lf, %lf]\n",
		init_pose_.pose.pose.position.x,
                init_pose_.pose.pose.position.y,
		init_w_deg_);
	publishTarget();
    }

    double duration_s = ros::Time().now().sec - last_published_s_;
    if ((nav_status_ == actionlib_msgs::GoalStatus::SUCCEEDED |
         nav_status_ == actionlib_msgs::GoalStatus::PENDING |
         duration_s > timeout_s_) && (duration_s > wait_after_new_target_s_)){
	publishTarget();
    }
}

void GoalManager::navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& status){
    if (status->status_list.size() > 0){
        nav_status_= status->status_list[0].status;
	is_first_ = false;
    }
}

void GoalManager::publishTarget(){
    geometry_msgs::PoseStamped target;
    generateTarget(target);
    pub_target_.publish(target);
    ROS_INFO("Target pose published.");
    ROS_INFO("[x, y, w] = [%lf, %lf, %lf]\n",
    	target.pose.position.x,
            target.pose.position.y,
    	acos(target.pose.orientation.w)*2/M_PI*180.0);
    last_published_s_ = ros::Time().now().sec;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "goal_manager");
    GoalManager goal_manager;
    ros::spin();
    return 0;
}
