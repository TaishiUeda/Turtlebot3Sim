/*
 * goal_manager.cpp
 * Copyright (C) Taishi Ueda 2021  <taichi.ueda@gmal.com>
 *
 */

#include <nav_msgs/MoveBaseAction.h>
#include "goal_manager/goal_manager_node.h"

GoalManager::GoalManager(){
    bool is_ok = init();
    ROS_ASSERT(is_ok);
}

GoalManager::~GoalManager(){
}

bool GoalManager::init(){
    timer_ = nh_.createTimer(
	    ros::Duration(0.1),
	    &GoalManager::timerCallback,
	    this);
    return true;
}

void GoalManager::timerCallback(const ros::TimerEvent& e){
    printf("callbacked.\n");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "goal_manager");
    GoalManager goal_manager;
    ros::spin();
    return 0;
}
