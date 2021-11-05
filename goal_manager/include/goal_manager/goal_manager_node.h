/**
 * goal_manager.hpp
 * Copyright (C) Taishi Ueda 2021  <taichi.ueda@gmal.com>
 *
 * @brief Simple manager to publish goal for move_base.
 */

#ifndef GOAL_MANAGER_HPP
#define GOAL_MANAGER_HPP
#include <ros/ros.h>

class GoalManager{
    public:
    //! Constructor
    GoalManager();
    //! Constructor
    ~GoalManager();
    //! Initializer
    bool init();
    private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Publisher pub_;
    void timerCallback(const ros::TimerEvent& e);
};

#endif /* !GOAL_MANAGER_HPP */
