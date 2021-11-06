/**
 * goal_manager.hpp
 * Copyright (C) Taishi Ueda 2021  <taichi.ueda@gmal.com>
 *
 * @brief Simple manager to publish goal for move_base.
 */

#ifndef GOAL_MANAGER_HPP
#define GOAL_MANAGER_HPP
#include <stdint.h>
#include <random>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>

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
    ros::Publisher pub_init_pose_;
    ros::Publisher pub_target_;
    ros::Subscriber sub_status_;

    //! for generating randomized target pose.
    std::unique_ptr<std::uniform_real_distribution<double>> unif_ptr_;
    std::default_random_engine rnd_engine_;

    double timeout_s_{30.0};
    double wait_after_new_target_s_{10.0};
    double init_w_deg_{0.0};
    double target_center_x_m_{0.0};
    double target_center_y_m_{0.0};
    double target_radius_m_{2.0};
    double last_published_s_{0.0};
    uint8_t nav_status_{0u};
    bool is_first_{true};
    geometry_msgs::PoseWithCovarianceStamped init_pose_;

    /**
     * generate target pose for mobe_base.
     * \param[out] target geterated target.
     *
     * a target is generated on a circle of radius set by target_radius_m_,
     *     and center point set by target_center_x_m_ and target_center_y_m_.
     */
    void generateTarget(geometry_msgs::PoseStamped& target);

    //! publish initial pose and target pose continuously according to goal status.
    void timerCallback(const ros::TimerEvent& e);

    //! update goal status.
    void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& status);

    void publishTarget();
};

#endif /* !GOAL_MANAGER_HPP */
