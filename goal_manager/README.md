# goal_manager

This package publishes goals randomly for move_base.
Robots will run randomly with autonomous avoidance and navigation by move_base.

## Published topics
- "/initialpose": geometry_msgs/PoseWithCovarianceStamped <br/>
    initial pose to re-localization for amcl.
- "/move_base_simple/goal": geometry_msgs/PoseStamped <br />
    goal pose for navigation.

## Subscribed topics
- "/move_base/status": actionlib_msgs/GoalStatusArray <br/>
    goal status from move_base. This is used to judge if a new goal is needed.

## Launcher
Launch files are in "goal_manager/launch" directory. <br/>
Two launch files are provided.
- goal_manager.launch <br/>
   It launches only goal manager. You have to launch move_base in the advance.
- nav_random.launch <br/>
   It launches all of the necessary nodes to make turtlebot3 run in gazebo.

You can use them by next command after build the packages;
```bash
roslaunch goal_manager <launch_file_name>
```

## Parameters
 - "map_frame": frame_id for initial pose and goal pose.
 - "timeout_s": timeout to give up to reach published goal. New goal will published after timeout elapsed.
 - "interval_s": interval time to check the goal status.
 - "initial_pose_x_m": x of initial position for localization by amcl. This should be same as the position in Gazebo.
 - "initial_pose_y_m": y of initial position for localization by amcl. This should be same as the position in Gazebo.
 - "initial_pose_w_deg": direction of initial pose for localization by amcl. This should be same as the pose in Gazebo.
 - "target_center_x_m": goals will be set along a circle with center position of x set by this parameter.
 - "target_center_y_m": goals will be set along a circle with center position of y set by this parameter.
 - "target_radius_m": goals will be set along a circle with radius set by this parameter.

