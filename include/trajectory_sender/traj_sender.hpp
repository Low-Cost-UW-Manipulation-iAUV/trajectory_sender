#ifndef __ROS_CONTROL_TRAJECTORY__
#define __ROS_CONTROL_TRAJECTORY__

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"

namespace traj_sender {
class trajectory_sender {

public:
    trajectory_sender(const ros::NodeHandle &);
    ~trajectory_sender();
private:
    ros::NodeHandle nh_;
    std::vector<control_msgs::FollowJointTrajectoryGoal> goals;
    tf::TransformListener listener;
    tf::StampedTransform found_transform;
    ros::Subscriber sub_sonar;
    nav_msgs::Odometry callback_message;
    std::vector<double> starting_point;
    void get_goals(void);
    void send_goals(void);
    void get_current_position(void);

    void sub_callback(const nav_msgs::Odometry::ConstPtr&);
    void extract_6_DOF(const geometry_msgs::PoseStamped&);


};

} // end of namespace

#endif