#include "trajectory_sender/traj_sender.hpp"

namespace traj_sender {
trajectory_sender::trajectory_sender(const ros::NodeHandle &nh) {
    nh_ = nh;
    get_current_position();
    get_goals();
    send_goals();
    sub_sonar = nh_.subscribe<snav_msgs::Odometry>("/odometry/filtered", 1, &trajectory_sender::sub_callback, this);

}

trajectory_sender::~trajectory_sender(void) {}

void trajectory_sender::get_goals(void) {
    goals.resize(9);

// Fill in the first goal
    goals.at(0).trajectory.joint_names.resize(5);
    goals.at(0).trajectory.joint_names.at(0) = "x";
    goals.at(0).trajectory.joint_names.at(1) = "y";
    goals.at(0).trajectory.joint_names.at(2) = "z";
    goals.at(0).trajectory.joint_names.at(3) = "yaw";
    goals.at(0).trajectory.joint_names.at(4) = "pitch";

    // initial x position
    goals.at(0).trajectory.points.at(0).positions.push_back(-3.5);
    goals.at(0).trajectory.points.at(0).time_from_start = ros::Duration(20);

    // initial y position
    goals.at(0).trajectory.points.at(1).positions.push_back(-1.5);
    goals.at(0).trajectory.points.at(1).time_from_start = ros::Duration(20);

    // initial z position 
    goals.at(0).trajectory.points.at(2).positions.push_back(starting_point.at(2) - 0.5);
    goals.at(0).trajectory.points.at(2).time_from_start = ros::Duration(20);

    // yaw
    goals.at(0).trajectory.points.at(3).positions.push_back(0.0);
    goals.at(0).trajectory.points.at(3).time_from_start = ros::Duration(20);

    // pitch
    goals.at(0).trajectory.points.at(4).positions.push_back(0.0);
    goals.at(0).trajectory.points.at(4).time_from_start = ros::Duration(20);



    //second 0.1m forward

    goals.at(1) = goals.at(0);
    goals.at(1).trajectory.points.at(0).positions.at(0) += 0.1;

    // third - back to base
    goals.at(2) = goals.at(0);

    //fourth 0.1m left
    goals.at(3) = goals.at(0);
    goals.at(3).trajectory.points.at(1).positions.at(0) += 0.1;

    //fifth - back to base
    goals.at(4) = goals.at(0);

    //sixth - 0.1m down
    goals.at(5) = goals.at(0);
    goals.at(5).trajectory.points.at(2).positions.at(0) -= 0.1;

    //seventh - back to base
    goals.at(6) = goals.at(0);

    //eighth - move in a circle on the x,z plane
    goals.at(7) = goals.at(0);

    // prepare the 360 points trajectory for the valve turning - set 360 points to 0.0
    // on x
    goals.at(7).trajectory.points.at(0).positions.resize(360,0.0);
    // on y
    goals.at(7).trajectory.points.at(1).positions.resize(360,0.0);
    // on z
    goals.at(7).trajectory.points.at(2).positions.resize(360,0.0);
    // on yaw
    goals.at(7).trajectory.points.at(3).positions.resize(360,0.0);
    // on pitch
    goals.at(7).trajectory.points.at(4).positions.resize(360,0.0);


    // set the time allowed for this
    double circle_time = 60;
    goals.at(7).trajectory.points.at(0).time_from_start = ros::Duration(circle_time);
    goals.at(7).trajectory.points.at(1).time_from_start = ros::Duration(circle_time);
    goals.at(7).trajectory.points.at(2).time_from_start = ros::Duration(circle_time);
    goals.at(7).trajectory.points.at(3).time_from_start = ros::Duration(circle_time);
    goals.at(7).trajectory.points.at(4).time_from_start = ros::Duration(circle_time);

    //

    for (int x = 0; x < 360, x++) {
        // x = initial point x plus sin of x deg * radius
        goals.at(7).trajectory.points.at(0).positions.at(x) = goals.at(0).trajectory.points.at(0).positions.at(0) + sin((x * M_PI / 180) * 0.1 );
        // on y
        goals.at(7).trajectory.points.at(1).positions.at(x) =  goals.at(0).trajectory.points.at(1).positions.at(0);
        // on z
        goals.at(7).trajectory.points.at(2).positions.at(x) = goals.at(0).trajectory.points.at(2).positions.at(0) + cos((x * M_PI / 180) * 0.1 );
    }

    //ninth - back to base
    goals.at(8) = goals.at(0);
}

void trajectory_sender::send_goals(void) {
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("Controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for action server to start.");

    ac.waitForServer();

    ROS_INFO("Action server started, sending goal");

    // send all the goals one after the other
    for (std::std::vector<control_msgs::FollowJointTrajectoryAction>::Iterator it = goals.begin(), it != goals.end(), ++it) {

        ac.sendGoal(*it);

        bool finished_before_timeout = ac.waitForResult(*it.trajectory.points.at(0).time_from_start * 2) ;

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        } else {
            ROS_INFO("Action did not finish before the time out.");        
        }
    }
}

void trajectory_sender::sub_callback(const nav_msgs::Odometry::ConstPtr& message) {
    callback_message = *message;
} 

void trajectory_sender::get_current_position(void) {
    nav_msgs::Odometry temp_odometry = callback_message;

    geometry_msgs::PoseStamped pose_in;
    pose_in.header = temp_odometry.header;
    pose_in.pose = temp_odometry.pose.pose;

    geometry_msgs::PoseStamped pose_out;
    // find the subs pose in the pool frame and converts it into the feedback frame
    try {
        listener.waitForTransform(pose_in.header.frame_id, "/feedback", ros::Time(0), ros::Duration(20));
        listener.transformPose("/feedback", pose_in, pose_out);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("transform for controller feedback: %s\n", ex.what()); //Print exception which was caught
      return;
    }
    // We are in normal operation and want the positon to be in feedback frame and orientation in the pool frame
    // Otherwise our heading would be RPY(0,0,0) all the time
        pose_out.pose.orientation = pose_in.pose.orientation;

    // Get the data out of the odometry message
    extract_6_DOF(pose_out); 
}

void trajectory_sender::extract_6_DOF(const geometry_msgs::PoseStamped& stored_message) {
    starting_point.resize(6,0.0);
    starting_point.at(0) = stored_message.pose.position.x;
    starting_point.at(1) = stored_message.pose.position.y;
    starting_point.at(2) = stored_message.pose.position.z;  
    
    // get the Quaternion into 
    tf::Quaternion q(stored_message.pose.orientation.x, stored_message.pose.orientation.y, stored_message.pose.orientation.z, stored_message.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(starting_point.at(5), starting_point.at(4), starting_point.at(3));
}

} // end of namespace


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_sender");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // create the instance of the class
    traj_sender::trajectory_sender trajik(nh);

    ros::spin();
    ROS_INFO("trajectory_sender - Shutting down");
}