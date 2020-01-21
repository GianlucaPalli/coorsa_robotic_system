#include <ros/ros.h>



int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "task_control");
    ros::NodeHandle n;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("follow_joint_trajectory", true);
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    control_msgs::FollowJointTrajectoryActionGoal goal;
    trajectory_msgs::JointTrajectory traj;


