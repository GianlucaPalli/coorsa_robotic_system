
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <coorsa_interface/DetectBoxesAction.h>

typedef actionlib::SimpleActionServer<coorsa_interface::DetectBoxesAction> Server;

void execute(const coorsa_interface::DetectBoxesGoalConstPtr& goal, Server* as)  
{
  coorsa_interface::DetectBoxesResult result;
  std::vector<geometry_msgs::Pose> pose_vector;
  geometry_msgs::Pose box_pose;
  box_pose.position.x =0.5;
  box_pose.position.y = 0;
  box_pose.position.z = 0.3;
  pose_vector.push_back(box_pose);
  result.box_poses = pose_vector;
  as->setSucceeded(result);
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "fake_box_detector");
    ros::NodeHandle n;
    Server server(n, "package_detector", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}