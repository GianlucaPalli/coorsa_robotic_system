
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <coorsa_box_detector_msgs/DetectBoxesAction.h>

typedef actionlib::SimpleActionServer<coorsa_box_detector_msgs::DetectBoxesAction> Server;

void execute(const coorsa_box_detector_msgs::DetectBoxesGoalConstPtr& goal, Server* as)  
{
  coorsa_box_detector_msgs::DetectBoxesResult result;
  std::vector<geometry_msgs::Pose> pose_vector;
  geometry_msgs::Pose box_pose;
  box_pose.position.x =0.99;
  box_pose.position.y = 0.12;
  box_pose.position.z = 0.33;
  box_pose.orientation.x = -0.0083;
  box_pose.orientation.y = -0.0130;
  box_pose.orientation.z = -0.363;
  box_pose.orientation.w = 0.9312;
  pose_vector.push_back(box_pose);
  box_pose.position.x =1.16;
  box_pose.position.y = -0.09;
  box_pose.position.z = -0.038;
  box_pose.orientation.x = -0.0083;
  box_pose.orientation.y = -0.0130;
  box_pose.orientation.z = -0.363;
  box_pose.orientation.w = 0.9312;
  pose_vector.push_back(box_pose);
  box_pose.position.x =0.594;
  box_pose.position.y = -0.26;
  box_pose.position.z = 0.34;
  box_pose.orientation.x = -0.0083;
  box_pose.orientation.y = -0.0130;
  box_pose.orientation.z = -0.363;
  box_pose.orientation.w = 0.9312;
  pose_vector.push_back(box_pose);
  box_pose.position.x =0.77;
  box_pose.position.y = -0.11;
  box_pose.position.z = 0.33;
  box_pose.orientation.x = -0.0083;
  box_pose.orientation.y = -0.0130;
  box_pose.orientation.z = -0.363;
  box_pose.orientation.w = 0.9312;
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

/*
  box_pose.position.x =0.495;
  box_pose.position.y = 0.13;
  box_pose.position.z = 0.3;
  pose_vector.push_back(box_pose);
  box_pose.position.x =0.495;
  box_pose.position.y = -0.16;
  box_pose.position.z = 0.3;
  pose_vector.push_back(box_pose);
  box_pose.position.x =0.735;
  box_pose.position.y = 0.13;
  box_pose.position.z = 0.3;
  pose_vector.push_back(box_pose);
  box_pose.position.x =0.735;
  box_pose.position.y = -0.16;
  box_pose.position.z = 0.3;
  pose_vector.push_back(box_pose);*/
/*
    box_pose.position.x =0.6;
  box_pose.position.y = 0.13;
  box_pose.position.z = 0.3;
  pose_vector.push_back(box_pose);
  box_pose.position.x =0.6;
  box_pose.position.y = -0.16;
  box_pose.position.z = 0.3;
  pose_vector.push_back(box_pose);*/