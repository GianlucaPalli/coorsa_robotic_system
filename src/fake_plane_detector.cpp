
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <coorsa_interface/DetectPlanesAction.h>
#include <shape_msgs/Plane.h>

typedef actionlib::SimpleActionServer<coorsa_interface::DetectPlanesAction> Server;

void execute(const coorsa_interface::DetectPlanesGoalConstPtr& goal, Server* as)  
{
  coorsa_interface::DetectPlanesResult result;
  shape_msgs::Plane plane;
  for (int i=0;i<4;i++)
    plane.coef[i]=0;
  result.planes.push_back(plane);
  as->setSucceeded(result);
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "fake_plane_detector");
    ros::NodeHandle n;
    Server server(n, "plane_detector", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}