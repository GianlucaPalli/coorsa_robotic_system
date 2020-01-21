
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <coorsa_interface/EvaluatePlanAction.h>
#include <shape_msgs/Plane.h>

typedef actionlib::SimpleActionServer<coorsa_interface::EvaluatePlanAction> Server;

void execute(const coorsa_interface::EvaluatePlanGoalConstPtr& goal, Server* as)  
{
  coorsa_interface::EvaluatePlanResult result;
  std::vector<int> box_order;
  box_order = {1,2};
  result.box_order = box_order;
  as->setSucceeded(result);
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "fake_box_planner");
    ros::NodeHandle n;
    Server server(n, "task_planner", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}