
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <coorsa_interface/EvaluatePlanAction.h>
#include <shape_msgs/Plane.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

typedef actionlib::SimpleActionServer<coorsa_interface::EvaluatePlanAction> Server;

void execute(const coorsa_interface::EvaluatePlanGoalConstPtr& goal, Server* as)  
{
  coorsa_interface::EvaluatePlanResult result;
  std::vector<geometry_msgs::Pose> box_pose = goal->box_poses;
  std::sort(box_pose.begin(),box_pose.end(),[](geometry_msgs::Pose p1,geometry_msgs::Pose p2){
    return fabs(p1.position.x-p2.position.x)>0.02 ? p1.position.x<p2.position.x:p1.position.y>p2.position.y;});
  result.box_order = box_pose;
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