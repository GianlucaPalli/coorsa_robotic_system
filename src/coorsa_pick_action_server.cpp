
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <coorsa_interface/PerformPickingAction.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <time.h>

typedef actionlib::SimpleActionServer<coorsa_interface::PerformPickingAction> Server;
ros::Publisher motion_command;
bool motion;
std::vector<double> box_dimensions;

void execute(const coorsa_interface::PerformPickingGoalConstPtr& goal, Server* as)  
{
  coorsa_interface::PerformPickingResult result;
  geometry_msgs::Pose box_pose = goal->box_pose;
  sensor_msgs::JointState reference;

  reference.position.resize(6);
  reference.position[0] = box_pose.position.x + box_dimensions[1]/2.0;
  reference.position[1] = box_pose.position.y;
  reference.position[2] = box_pose.position.z+0.2;
  reference.position[3] = 2.2229;
  reference.position[4] = 2.2337;
  reference.position[5] = 0.015;
  std::cout<<"reference_" <<std::endl<<reference<<std::endl;
  motion_command.publish(reference);
  ros::Time begin = ros::Time::now();
  while((motion)||(ros::Time::now().toSec()-begin.toSec()<ros::Duration(1).toSec()));
  reference.position[2] = box_pose.position.z+0.04;
  motion_command.publish(reference);
  begin = ros::Time::now();
  while((motion)||(ros::Time::now().toSec()-begin.toSec()<ros::Duration(1).toSec()));
  reference.position[0]=reference.position[0]-0.1;
  motion_command.publish(reference);
  begin = ros::Time::now();
  while((motion)||(ros::Time::now().toSec()-begin.toSec()<ros::Duration(1).toSec()));
  reference.position[2] = box_pose.position.z+0.2;
  motion_command.publish(reference);
  begin = ros::Time::now();
  while((motion)||(ros::Time::now().toSec()-begin.toSec()<ros::Duration(1).toSec()));
  as->setSucceeded(result);
}

void motion_cb(const sensor_msgs::JointState &msg){
    motion = (bool)msg.position[0];
    //std::cout<<motion<<std::endl;
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "coorsa_pick_action_server");
    ros::NodeHandle n;
    motion = false;
    box_dimensions = {0.30,0.24,0.20};
    motion_command = n.advertise<sensor_msgs::JointState>("ur5_robot/command",1);
    ros::Subscriber motion_controller = n.subscribe("ur5_robot/motion",1,motion_cb);
    Server server(n, "pick_action", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}
