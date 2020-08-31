// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
//#include <my_priority_level_control/main_test.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <opencv_apps/LineArrayStamped.h>
#include <Eigen/Geometry> 
#include <kdl/frames.hpp>
#include <actionlib/server/simple_action_server.h>
#include <coorsa_interface/PerformPickingAction.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <csignal>
#include <visualization_msgs/MarkerArray.h>

#define CYCLE_HZ 50

typedef actionlib::SimpleActionServer<coorsa_interface::PerformPickingAction> Server;
using namespace Eigen;
//#define CYCLE_HZ 125
//#define FORCE_TH 30.0

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc

#define IMAGE_HEIGHT 264
#define PX_TO_MM -0.79
#define CAMERA_OFFSET 85.5
#define TOOL_APPR 0.16
#define TOOL_DEPTH 0.04
#define MOVE_TIME 5.0
#define CALIBRATION_HEIGHT 0.255 //0.475
#define POS_ERROR_TH 0.003
#define BOX_DISP -0.50

#define ROS_RATE 10

double disp = 0.0;
double force = 0.0;
double theta = 0.0;
bool received = true;

enum _state {INIT, MOVE_OVER_BOX,WAIT_OVER_BOX, WAIT_INIT_POSE, WAIT_EVAL, CAMERA_ALIGN, WAIT_NEW_EVAL, TOOL_ALIGN, TOOL_APPROACH, TOOL_INSERT, BOX_MOVE, TOOL_REMOVE, FINISH};

_state state = INIT;
ros::Publisher pub_ee_setpoint;


 bool lineInRange(float y1, float y2){
     if ((y1 < IMAGE_HEIGHT-50 && y1 > IMAGE_HEIGHT-210) && (y2 < IMAGE_HEIGHT-50 && y2 > IMAGE_HEIGHT-210))
         return true;
     else 
         return false;
 }



void callbackLines(const opencv_apps::LineArrayStamped::ConstPtr& msg) 
{
  if(!received){
  ROS_INFO("line message received");

    ROS_INFO("size: %d", (int) msg->lines.size());

    double y = 0.0;
    double angle;
    int count = 0;
    
    theta = 0.0;
    
    if(msg->lines.size()==0) return;

    for(int i=0; i<msg->lines.size(); i++)
    {
      ROS_INFO_STREAM("point " << i);
      ROS_INFO_STREAM("1 x y: " << msg->lines[i].pt1.x << " " << msg->lines[i].pt1.y);
      ROS_INFO_STREAM("2 x y: " << msg->lines[i].pt2.x << " " << msg->lines[i].pt2.y);


      angle = atan(-(msg->lines[i].pt1.y - msg->lines[i].pt2.y)/(msg->lines[i].pt1.x - msg->lines[i].pt2.x));
      if ((fabs(angle)<M_PI/9) && lineInRange(msg->lines[i].pt1.y, msg->lines[i].pt2.y)){        
        theta += angle;
        y += msg->lines[i].pt1.y + msg->lines[i].pt2.y;
        count++;
      }
        
    }
    
    if (count!=0){
      theta = theta/count;
      y = y/(2*count);
      received = true;
    }
      

    disp = (y - IMAGE_HEIGHT/2) * PX_TO_MM + CAMERA_OFFSET;

    ROS_INFO_STREAM("disp: " << disp);
    ROS_INFO_STREAM("theta: " << theta);

    
  }
}

void callbackWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    //ROS_INFO("New wrench received");
    //force = -0.1*(force-msg->wrench.force.z);
    force = msg->wrench.force.z;
    //ROS_INFO_STREAM("New wrench received " << force);
    
}

bool go_on = true;

void signalHandler( int signum ) {
   std::cout << "Interrupt signal (" << signum << ") received.\n";

   // cleanup and close up stuff here  
   // terminate program  

   sleep(1);

   go_on = false;
}

void move_execute(Eigen::Matrix<long double,Eigen::Dynamic,1> &xd, ros::Publisher &pub_ee_setpoint)
{

  std_msgs::Float64MultiArray setpointMsg;/*it's defined by the arrayed version of the matrix and the layout of the matrix*/
  setpointMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());//one structure MultiArrayDimension per dimension
  // setpointMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  setpointMsg.layout.data_offset = 0;
  setpointMsg.layout.dim[0].label = "";
  setpointMsg.layout.dim[0].size = 8;
  setpointMsg.layout.dim[0].stride = 0;

  setpointMsg.data.clear();

  for (int i=0; i<7; i++)
  {
    setpointMsg.data.push_back(xd(i,0));
  }

  setpointMsg.data.push_back(MOVE_TIME);

  pub_ee_setpoint.publish(setpointMsg);

  //ros::Duration(2*MOVE_TIME).sleep();
}

double distance(Eigen::Matrix<long double,Eigen::Dynamic,1> x1, Eigen::Matrix<long double,Eigen::Dynamic,1> x2)
{
  double distance = 0.0;
  for (int i=0; i<3; i++) distance += pow(x1(i,0)-x2(i,0),2);
  /*KDL::Rotation rotref = KDL::Rotation::Quaternion(x1(4,0),x1(5,0),x1(6,0),x1(3,0));
  KDL::Rotation rotnow = KDL::Rotation::Quaternion(x2(4,0),x2(5,0),x2(6,0),x2(3,0));
  double yawnow,n1,n2,yawref;
  rotref.GetRPY(n1,n2,yawref);
  rotnow.GetRPY(n1,n2,yawnow);
  double yaw_est = fabs(yawnow-yawref)/50.0;
  //distance += yaw_est;
  //std::cout<<"est"<< yaw_est<<std::endl;*/
  return sqrt(distance);
}

void execute(const coorsa_interface::PerformPickingGoalConstPtr& goal, Server* as)  
{
  
  coorsa_interface::PerformPickingResult result;
  geometry_msgs::Pose box_pose = goal->box_pose;
  sensor_msgs::JointState reference;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transform_above;
  Eigen::Matrix<long double,Eigen::Dynamic,1> x, xd, x0,x_over_box;
  state = INIT;
  x.resize(7,1);
  xd.resize(7,1);
  x0.resize(7,1);
  x_over_box.resize(7,1);

  x0 << 0.4, 0.0, CALIBRATION_HEIGHT, 0.707, 0.0, 0.707, 0.0;
  x_over_box << 0.4, 0.0, 0.8, 0.707, 0.0, 0.707, 0.0; 

  x0(0,0) = box_pose.position.x;
  x0(1,0) = box_pose.position.y;
  x0(2,0) = box_pose.position.z + CALIBRATION_HEIGHT;

  double picking_orientation,dummy_value1,dummy_value2,motion_orientation,xt,yt,zt,wt;
  KDL::Rotation picking_matrix = KDL::Rotation::Quaternion(box_pose.orientation.x,box_pose.orientation.y,box_pose.orientation.z,box_pose.orientation.w);
  picking_matrix.GetRPY(dummy_value1,dummy_value2,picking_orientation);
  std::cout<<"roll "<< dummy_value1<<std::endl;
  std::cout<<"pitch "<< dummy_value2<<std::endl;
  //picking_orientation = picking_orientation;
  std::cout<<"picking orientation before adjustment"<< picking_orientation<<std::endl;
  /*if (fabs(picking_orientation)>(M_PI/2.0)) {
    std::cout<<"motion error stopping"<<std::endl;
    as->setSucceeded();
    ros::shutdown();
  }*/
  if (picking_orientation>0){
    picking_orientation= picking_orientation-M_PI/2.0;
  }
  else{
    picking_orientation = picking_orientation+M_PI/2.0;
  }
  std::cout<<"picking orientation after adjustment"<< picking_orientation<<std::endl;


  motion_orientation = picking_orientation;
  std::cout<<"motion orientation "<< motion_orientation<<std::endl;

  picking_orientation = picking_orientation+M_PI;
  std::cout<<"picking orientation "<< picking_orientation<<std::endl;
  
  KDL::Rotation orientation_camera = KDL::Rotation::EulerZYX(0,M_PI/2.0,-picking_orientation);
  orientation_camera.GetQuaternion(xt,yt,zt,wt);

  x0(3,0)=wt;
  x0(4,0)=xt;
  x0(5,0)=yt;
  x0(6,0)=zt;

  Eigen::Quaterniond qua0;
  qua0.w()= x0(3,0);
  qua0.x()= x0(4,0);
  qua0.y()= x0(5,0);
  qua0.z()= x0(6,0);

  x_over_box = x0;
  x_over_box(2,0) += 0.35;

  bool first = true;

  go_on = true;

  ROS_INFO("action received");

  ros::Rate rate(ROS_RATE);

  while (ros::ok() && go_on){

       
    // ROS_INFO("################INSIDE LOPP");


    //  ROS_INFO("Init =0 translation: x= %f, y= %f, z= %f; rotation: x= %f, y= %f, z= %f, w= %f ", transform_above.transform.translation.x, transform_above.transform.translation.y, transform_above.transform.translation.z, transform_above.transform.rotation.x, transform_above.transform.rotation.y, transform_above.transform.rotation.z , transform_above.transform.rotation.w );
    
      try 
      {
        // transform_above = tfBuffer.lookupTransform("base_link", "rg2_eef_link",  ros::Time(0),ros::Duration(1000/cycleHz));
        transform_above = tfBuffer.lookupTransform("base_link", "ee_link",  ros::Time(0),ros::Duration(1000/CYCLE_HZ));

      } catch (tf::TransformException ex) 
      {
        ROS_ERROR("%s",ex.what());
      }

      x(0,0)= transform_above.transform.translation.x;
      x(1,0)= transform_above.transform.translation.y;
      x(2,0)= transform_above.transform.translation.z;
      x(3,0)= transform_above.transform.rotation.w;
      x(4,0)= transform_above.transform.rotation.x;
      x(5,0)= transform_above.transform.rotation.y;
      x(6,0)= transform_above.transform.rotation.z;
            
      //ROS_INFO_STREAM("actual pose: " << x );

      switch(state)
      {
          case INIT:
          ROS_INFO_STREAM("move to init " << x0);
          
          move_execute(x0, pub_ee_setpoint);

          state = WAIT_INIT_POSE;
          break;
        case WAIT_INIT_POSE:{
          double dist = distance(x0, x);
          if(dist < POS_ERROR_TH){
            ROS_INFO_STREAM("init pose reached" );
            received = false;
            state = WAIT_EVAL;
          }
          /*else
          {
            std::cout<<"big distances "<<dist<<std::endl;
          }*/
        }break;
        case MOVE_OVER_BOX:
          ROS_INFO_STREAM("move over_box " << x0);
          
          move_execute(x0, pub_ee_setpoint);

          state = WAIT_OVER_BOX;
          break;
        case WAIT_OVER_BOX:{
          double dist = distance(x0, x);
          if(dist < POS_ERROR_TH){
            ROS_INFO_STREAM("perform line evaluation" );
            received = false;
            state = WAIT_EVAL;
          }
          /*else
          {
            std::cout<<"big distances "<<dist<<std::endl;
          }*/
        }
          break;
        
        case WAIT_EVAL:
          if (received) {
            ROS_INFO_STREAM("align camera" );

            for (int i=0; i<3; i++) xd(i,0) = x(i,0);
            for (int i=3; i<7; i++) xd(i,0) = x0(i,0);
            xd(0,0) = x(0,0) + (disp-CAMERA_OFFSET)/1000*cos(motion_orientation);
            xd(1,0) = x(1,0) + (disp-CAMERA_OFFSET)/1000*sin(motion_orientation);

            move_execute(xd, pub_ee_setpoint);
            ROS_INFO_STREAM("target pose: " << xd );

            state = CAMERA_ALIGN;
          }        
          break;
        case CAMERA_ALIGN:
          if(distance(xd, x) < POS_ERROR_TH){
            ROS_INFO_STREAM("camera aligned, perform new evaluation" );
            received = false;
            state = WAIT_NEW_EVAL;
          }
          break;
        case WAIT_NEW_EVAL:
          if (received) {
            ROS_INFO_STREAM("align tool" );

            for (int i=0; i<3; i++) xd(i,0) = x(i,0);
            xd(0,0) = x(0,0) + (disp)/1000*cos(motion_orientation);
            xd(1,0) = x(1,0) + (disp)/1000*sin(motion_orientation);
            Quaterniond q(AngleAxisd(theta, Vector3d(0,0,1))); 
            q *= qua0;
            
            //ROS_INFO_STREAM("do rotation " << q.w << q.x << q.y << q.z);
            
            xd(3,0) = q.w();
            xd(4,0) = q.x();
            xd(5,0) = q.y();
            xd(6,0) = q.z();             

            move_execute(xd, pub_ee_setpoint);

            state = TOOL_ALIGN;
          }        
          break;
        case TOOL_ALIGN:
          if(distance(xd, x) < POS_ERROR_TH){
            ROS_INFO_STREAM("tool aligned, perform insertion" );
          
            for (int i=0; i<3; i++) xd(i,0) = x(i,0);
            xd(2,0) = x(2,0) - TOOL_APPR;
            
            move_execute(xd, pub_ee_setpoint);

            state = TOOL_APPROACH;
          }
          break;
        case TOOL_APPROACH:
          ROS_INFO_STREAM(force);
          if(distance(xd, x) > POS_ERROR_TH && fabs(force) > 25){
            ROS_INFO_STREAM("tool in contact, perform new evaluation" );
            
            move_execute(x0, pub_ee_setpoint);

            state = WAIT_INIT_POSE;
          
          }

          if(distance(xd, x) < POS_ERROR_TH){
            ROS_INFO_STREAM("tool aligned, perform insertion" );
          
            for (int i=0; i<3; i++) xd(i,0) = x(i,0);
            xd(2,0) = x(2,0) - TOOL_DEPTH;

            move_execute(xd, pub_ee_setpoint);

            state = TOOL_INSERT;
          }
          break;
        case TOOL_INSERT:
          if(distance(xd, x) < POS_ERROR_TH){
            ROS_INFO_STREAM("tool inserted, move box" );
          
            for (int i=0; i<3; i++) xd(i,0) = x(i,0);
            xd(0,0) = x(0,0) + BOX_DISP*cos(motion_orientation);
            xd(1,0) = x(1,0) + BOX_DISP*sin(motion_orientation);
             move_execute(xd, pub_ee_setpoint);

            state = BOX_MOVE;
          }
          break;
        case BOX_MOVE:
          if(distance(xd, x) < POS_ERROR_TH){
            ROS_INFO_STREAM("box moved, remove tool" );
          
            for (int i=0; i<3; i++) xd(i,0) = x(i,0);
            xd(2,0) = x(2,0) + TOOL_DEPTH + TOOL_APPR;

             move_execute(xd, pub_ee_setpoint);

            state = TOOL_REMOVE;
          }
          break;          
        case TOOL_REMOVE:
          if(distance(xd, x) < POS_ERROR_TH){
            ROS_INFO_STREAM("tool removed, back to initial position" );
          
            for (int i=0; i<7; i++) xd(i,0) = x0(i,0);

             move_execute(xd, pub_ee_setpoint);

            state = FINISH;
          }
          break;
        case FINISH:{
            std::cout<<"finish"<<std::endl;
            state = INIT;
            go_on = false;
        }
        default:{
          std::cout<<"default????"<<std::endl;
          go_on = false;
        }

          break;
      }

      ros::spinOnce();
      rate.sleep();
  }
  
  as->setSucceeded(result);
}




int main(int argc, char** argv) 
{
  signal(SIGINT, signalHandler);  
  // ROS set-ups:
  ros::init(argc, argv, "coorsa_test_action"); //node name

  ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

  ROS_INFO("main: instantiating an object of type ExampleRosClass");
  

  ROS_INFO("main: going into spin; let the callbacks do all the work");
  


  visualization_msgs::Marker marker;

  ROS_INFO("Initializing Subscribers");

  ros::Subscriber sub_hough_lines = nh.subscribe<opencv_apps::LineArrayStamped>("/hough_lines/lines", 1, &callbackLines);

  ros::Subscriber sub_wrench = nh.subscribe<geometry_msgs::WrenchStamped>("/wrench", 1, &callbackWrench);


  ROS_INFO("Initializing Publishers");
  // minimal_publisher_ = nh.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true); 

  pub_ee_setpoint = nh.advertise<std_msgs::Float64MultiArray>("/ee_setpoint", 1);

  ROS_INFO("Initializing Action Server");
  Server server(nh, "pick_action", boost::bind(&execute, _1, &server), false);
  server.start();

  //vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  //pub_ee_setpoint.publish(setpointMsg);





  
  

 

  // ros::spinOnce();

  // ros::Duration(2*MOVE_TIME).sleep();

  // state = INIT;

  ROS_INFO("################BEFORE LOOP");

  ros::spin();

  

  return 0;
} 

