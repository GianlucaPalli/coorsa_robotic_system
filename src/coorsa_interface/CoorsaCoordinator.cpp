/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CoorsaCoordinator.cpp
 * Author: Davide Chiaravalli
 * 
 * Created on Gen 7, 2020, 2:05 PM
 */

#include "CoorsaCoordinator.h"


namespace coorsa_interface {

    CoorsaCoordinator::CoorsaCoordinator(Eigen::Vector3f box_dimensions):plane_detector_client("box_detector_node/detect_planes",true),
                                            package_detector_client("box_detector_node/detect_boxes",true),
                                            task_planner_client("task_planner",true),
                                            pick_action_client("pick_action",true),
                                            expo_action_client("/o3d3xx_config/set_exposure",true){//box_detector_node/detect_planesbox_detector_node/detect_boxes
        this->connectToActionServers();
        this->box_dimensions = box_dimensions;
        this->task_complete = false;
        this->motion = false;
        this->pub_ee_setpoint = this->n.advertise<std_msgs::Float64MultiArray>("/ee_setpoint", 1);
        this->sub_motion = n.subscribe("ur5_robot/motion",1,&CoorsaCoordinator::motion_cb,this);
        ros::Time begin = ros::Time::now();
        while(ros::Time::now().toSec()-begin.toSec()<ros::Duration(2).toSec());
    }
	
    CoorsaCoordinator::~CoorsaCoordinator(){
    }

    void CoorsaCoordinator::motion_cb(const sensor_msgs::JointState &msg){
        this->motion = (bool)msg.position[0];
    }

    void CoorsaCoordinator::move_execute(Eigen::Matrix<long double,Eigen::Dynamic,1> &xd)
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

    setpointMsg.data.push_back(1.0); //MOVE_TIME

    this->pub_ee_setpoint.publish(setpointMsg);

    //ros::Duration(2*MOVE_TIME).sleep();
    }


    void CoorsaCoordinator::save_camera_transform(){
        tf::StampedTransform transform;
        this->listener.waitForTransform("base_link", "o3d3xx/camera_link",ros::Time::now(), ros::Duration(3.0));
        try{
            this->listener.lookupTransform("base_link", "o3d3xx/camera_link",  
                                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        //KDL::Rotation rot=KDL::Rotation::Quaternion(transform.getRotation().getAxis().getX(),transform.getRotation().getAxis().getY(),transform.getRotation().getAxis().getZ(),transform.getRotation().getW());
        //KDL::Vector vec = KDL::Vector(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
        tf::transformTFToKDL(transform,this->camera_transform);
        //this->camera_transform.M = KDL::Rotation::Quaternion(0.00615923,0.430192,0.0118229,0.902639);
        //this->camera_transform = KDL::Frame(rot,vec);
        //std::cout <<this->camera_transform.p[0]<<" "<<this->camera_transform.p[1]<<" "<<this->camera_transform.p[2]<<" "<<std::endl;
        }

    void CoorsaCoordinator::change_poses_to_world(){
        std::cout <<this->camera_transform.p[0]<<" "<<this->camera_transform.p[1]<<" "<<this->camera_transform.p[2]<<" "<<std::endl;
        geometry_msgs::Pose temp_pose;
        for (int i=0;i<this->box_poses.size();i++){
            //save current box pose
            temp_pose=this->box_poses[i];
            //transform in KDL::frame
            KDL::Rotation rot = KDL::Rotation::Quaternion(temp_pose.orientation.x,temp_pose.orientation.y,temp_pose.orientation.z,temp_pose.orientation.w);
            KDL::Vector vec = KDL::Vector(temp_pose.position.x,temp_pose.position.y,temp_pose.position.z);
            //KDL::Frame box_temp = KDL::Frame(rot,vec);
            KDL::Frame box_temp = KDL::Frame(rot,vec);
            //transform to robot_base_frame
            box_temp = this->camera_transform*box_temp;
            //save back in the vector
            std::vector<double> quat(4);
            box_temp.M.GetQuaternion(quat[0],quat[1],quat[2],quat[3]);
            this->box_poses[i].position.x = box_temp.p[0];
            this->box_poses[i].position.y = box_temp.p[1];
            this->box_poses[i].position.z = box_temp.p[2];
            this->box_poses[i].orientation.x = quat[0];
            this->box_poses[i].orientation.y = quat[1];
            this->box_poses[i].orientation.z = quat[2];
            this->box_poses[i].orientation.w = quat[3];

            std::cout<<"i "<<i<<"\n"<<this->box_poses[i]<<std::endl;
        }
    }

    double CoorsaCoordinator::distance(Eigen::Matrix<long double,Eigen::Dynamic,1> x1, Eigen::Matrix<long double,Eigen::Dynamic,1> x2)
    {
    double distance = 0.0;
    for (int i=0; i<3; i++) distance += pow(x1(i,0)-x2(i,0),2);
    
    return sqrt(distance);
    }

    double CoorsaCoordinator::distance(std::vector<double> x1, std::vector<double> x2)
    {
    double distance = 0.0;
    for (int i=0; i<3; i++) distance += pow(x1[i]-x2[i],2);
    
    return sqrt(distance);
    }

    void CoorsaCoordinator::setExposure(float exposure){
        coorsa_o3d3xx_config_msgs::SetExposureGoal goal;
        goal.exposure = exposure;
        this->expo_action_client.sendGoal(goal);
        sleep(5.0);
    }


    void CoorsaCoordinator::goToStartPose(std::vector<double> start_pose,float time){
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transform_above;
        std_msgs::Float64MultiArray msg;
        std::vector<double> current_pose;
        current_pose.resize(7);
        msg.data.resize(8);
        for (int i=0;i<7;i++){
            msg.data[i] = start_pose[i];
        }
        msg.data[7] = time;
        this->pub_ee_setpoint.publish(msg);

        while (ros::ok()){

            try 
            {
                // transform_above = tfBuffer.lookupTransform("base_link", "rg2_eef_link",  ros::Time(0),ros::Duration(1000/cycleHz));
                transform_above = tfBuffer.lookupTransform("base_link", "ee_link",  ros::Time(0),ros::Duration(1000/CYCLE_HZ));

            } catch (tf::TransformException ex) 
            {
                ROS_ERROR("%s",ex.what());
            }

            current_pose[0]= transform_above.transform.translation.x;
            current_pose[1]= transform_above.transform.translation.y;
            current_pose[2]= transform_above.transform.translation.z;
            current_pose[3]= transform_above.transform.rotation.w;
            current_pose[4]= transform_above.transform.rotation.x;
            current_pose[5]= transform_above.transform.rotation.y;
            current_pose[6]= transform_above.transform.rotation.z;
            if(distance(start_pose, current_pose) < 0.005) break;
        }
    }

    void CoorsaCoordinator::detectNewPlane(){
        coorsa_box_detector_msgs::DetectPlanesGoal goal;
        std::vector<shape_msgs::Plane> result_plane;
        goal.num_planes = 1;
        std::cout<<"chech_plane"<<std::endl;
        this->plane_detector_client.sendGoal(goal);
        std::cout<<"done_plane"<<std::endl;
        this->plane_detector_client.waitForResult();
        result_plane = this->plane_detector_client.getResult()->planes;
        std::cout<<"result_plane"<<std::endl;
        this->selectActivePlane(result_plane);

        std::cout<<result_plane[0];
    }

    void CoorsaCoordinator::detectPackagesOnPlane(){
        coorsa_box_detector_msgs::DetectBoxesGoal goal;
        coorsa_box_detector_msgs::DetectBoxesResult result;
        goal.box_dimensions.x = this->box_dimensions[0];
        goal.box_dimensions.y = this->box_dimensions[1];
        goal.box_dimensions.z = this->box_dimensions[2];
        goal.plane_vertical_offset = 0;
        goal.plane_coefficients = this->current_plane.coef;
        this->package_detector_client.sendGoal(goal);
                std::cout<<"chech"<<std::endl;
        this->package_detector_client.waitForResult();
                std::cout<<"done"<<std::endl;
        std::vector<geometry_msgs::Pose> temp_poses;

        this->box_poses = package_detector_client.getResult()->box_poses;

        this->save_camera_transform();
        this->change_poses_to_world();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
/*
        temp_poses = this->box_poses;
        this->box_poses.clear();
        std::cout<<"size"<<temp_poses.size()<<std::endl;
        for (int i=0;i<temp_poses.size();i++){
            if (fabs(temp_poses[i].position.z-this->box_dimensions[2])<0.05){
                this->box_poses.push_back(temp_poses[i]);
            }
        }*/
        std::cout<<"size"<<this->box_poses.size()<<std::endl;
        for (int i=0;i<this->box_poses.size();i++){
            std::cout<<this->box_poses[i].position.x<<std::endl;
            std::cout<<this->box_poses[i].position.y<<std::endl;
            std::cout<<this->box_poses[i].position.z<<std::endl;
            std::cout<<this->box_poses[i].orientation.x<<std::endl;
            std::cout<<this->box_poses[i].orientation.y<<std::endl;
            std::cout<<this->box_poses[i].orientation.z<<std::endl;
            std::cout<<this->box_poses[i].orientation.w<<std::endl;
            std::cout<<"stop"<<std::endl;
        }
    } 

    void CoorsaCoordinator::evaluatePickingPlan(){
        EvaluatePlanGoal goal;
        EvaluatePlanResult result;
        goal.box_poses = this->box_poses;
        this->task_planner_client.sendGoal(goal);
        this->task_planner_client.waitForResult();
        this->box_poses =this->task_planner_client.getResult()->box_order;
    }

    void CoorsaCoordinator::performPickingOnPlane(){
        int num_boxes = this->box_poses.size();
        for (int i=0;i<2;i++){
            this->activateBoxPick(i);
        }
    }

    void CoorsaCoordinator::activateBoxPick(int box_num){
        PerformPickingGoal goal;
        goal.box_pose = this->box_poses[box_num];
        goal.box_pose.position.x = goal.box_pose.position.x ;
        this->pick_action_client.sendGoal(goal);
        this->pick_action_client.waitForResult();
    }

    void CoorsaCoordinator::selectActivePlane(std::vector<shape_msgs::Plane> planes){
        //with multiple planes choice based on num_points in result
        this->current_plane = planes[0];
        //std::cout<<this->current_plane<<std::endl;
    }

    void CoorsaCoordinator::connectToActionServers(){
        ROS_INFO("waiting for plane detection action server");
        //this->plane_detector_client.waitForServer();
        ROS_INFO("plane detection action server found");
        ROS_INFO("waiting for package detection action server");
        //this->package_detector_client.waitForServer();
        ROS_INFO("package detection action server found");
        ROS_INFO("waiting for task planner action server");
        //this->task_planner_client.waitForServer();
        ROS_INFO("task planner action server found");
        ROS_INFO("waiting for pick action server");
        //this->pick_action_client.waitForServer();
        ROS_INFO("pick action server found");
    };
}



