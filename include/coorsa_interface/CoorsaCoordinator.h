/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Coorsa_coordinator.h
 * Author: Davide Chiaravalli
 *
 * Created on Sep 8, 2016, 2:05 PM
 */

#ifndef COORSACOORDINATOR_H
#define COORSACOORDINATOR_H

#define POS_ERROR_TH 0.002
#define CYCLE_HZ 50

#include <ros/ros.h>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <coorsa_box_detector_msgs/DetectBoxesAction.h>
#include <coorsa_box_detector_msgs/DetectPlanesAction.h>
#include <coorsa_interface/EvaluatePlanAction.h>
#include <coorsa_interface/PerformPickingAction.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include "tf_conversions/tf_kdl.h"
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <coorsa_o3d3xx_config_msgs/SetExposureAction.h>

namespace coorsa_interface{

    class CoorsaCoordinator {
        public:
            CoorsaCoordinator(Eigen::Vector3f box_dimensions);
            ~CoorsaCoordinator();
            void detectNewPlane();
            void detectPackagesOnPlane();
            void evaluatePickingPlan();
            void performPickingOnPlane();
            void save_camera_transform();
            void goToStartPose(std::vector<double>,float time);
            void setExposure(float exposure);


        private:
            actionlib::SimpleActionClient<coorsa_box_detector_msgs::DetectPlanesAction> plane_detector_client;               //client for plane detection action server
            actionlib::SimpleActionClient<coorsa_box_detector_msgs::DetectBoxesAction> package_detector_client;              //client for box detection action server
            actionlib::SimpleActionClient<EvaluatePlanAction> task_planner_client;                 //client for task planner action server                     
            actionlib::SimpleActionClient<PerformPickingAction> pick_action_client;                //client for picking task action server
            actionlib::SimpleActionClient<coorsa_o3d3xx_config_msgs::SetExposureAction> expo_action_client;                //client for picking task action server
            shape_msgs::Plane current_plane;                                                       //current picking plane detected
            std::vector<geometry_msgs::Pose> box_poses;                                                       //vector of position of all the boxes on a plane
            Eigen::Vector3f box_dimensions;                                                        //dimension of the boxes to pick
            std::vector<int> box_picking_plan;                                                     //picking order of the box vector
            bool task_complete;                                                                    //flag to check for task completion
            tf::TransformListener listener;
            KDL::Frame camera_transform;
            ros::NodeHandle n;
            ros::Publisher pub_ee_setpoint;
            ros::Subscriber sub_motion;
            bool motion;

            void activateBoxPick(int box_num);
            void selectActivePlane(std::vector<shape_msgs::Plane> planes);
            void connectToActionServers();
            void change_poses_to_world();
            void motion_cb(const sensor_msgs::JointState &msg);
            void move_execute(Eigen::Matrix<long double,Eigen::Dynamic,1> &xd);
            double distance(Eigen::Matrix<long double,Eigen::Dynamic,1> x1, Eigen::Matrix<long double,Eigen::Dynamic,1> x2);
            double distance(std::vector<double> x1, std::vector<double> x2);

    };
}

#endif //COORSACOORDINATOR.H