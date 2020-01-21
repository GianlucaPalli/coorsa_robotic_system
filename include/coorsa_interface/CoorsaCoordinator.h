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

#include <ros/ros.h>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <coorsa_interface/DetectBoxesAction.h>
#include <coorsa_interface/DetectPlanesAction.h>
#include <coorsa_interface/EvaluatePlanAction.h>
#include <coorsa_interface/PerformPickingAction.h>

namespace coorsa_interface{

    class CoorsaCoordinator {
        public:
            CoorsaCoordinator(Eigen::Vector3f box_dimensions);
            ~CoorsaCoordinator();
            void detectNewPlane();
            void detectPackagesOnPlane();
            void evaluatePickingPlan();
            void performPickingOnPlane();


        private:
            actionlib::SimpleActionClient<DetectPlanesAction> plane_detector_client;               //client for plane detection action server
            actionlib::SimpleActionClient<DetectBoxesAction> package_detector_client;              //client for box detection action server
            actionlib::SimpleActionClient<EvaluatePlanAction> task_planner_client;                 //client for task planner action server                     
            actionlib::SimpleActionClient<PerformPickingAction> pick_action_client;                //client for picking task action server
            shape_msgs::Plane current_plane;                                                       //current picking plane detected
            std::vector<geometry_msgs::Pose> box_poses;                                                       //vector of position of all the boxes on a plane
            Eigen::Vector3f box_dimensions;                                                        //dimension of the boxes to pick
            std::vector<int> box_picking_plan;                                                     //picking order of the box vector
            bool task_complete;                                                                    //flag to check for task completion

            void activateBoxPick(int box_num);
            void selectActivePlane(std::vector<shape_msgs::Plane> planes);
            void connectToActionServers();

    };
}

#endif //COORSACOORDINATOR.H