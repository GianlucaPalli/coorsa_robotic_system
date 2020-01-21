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

    CoorsaCoordinator::CoorsaCoordinator(Eigen::Vector3f box_dimensions):plane_detector_client("plane_detector",true),
                                            package_detector_client("package_detector",true),
                                            task_planner_client("task_planner",true),
                                            pick_action_client("pick_action",true){
        this->connectToActionServers();
        this->box_dimensions = box_dimensions;
        this->task_complete = false;
    }
	
    CoorsaCoordinator::~CoorsaCoordinator(){
    }

    void CoorsaCoordinator::detectNewPlane(){
        DetectPlanesGoal goal;
        std::vector<shape_msgs::Plane> result_plane;
        goal.num_planes = 1;
        this->plane_detector_client.sendGoal(goal);
        this->plane_detector_client.waitForResult();
        result_plane = this->plane_detector_client.getResult()->planes;
        this->selectActivePlane(result_plane);
    }

    void CoorsaCoordinator::detectPackagesOnPlane(){
        DetectBoxesGoal goal;
        DetectBoxesResult result;
        goal.box_dimensions.x = this->box_dimensions[0];
        goal.box_dimensions.y = this->box_dimensions[1];
        goal.box_dimensions.z = this->box_dimensions[2];
        goal.plane_vertical_offset = 0;
        goal.plane_coefficients = this->current_plane.coef;
        this->package_detector_client.sendGoal(goal);
        this->package_detector_client.waitForResult();
        this->box_poses = package_detector_client.getResult()->box_poses;
    }

    void CoorsaCoordinator::evaluatePickingPlan(){
        EvaluatePlanGoal goal;
        EvaluatePlanResult result;
        goal.box_poses = this->box_poses;
        this->task_planner_client.sendGoal(goal);
        this->task_planner_client.waitForResult();
        this->box_picking_plan =this->task_planner_client.getResult()->box_order;
    }

    void CoorsaCoordinator::performPickingOnPlane(){
        int num_boxes = this->box_picking_plan.size();
        for (int i=0;i<num_boxes;i++){
            this->activateBoxPick(this->box_picking_plan[i]);
        }
    }

    void CoorsaCoordinator::activateBoxPick(int box_num){
        PerformPickingGoal goal;
        goal.box_pose = this->box_poses[box_num];
        this->pick_action_client.sendGoal(goal);
        this->pick_action_client.waitForResult();
    }

    void CoorsaCoordinator::selectActivePlane(std::vector<shape_msgs::Plane> planes){
        //with multiple planes choice based on num_points in result
        this->current_plane = planes[0];
    }

    void CoorsaCoordinator::connectToActionServers(){
        ROS_INFO("waiting for plane detection action server");
        this->plane_detector_client.waitForServer();
        ROS_INFO("plane detection action server found");
        ROS_INFO("waiting for package detection action server");
        this->package_detector_client.waitForServer();
        ROS_INFO("package detection action server found");
        ROS_INFO("waiting for task planner action server");
        this->task_planner_client.waitForServer();
        ROS_INFO("task planner action server found");
        ROS_INFO("waiting for pick action server");
        this->pick_action_client.waitForServer();
        ROS_INFO("pick action server found");
    };
}



