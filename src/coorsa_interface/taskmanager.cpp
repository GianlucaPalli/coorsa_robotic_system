#include "coorsa_interface/taskmanager.h"

namespace rvt = rviz_visual_tools;

TaskManager::TaskManager(std::string group): planning_group(group)
{

    tfListener = new tf2_ros::TransformListener(tfBuffer);

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script

    // For visualizing things in rviz
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers"));

    visual_tools_->deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in Rviz
    visual_tools_->loadRemoteControl();

    // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
    text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75; // above head of PR2
    visual_tools_->publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
    visual_tools_->trigger();


    robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
    robot_model = robot_model_loader.getModel();

    //myplanning_scene = new planning_scene::PlanningScene(robot_model);


    planning_scene_monitor_ =  std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description"/*, tf_listener_*/);


    
    robot_trajectory = new robot_trajectory::RobotTrajectory(robot_model, planning_group);

    myrobot_state = new robot_state::RobotState(robot_model);

    myrobot_state->printStateInfo();
    myrobot_state->printStatePositions();

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    move_group = new moveit::planning_interface::MoveGroupInterface(planning_group);

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(planning_group);

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    planning_frame = move_group->getPlanningFrame();
    planning_frame.erase(0, 1);
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", planning_frame.c_str());

    planning_KDLFrame = KDL::Frame::Identity();

    // We can also print the name of the end-effector link for this group.
    ee_link = move_group->getEndEffectorLink();
    ROS_INFO_NAMED("tutorial", "End effector link: %s", ee_link.c_str());


    move_group->setNumPlanningAttempts(3);
    move_group->setPlanningTime(5);

    //move_group->setMaxVelocityScalingFactor(0.1);

     
};


bool TaskManager::recallJointConfig(std::string namedPose)
{
    move_group->setNamedTarget(namedPose);

    planAndExecute();

}




void TaskManager::addCollisionObject(std::string name, shape_msgs::SolidPrimitive primitive, geometry_msgs::Pose box_pose)
{
    // Adding/Removing Objects and Attaching/Detaching Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = name;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in Rviz of status
    visual_tools_->publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools_->trigger();

    // Sleep to allow MoveGroup to recieve and process the collision object message
    //ros::Duration(1.0).sleep();
}


geometry_msgs::TransformStamped TaskManager::getObjectTransformStamped(std::string child, std::string parent)
{
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform(parent, child, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }

    return transformStamped;
}

KDL::Frame TaskManager::getObjectKDLFrame(std::string child, std::string parent)
{
    return tf2::transformToKDL(getObjectTransformStamped(child, parent));
}

void TaskManager::setStartStateAsCurrent()
{
    move_group->setStartState(*move_group->getCurrentState());
}

void TaskManager::clearWaypoints()
{
    waypoints.clear();
}

void TaskManager::addWaypoint(KDL::Frame waypoint, std::string parent)
{
    waypoints.push_back(getPoseFromKDL(waypoint, parent));
}

void TaskManager::setPlanningTime(double time)
{
    move_group->setPlanningTime(time);
}

bool TaskManager::executeCartesianPath(double speed_scaling, double acceleration_scaling, double jump_threshold, double eef_step)
{
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Cartesian path generation: (%.2f%% acheived)", fraction * 100.0);

    if(speed_scaling > 1.0) speed_scaling = 1.0;
    if(acceleration_scaling > 1.0) acceleration_scaling = 1.0;

    if(speed_scaling < 0.0) speed_scaling = 0.0;
    if(acceleration_scaling < 0.0) acceleration_scaling = 0.0;

    if(speed_scaling < 1.0){
        robot_trajectory->setRobotTrajectoryMsg(*myrobot_state, trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization time_parametrization;

        time_parametrization.computeTimeStamps(*robot_trajectory, speed_scaling, acceleration_scaling);

        robot_trajectory->getRobotTrajectoryMsg(trajectory);
    }

    visual_tools_->publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools_->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools_->trigger();

    ROS_INFO_STREAM("Execute cartesian path ");
    my_plan.trajectory_= trajectory;
    move_group->execute(my_plan);
}

geometry_msgs::PoseStamped TaskManager::getCurrentPoseStamped()
{
    return move_group->getCurrentPose();
}

geometry_msgs::Pose TaskManager::getCurrentPose()
{
    geometry_msgs::PoseStamped current_pose_stamped = getCurrentPoseStamped();

    return current_pose_stamped.pose;

}

KDL::Frame TaskManager::getCurrentKDLFrame()
{
//    geometry_msgs::PoseStamped current_pose_stamped = getCurrentPoseStamped();

//    geometry_msgs::Quaternion orient = current_pose_stamped.pose.orientation;
//    geometry_msgs::Point pos = current_pose_stamped.pose.position;

//    KDL::Rotation::Quaternion rotation(orient.x, orient.y, orient.z, orient.w);
//    KDL::Vector position(pos.x, pos.y, pos.z);

    KDL::Frame target;//(position, orientation);

    tf::poseMsgToKDL(getCurrentPose(), target);

    return target;

}

void TaskManager::addWaypoint(geometry_msgs::Pose waypoint)
{
    waypoints.push_back(waypoint);
}

void TaskManager::visualizePose(geometry_msgs::Pose pose, std::string label)
{

    visual_tools_->publishAxisLabeled(pose, label, rvt::SMALL);
    visual_tools_->trigger();

}

geometry_msgs::Pose TaskManager::getPoseFromKDL(KDL::Frame frame, const std::string& parent)
{
    tf2::Stamped<KDL::Frame> target_stamped(frame, ros::Time(), parent);

    geometry_msgs::PoseStamped pose_stamped = tf2::toMsg(target_stamped);

    return pose_stamped.pose;
}

void TaskManager::resetTrajectory()
{
    clearWaypoints();

    setStartStateAsCurrent();

    addWaypoint(getCurrentPose());
}

bool TaskManager::moveApproachForward(double distance, std::string frame, std::string parent, double speed_scaling)
{
    KDL::Frame target_frame = getObjectKDLFrame(frame, parent);

    return moveApproachForward(distance, target_frame, speed_scaling);
}

bool TaskManager::moveApproachBackward(double distance, std::string frame, std::string parent, double speed_scaling)
{
    KDL::Frame target_frame = getObjectKDLFrame(frame, parent);

    return moveApproachBackward(distance, target_frame, speed_scaling);

}

KDL::Frame TaskManager::getApproachVector(double distance, KDL::Frame target_frame)
{
    //KDL::Frame displ(KDL::Vector(0,0,0.2));
    KDL::Frame displ = KDL::Frame(target_frame.M)*KDL::Frame(KDL::Vector(0,0, -distance));

    KDL::Frame target_approach = target_frame;

    target_approach.p += displ.p;

    return target_approach;
}



bool TaskManager::moveApproachForward(double distance, KDL::Frame target_frame, double speed_scaling)
{
    resetTrajectory();

    target_frame = moveToPlanningFrame(target_frame);

    addWaypoint(getApproachVector(distance, target_frame));

    addWaypoint(target_frame);

    return executeCartesianPath(speed_scaling);
}

bool TaskManager::moveApproachBackward(double distance, KDL::Frame target_frame, double speed_scaling)
{
    resetTrajectory();

    target_frame = moveToPlanningFrame(target_frame);

    addWaypoint(target_frame);

    addWaypoint(getApproachVector(distance, target_frame));

    return executeCartesianPath(speed_scaling);
}

bool TaskManager::moveApproachBackward(double distance, double speed_scaling)
{
    resetTrajectory();

    addWaypoint(getApproachVector(distance));

    return executeCartesianPath(speed_scaling);
}

KDL::Frame TaskManager::getApproachVector(double distance)
{
    return getApproachVector(distance, getCurrentKDLFrame());
}




bool TaskManager::executeCircularPath(KDL::Frame target_frame, KDL::Frame axis_frame, double final_angle, double angle_step, double speed_scaling)
{
    resetTrajectory();

    visual_tools_->publishAxisLabeled(getPoseFromKDL(axis_frame,"world"), "joint_frame" , rvt::SMALL);

    target_frame = moveToPlanningFrame(target_frame);

    axis_frame = moveToPlanningFrame(axis_frame);


    //target_frame.p -= axis_frame.p;

    //KDL::Vector handle_relative_position = target_frame.p;

    KDL::Frame handle_relative_frame = axis_frame.Inverse()*target_frame;

    //double init_angle = atan2(handle_relative_position.y(), handle_relative_position.x());

    //double init_angle = atan2(handle_relative_frame.p.y(), handle_relative_frame.p.x());

    //ROS_INFO_NAMED("tutorial", "init_angle = %f", init_angle);


    for (double angle = angle_step; angle > final_angle; angle += angle_step) {
        KDL::Rotation rotation =  KDL::Rotation::RotZ(angle);

      //  ROS_INFO_NAMED("tutorial", "angle = %f", angle);

        //KDL::Frame displ(rotation*handle_orient, rotation*handle_position);

        KDL::Frame displ = KDL::Frame(rotation)*handle_relative_frame;

        addWaypoint(axis_frame*displ);
    }

    setPlanningTime(10.0);

    executeCartesianPath(speed_scaling);
}

bool TaskManager::moveApproachForward(double distance, double speed_scaling)
{
    resetTrajectory();

    addWaypoint(getApproachVector(-distance));

    return executeCartesianPath(speed_scaling);
}

bool TaskManager::moveJointRelativeByNumber(double displacement, int joint_number)
{
    if (joint_number < 1 || joint_number > 6) return false;

    std::vector<double> joint_values =  move_group->getCurrentJointValues();

    //for(int i = 0; i < 6; i++) ROS_INFO_NAMED("tutorial", "Joint angle %i: %f", i, joint_values[i]);

    joint_values[joint_number-1] += displacement;

    //for(int i = 0; i < 6; i++) ROS_INFO_NAMED("tutorial", "Joint angle %i: %f", i, joint_values[i]);

    resetTrajectory();

    move_group->setJointValueTarget(joint_values);

    planAndExecute();
}

bool TaskManager::planAndExecute()
{
    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //ROS_INFO_NAMED("tutorial", "Planning joint pose %s: %s", namedPose.c_str(), success ? "OK" : "FAILED");

    if(!success){
        ROS_INFO_NAMED("tutorial", "Unable to plan the requested motion: %s", success ? "OK" : "FAILED");

        return success;
    }

    success = (move_group->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //ROS_INFO_NAMED("tutorial", "Execution of joint pose %s: %s", namedPose.c_str(), success ? "OK" : "FAILED");

    if(!success){
        ROS_INFO_NAMED("tutorial", "Unable to plan the requested motion: %s", success ? "OK" : "FAILED");

    }

    return success;

}

void TaskManager::setJointConstraint(std::string joint, double position, double lower_value, double higher_value)
{
    moveit_msgs::JointConstraint jc;
    jc.joint_name = joint;
    jc.position = position;
    jc.tolerance_above = higher_value;
    jc.tolerance_below = lower_value;
    jc.weight = 1.0;
    moveit_msgs::Constraints path_constraints;
    path_constraints.joint_constraints.push_back(jc);
   // move_group->setPathConstraints(path_constraints);

//    moveit_msgs::OrientationConstraint ocm;

//      ocm.link_name = "gripper_extender_link";
//      ocm.header.frame_id = "ground_base1";
//      ocm.orientation.w = 1.0;
//      ocm.absolute_x_axis_tolerance = 0.0;
//      ocm.absolute_y_axis_tolerance = 0.0;
//      ocm.absolute_z_axis_tolerance = 0.1;
//      ocm.weight = 1.0;
//      //moveit_msgs::Constraints test_constraints;
//      path_constraints.orientation_constraints.push_back(ocm);
      move_group->setPathConstraints(path_constraints);
}

bool TaskManager::moveJointSpace(double distance, std::string frame, std::string parent, double speed_scaling)
{
    KDL::Frame target_frame = getObjectKDLFrame(frame, parent);

    return moveJointSpace(distance, target_frame, speed_scaling);
}

bool TaskManager::moveJointSpace(double distance, KDL::Frame target_frame, double speed_scaling)
{
    moveJointSpace(getApproachVector(-distance, target_frame), speed_scaling);

    return moveJointSpace(target_frame, speed_scaling);
}

bool TaskManager::moveJointSpace(std::string frame, std::string parent, double speed_scaling)
{
    KDL::Frame target_frame = getObjectKDLFrame(frame, parent);

    return moveJointSpace(target_frame, speed_scaling);
}

bool TaskManager::moveJointSpace(KDL::Frame target_frame, double speed_scaling)
{
    target_frame = moveToPlanningFrame(target_frame);

    move_group->setPoseTarget(getPoseFromKDL(target_frame, "world"));

    move_group->setNumPlanningAttempts(3);
    move_group->setPlanningTime(10);
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Planning: %s", success ? "OK" : "FAILED");
    success = (move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Executing plan: %s", success ? "OK" : "FAILED");

    return success;
}

void TaskManager::printJointValues()
{
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);

    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();
    //if you want to modify it
    //planning_scene::PlanningScenePtr scene = ps->diff();
    //scene->decoupleParent();

    // Get Joint Values
    // ^^^^^^^^^^^^^^^^
    // We can retreive the current set of joint values stored in the state for the Panda arm.
    std::vector<double> joint_values;
    //std::vector<double> joint_values =  move_group->getCurrentJointValues();

    //robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));

    //kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    //myplanning_scene->setCurrentState(*kinematic_state);

    robot_state::RobotState new_kinematic_state = ps->getCurrentStateNonConst();

    new_kinematic_state.copyJointGroupPositions(joint_model_group, joint_values);
    //std::vector<double> joint_values(myrobot_state->getJointPositions(joint_names));
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

}

void TaskManager::update()
{
    myrobot_state->update();
}

void TaskManager::checkCollision(collision_detection::CollisionRequest &collision_request, collision_detection::CollisionResult &collision_result)
{
    planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);

    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();

    auto robot_model_ptr = ps->getRobotModel();  // `planning_scene_` is a pointer to a moveit::planning_scene::PlanningScene instance
    collision_detection::CollisionRobotFCL crobot(robot_model_ptr);

    //getting Allowed Collision Matrix    
    collision_detection::AllowedCollisionMatrix acm = ps->getAllowedCollisionMatrix();
    
    //acm.print(std::cout);
    
    crobot.checkSelfCollision(collision_request, collision_result, ps->getCurrentStateNonConst(), acm);       

}

void TaskManager::distanceSelf(collision_detection::DistanceRequest &distance_request, collision_detection::DistanceResult &distance_result)
{
    planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);

    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();

    auto robot_model_ptr = ps->getRobotModel();  // `planning_scene_` is a pointer to a moveit::planning_scene::PlanningScene instance
    collision_detection::CollisionRobotFCL crobot(robot_model_ptr);  
    
    collision_detection::AllowedCollisionMatrix acm = ps->getAllowedCollisionMatrix();

    //acm.setEntry("ur10wrist_2_link","ur10forearm_link",true);
    
    distance_request.acm = &acm;
    
    //crobot.distanceSelf(distance_request, distance_result, ps->getCurrentStateNonConst());
    
   // ROS_INFO("Setting up a collision world for the trial meshes:");
    const collision_detection::WorldPtr trialWorld = ps->getWorldNonConst();//trialWorld(new collision_detection::World());
    collision_detection::CollisionWorldFCL trialCollisionWorld(trialWorld);
    //ROS_INFO("    Created trial collision world.");

    trialCollisionWorld.distanceRobot(distance_request, distance_result,  crobot, ps->getCurrentStateNonConst());
   
}
