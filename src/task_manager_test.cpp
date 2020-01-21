
#include "coorsa_interface/taskmanager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Code_Example");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);

    spinner.start();

    TaskManager task_manager;
/*
task_manager.resetTrajectory();
 KDL::Rotation tool_entering_orient = KDL::Rotation::RPY(-3.14, 0, 0);
    KDL::Frame door_front(tool_entering_orient, KDL::Vector(0.4,0.0,0.3));*/
    // Adding/Removing Objects and Attaching/Detaching Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
/*
    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 2;
    primitive.dimensions[2] = 0.4;

    //Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = -0.01;

   // task_manager.addCollisionObject("box1", primitive, box_pose);

    // Define a box to add to the world.
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.5;

    //Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0.1;
    box_pose.position.z = 0.19;

    task_manager.addCollisionObject("box2", primitive, box_pose);

    // Define a box to add to the world.
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 1;

    //Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0.1;
    box_pose.position.z = 0.44;

    task_manager.addCollisionObject("box3", primitive, box_pose);


    //std::vector<std::string> pose_vector = {"test","up","up front","arm ready"};
    std::vector<std::string> pose_vector = {"test","arm ready"};

    for (int i = 0; i < pose_vector.size(); i++){
        task_manager.recallJointConfig(pose_vector[i]);
        //ros::Duration(5).sleep();
    }

    task_manager.setJointConstraint("rg2_joint", 0.0, 0.3, 0.3);

    KDL::Frame body_frame = task_manager.getObjectKDLFrame("WMEU_1_0::Body", "world");

    task_manager.moveApproachForward(0.2, "WMEU_1_0::Handle", "world", 0.2);

    task_manager.resetTrajectory();

    task_manager.setJointConstraint("rg2_joint", 0.0, 0.3, 0.3);

    KDL::Frame handle_frame = task_manager.getObjectKDLFrame("WMEU_1_0::Handle", "world");

    //KDL::Vector door_joint(-0.22, -0.015-0.228, 0.01); // -0.22 -0.015 0.01
    //KDL::Frame door_joint_frame(KDL::Rotation::RPY(0, 0, 0), body_frame*door_joint);
    
    KDL::Frame door_joint_frame = task_manager.getObjectKDLFrame("WMEU_1_0::Door_joint_dummy", "world");

    //task_manager.executeCircularPath(handle_frame, door_joint_frame, -0.5, -0.1, 0.01);

    //task_manager.moveApproachBackward(0.4, 0.2);

    //KDL::Frame good_frame = task_manager.getCurrentKDLFrame();

    //task_manager.moveApproachForward(0.2, handle_frame, 0.2);

    task_manager.executeCircularPath(handle_frame, door_joint_frame, -1.5, -0.1, 0.1);

    task_manager.moveApproachBackward(0.4, 0.2);

    task_manager.recallJointConfig("arm ready");

    task_manager.recallJointConfig("entering");

    //task_manager.resetTrajectory();

//    visual_tools_->prompt("next step");

    KDL::Rotation tool_entering_orient = KDL::Rotation::RPY(-1.57, -1.57, 0);
    KDL::Frame door_front(tool_entering_orient, body_frame*KDL::Vector(-0.0,0.0,0.0));

//    KDL::Frame entering_frame = task_manager.getObjectKDLFrame("WMEU_1_0::Body", "world");

//    entering_frame.p.z(0.43);

    task_manager.moveApproachForward(0.5, door_front, 0.5);

    double entering_depth = 0.0;

    door_front.p.y(door_front.p.y() + entering_depth);

    //KDL::Frame entering(tool_entering_orient, KDL::Vector(0.0,entering_depth,0.35));

    task_manager.moveApproachForward(entering_depth, door_front, 0.05);

    tool_entering_orient = KDL::Rotation::RPY( 0, 0, 0);
    door_front = KDL::Frame(tool_entering_orient, KDL::Vector(-0.0,entering_depth,0.63));

    task_manager.resetTrajectory();

    task_manager.addWaypoint(door_front);

    task_manager.executeCartesianPath(0.1);

//    bool success = task_manager.moveJointRelativeByNumber(-1.57, 4);

//    ROS_INFO_NAMED("tutorial", "Planning: %s", success ? "OK" : "FAILED");

//    success = task_manager.moveJointRelativeByNumber(1.57, 5);

//    ROS_INFO_NAMED("tutorial", "Planning: %s", success ? "OK" : "FAILED");

   // task_manager.visual_tools_->prompt("next step");


//    task_manager.moveApproachBackward(0.2, 0.05);

    task_manager.moveApproachBackward(0.6, 0.5);

//    task_manager.visualizePose(door_front, "door_front");

//    task_manager.executeCartesianPath();


    //visual_tools_->prompt("next step");
    */
   ros::spin();
    ros::shutdown();

    return 0;

}
