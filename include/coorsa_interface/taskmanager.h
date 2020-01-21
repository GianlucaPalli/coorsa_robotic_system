#ifndef TASKMANAGER_H
#define TASKMANAGER_H


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/JointConstraint.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

#include <moveit_msgs/AllowedCollisionMatrix.h>


#include <ros/ros.h>
#include <gazebo_msgs/GetLinkState.h>

#include <tf2/convert.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_bullet/tf2_bullet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <kdl_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

class TaskManager {

public:

    TaskManager(std::string group = "manipulator");

    void addCollisionObject(std::string name, shape_msgs::SolidPrimitive primitive, geometry_msgs::Pose box_pose);
    bool recallJointConfig(std::string namedPose);

    geometry_msgs::TransformStamped getObjectTransformStamped(std::string child, std::string parent);
    KDL::Frame getObjectKDLFrame(std::string child, std::string parent);

    void setPlanningFrame(std::string parent = "world") {planning_KDLFrame = getObjectKDLFrame(planning_frame, parent); }

    KDL::Frame moveToPlanningFrame(KDL::Frame target_frame){ return planning_KDLFrame.Inverse()*target_frame;}

    void setStartStateAsCurrent();

    void clearWaypoints();
    void addWaypoint(KDL::Frame waypoint, std::string parent = "world");

    void addWaypoint(geometry_msgs::Pose waypoint);

    void setPlanningTime(double time);

    void resetTrajectory(void);

    KDL::Frame getApproachVector(double distance, KDL::Frame target_frame);

    KDL::Frame getApproachVector(double distance);

    bool moveApproachForward(double distance, std::string frame, std::string parent = "world", double speed_scaling = 1.0);

    bool moveApproachForward(double distance, KDL::Frame target_frame, double speed_scaling = 1.0);

    bool moveApproachForward(double distance, double speed_scaling = 1.0);


    bool moveJointSpace(double distance, std::string frame, std::string parent = "world", double speed_scaling = 1.0);

    bool moveJointSpace(double distance, KDL::Frame target_frame, double speed_scaling = 1.0);

    bool moveJointSpace(double distance, double speed_scaling = 1.0);

    bool moveJointSpace(std::string frame, std::string parent = "world", double speed_scaling = 1.0);

    bool moveJointSpace(KDL::Frame target_frame, double speed_scaling = 1.0);

    bool moveApproachBackward(double distance, std::string frame, std::string parent = "world", double speed_scaling = 1.0);

    bool moveApproachBackward(double distance, KDL::Frame target_frame, double speed_scaling = 1.0);

    bool moveApproachBackward(double distance, double speed_scaling = 1.0);
    
    bool moveJointRelativeByNumber(double displacement, int joint_number);

    bool executeCartesianPath(double speed_scaling = 1.0, double acceleration_scaling = 1.0, double jump_threshold = 0.0, double eef_step = 0.01);

    bool executeCircularPath(KDL::Frame target_frame, KDL::Frame axis_frame, double final_angle, double angle_step = 0.1, double speed_scaling = 1.0);

    bool planAndExecute();

    void setJointConstraint(std::string joint, double position, double lower_value, double higher_value);

    void printJointValues();

    void update();

    geometry_msgs::Pose getCurrentPose();

    geometry_msgs::PoseStamped getCurrentPoseStamped();

    KDL::Frame getCurrentKDLFrame();

    void visualizePose(geometry_msgs::Pose pose, std::string label = "pose");

    geometry_msgs::Pose getPoseFromKDL(KDL::Frame frame, const std::string& parent);

    void checkCollision(collision_detection::CollisionRequest &collision_request, collision_detection::CollisionResult &collision_result);

    void distanceSelf(collision_detection::DistanceRequest &distance_request, collision_detection::DistanceResult &distance_result);

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    //planning_scene::PlanningScene* myplanning_scene;
    
    //for visualizationa of collision points of rviz (added by Apurva for testing)
    ros::Publisher marker_pub;

private:
    std::string planning_group;

    std::string planning_frame;

    KDL::Frame planning_KDLFrame;

    std::string ee_link;

    std::vector<geometry_msgs::Pose> waypoints;

    Eigen::Affine3d text_pose;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;


    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr robot_model;

    robot_trajectory::RobotTrajectory* robot_trajectory;

    robot_state::RobotState* myrobot_state;

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface* move_group;

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group;

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    moveit_msgs::RobotTrajectory trajectory;

    const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";
    
    //for visualizationa of collision points of rviz (added by Apurva for testing)
    //ros::Publisher marker_pub;
    visualization_msgs::MarkerArray marker_array;
    
};

#endif // TASKMANAGER_H

