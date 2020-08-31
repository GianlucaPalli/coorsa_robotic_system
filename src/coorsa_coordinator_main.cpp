#include <ros/ros.h>
#include <coorsa_interface/CoorsaCoordinator.h>
#include <Eigen/Dense>
#include <iostream>



int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "coorsa_coordinator_main");
    ros::NodeHandle n;

    //coordinator class definition
    Eigen::Vector3f box_dimensions(0.315,0.23,0.210);
    coorsa_interface::CoorsaCoordinator coordinator(box_dimensions);

    //task loop definition
    char test_go;

    //go to start pose
    std::vector<double> start_pose;
    start_pose.resize(7);
    start_pose[0] = 0.3;
    start_pose[1] = 0.5;
    start_pose[2] = 0.6;
    start_pose[3] = -0.2158;
    start_pose[4] = 0.8034;
    start_pose[5] = -0.2706;
    start_pose[6] = -0.4843;

    std::vector<double> stop_pose;
    stop_pose.resize(7);
    stop_pose[0] = 0.3;
    stop_pose[1] = 0.5;
    stop_pose[2] = 0.6;
    stop_pose[3] = -0.2158;
    stop_pose[4] = 0.8034;
    stop_pose[5] = -0.2706;
    stop_pose[6] = -0.4843;
    
    std::cout<<"do you want to go to start pose?"<<std::endl;
    test_go=std::getchar();
    if (test_go=='y')
        coordinator.goToStartPose(start_pose,5.0);
    //coordinator.save_camera_transform();

    //set box detection exposure
    coordinator.setExposure(1000);
    //find new plane
    coordinator.detectNewPlane();
    std::cout<<"plane detected"<<std::endl;

    //detect packages on plane
    coordinator.detectPackagesOnPlane();
    std::cout<<"box detected"<<std::endl;

    std::cout<<"press enter to proceed"<<std::endl;
    getchar();
    getchar();

    //set line detection exposure
    coordinator.setExposure(80);


    //evaluate picking strategy
    coordinator.evaluatePickingPlan();
    std::cout<<"plan evaluated"<<std::endl;

    //perform picking on plane
    std::cout<<"perform picking on plane"<<std::endl;
    coordinator.performPickingOnPlane();

    std::cout<<"do you want to go to stop pose?"<<std::endl;
    test_go=std::getchar();
    if (test_go=='y')
        coordinator.goToStartPose(stop_pose,5.0);
    return 0;
}


