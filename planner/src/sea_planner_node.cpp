#include <ros/ros.h>
#include <thread>
// #include "sea_planner/rtrrt.h"
// #include "sea_planner/rtrrt_base.h"
#include "sea_planner/rtrrtp.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sea_planner_node");
    
    ros::NodeHandle nh;
    // Create planner Object
    ParamLoader param_loader;
    rtrrtp planner;
    std::cout << termcolor::bold << termcolor::green
              << "Planner Node Launched!"
              << termcolor::reset << std::endl;
    // ros::MultiThreadedSpinner threadNum(2);
    // ros::spin(threadNum);
    ros::spin();
    return 0;
}
