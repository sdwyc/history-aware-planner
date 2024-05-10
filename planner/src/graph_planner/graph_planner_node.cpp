#include <ros/ros.h>
#include <thread>
#include "sea_planner/utility.h"
#include "graph_planner/graph_planner.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_planner_node");
    
    ros::NodeHandle nh;
    // Create planner Object
    ParamLoader param_loader;
    graph_planner planner;
    std::cout << termcolor::bold << termcolor::green
              << "Graph Planner Node Launched!"
              << termcolor::reset << std::endl;
    ros::MultiThreadedSpinner threadNum(2);
    ros::spin(threadNum);
    return 0;
}
