#ifndef UTILITY_H_
#define UTILITY_H_

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <flann/flann.hpp>

#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/GridMap.h"
#include "termcolor.hpp"

#define SQ(x) ((x) * (x))
#define TAN60 1.73205081

using namespace grid_map;
using namespace std;

typedef pcl::PointXYZI PointFrontier;

const float inf = std::numeric_limits<float>::infinity();
const int INF = std::numeric_limits<int>::max();

const float Eta = 100.0;	// Cost value magnification
const float informationRadius = 1.0; // Check node information gain
const float inflationRadius = 0.8; // The inflation radius for obstacle node
const float maxPenalty = 10.0; // The maximum of Penalty factor
const float regressionFactor = 5.0; // Control the penalty's slope

class ParamLoader
{
public:
	string prefix = "seap";
    ros::NodeHandle nh;

    string baseFrame;
    string globalFrame;

    float robotHeight;

    float epsilon; // rtrrt grow rate
    float minNodeDistance; // The minimum distance between nodes
    float neiborRangeThre; // nodes less than this value are considered neibor nodes
    int minTreeNodeNum;	// The minimum number of tree nodes need by navigation
    int minTreeEdgeNum;	// The minimum number of tree edges need by navigation
    int maxNodesNum; // The maximum node number of whole tree
    float alpha;	// rtrrt sample rate threshold for robot->goal
    float beta; // rtrrt sample rate threshold for random sample
    float Gamma; // rtrrt sample rate threshold for sensor range sample
    float allowRewiringFromRand;	// Single rewiring-from-rand step time constrain
    float allowRewiringFromRoot;	// Single rewiring-from-root step time constrain
    float gradientDiffThre;	// Maximum gradient threshold for edge, rad form
    float targetTolerance; // The tolerance of consider node as target node
    float goalTolerance; // The tolerance of arriving goal
    float waypointTolerance; // The tolerance of arriving waypoint
    float changRootTolerance; // The tolerance of arriving waypoint
    float informationThre; // Check node information gain
    float frontierRadius; // Check node's surrounding whether exist NaN grid
    float frontierDisThre; // The leaf nodes less than this value aren't considered as frontier
    float exploredAreaRadius; // The leaf nodes' circle check radius for avoiding explored area
    int exploredAreaThre; // The leaf nodes' circle number threshold of existed nodes
    float findNearNodeThre; // The distance threshold to final goal in order to select navigation type
    int obstacleSectorNumThre; // If one node orientation_array element(=1) greater than this value, considered obstacle node

    // Load Parameters
    ParamLoader()
    {
		// Frames
        nh.param<string>(prefix+"/robotFrame", baseFrame, "base_link");
        nh.param<string>(prefix+"/globalFrame", globalFrame, "map");

		nh.param<float>(prefix+"/RobotHeight", robotHeight, 0.11);

		// nh.param<float>(prefix+"/Eta", Eta, 100.0);
        nh.param<float>(prefix+"/epsilon", epsilon, 0.8);
        nh.param<float>(prefix+"/minNodeDistance", minNodeDistance, 0.2);
        nh.param<float>(prefix+"/neiborRangeThre", neiborRangeThre, 1.2);
        nh.param<float>(prefix+"/alpha", alpha, 0.1);
        nh.param<float>(prefix+"/beta", beta, 1.4);
        nh.param<float>(prefix+"/Gamma", Gamma, 0.5);
        nh.param<float>(prefix+"/allowRewiringFromRand", allowRewiringFromRand, 2.0);
        nh.param<float>(prefix+"/allowRewiringFromRoot", allowRewiringFromRoot, 4.0);
        nh.param<float>(prefix+"/gradientDiffThre", gradientDiffThre, 7.0);
		gradientDiffThre *= M_PI/180;

        nh.param<float>(prefix+"/targetTolerance", targetTolerance, 0.4);
        nh.param<float>(prefix+"/goalTolerance", goalTolerance, 0.5);
        nh.param<float>(prefix+"/waypointTolerance", waypointTolerance, 0.4);
        nh.param<float>(prefix+"/changRootTolerance", changRootTolerance, 0.5);
        nh.param<float>(prefix+"/informationThre", informationThre, 0.3);
        nh.param<float>(prefix+"/frontierRadius", frontierRadius, 1.0);
        nh.param<float>(prefix+"/frontierDisThre", frontierDisThre, 3.0);
        nh.param<float>(prefix+"/exploredAreaRadius", exploredAreaRadius, 0.8);
        nh.param<float>(prefix+"/findNearNodeThre", findNearNodeThre, 2.0);
        nh.param<int>(prefix+"/minTreeNodeNum", minTreeNodeNum, 3600);
        nh.param<int>(prefix+"/minTreeEdgeNum", minTreeEdgeNum, 500);
        nh.param<int>(prefix+"/maxNodesNum", maxNodesNum, 100000);
        nh.param<int>(prefix+"/exploredAreaThre", exploredAreaThre, 12);
        nh.param<int>(prefix+"/obstacleSectorNumThre", obstacleSectorNumThre, 2);

        usleep(100);	// Wait 100ms for loading
    }
};

namespace util{
	// Pose in 3d space
	struct Pose
	{
		float x;
		float y;
		float z;
		float roll;
		float pitch;
		float yaw;
	};

	// Get Euclidean Distance of two points
	float distance(float x1, float x2, float y1, float y2);

	// Get Z value of specific position
	float getElevation(float x, float y, grid_map::GridMap elevation_map, string layer="elevation");

	// Get gradient of two point
	float gradient(float distance, float z_diff);

	// Get cost value of given information
	float costFunc(float sub_distance, float distance_heuristic , float sub_gradient, float total_distance, float total_gradient);

	// Get leaf node information gain so that consider whether is frontier node
	float getInfoGain(float node_x, float node_y, grid_map::GridMap elevation_map, string layer="elevation");

	// Get rrt node obstacle orientation array element(==1)
	int getOrientation(const int arr[8]);
	
	// Get node penalty factor val using sigmoid function
	float getPenalty(const float min_distance_to_obs);

    template<typename T>
    sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        if (thisPub.getNumSubscribers() != 0)
            thisPub.publish(tempCloud);
        return tempCloud;
}

}

#endif  // UTILITY_H_
