#ifndef RTRRT_H_
#define RTRRT_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <utility>
#include <map>
#include <mutex>
#include <list>
#include <queue>
#include <random>
#include <cstdlib>
#include "rtrrt_base.h"
#include "kd_search/kd_search.h"

using namespace std;
using namespace util;
using namespace rtrrt_ns;
using namespace termcolor;

typedef pair<rtrrt_ns::Node::Ptr, rtrrt_ns::Node::Ptr> NodePair;
typedef unordered_map<rtrrt_ns::Edge::Ptr, NodePair> Tree;
typedef vector<rtrrt_ns::Node::Ptr> Nodes;
typedef list<rtrrt_ns::Node::Ptr> NodesList;
typedef list<rtrrt_ns::Edge::Ptr> Edges;

class rtrrt : public ParamLoader
{
    private:
        // ROS parameter
        ros::NodeHandle nh;
        string global_frame;
        string base_frame;
        // ROS publisher for visualization
        ros::Publisher pub_rtrrt_edge;
        ros::Publisher pub_rtrrt_node;
        ros::Publisher pub_removed_node;
        ros::Publisher pub_removed_edge;
        ros::Publisher pub_root_marker;
        ros::Publisher pub_leaf_node;
        ros::Publisher pub_frontier_node;
        ros::Publisher pub_goal_marker;
        ros::Publisher pub_subgoal_marker;
        ros::Publisher pub_path;
        ros::Publisher pub_range_map;
        ros::Publisher pub_sample_node;
        ros::Publisher pub_obstacle_node;
        ros::Publisher treePub;
        ros::Publisher treeNodesPub;
        // ROS subscriber
        ros::Subscriber sub_graph_vertice;
        ros::Subscriber sub_robot_pose;

        ros::Timer TimerVis;
        ros::Time last_time;

        rtrrt_ns::Node sampleNode;
        rtrrt_ns::Node newNode;
        queue<rtrrt_ns::Node::Ptr> q_rewire_from_root;  // Rewiring-from-rand queue
        queue<rtrrt_ns::Node::Ptr> q_rewire_from_rand;  // Rewiring-from-root queue
        std::mt19937_64 gen_;
        std::uniform_real_distribution<double> uniform_rand_;
        std::uniform_real_distribution<double> uniform_rand1_;
        float path_length;
        grid_map::GridMap sensor_range_map;
        float map_x;
        float map_y;
        float map_resolution;
        bool receivedGridMap;
        bool updatedPose;
        // KDTree
        //! for vertex_id sorting
        unordered_map<size_t, rtrrt_ns::Node::Ptr> id_nodes; // id-node pair
        std::shared_ptr<KDTreePt::PointCloud<float>> kdtree_pts; // Total point list (include removed point)
        std::shared_ptr<nanoKDTree> kdtree_;

        unordered_map<size_t, rtrrt_ns::Node::Ptr> obs_id_nodes; // id-node pair for obstable nodes
        std::shared_ptr<KDTreePt::PointCloud<float>> obstacle_pts; // Total point list (include removed point)
        std::shared_ptr<nanoKDTree> obstacleKdtree_;

        pcl::KdTreeFLANN<PointFrontier>::Ptr pcl_kdtree;

        int skipCountRewiring = 3; // Skip certain num update step to control rewiring-from-root freq
        std::mutex mtx;

        pcl::PointCloud<PointFrontier>::Ptr graphVertices;
        pcl::PointCloud<PointFrontier>::Ptr treeNodes;


    public:
        Tree tree;
        NodesList node_set;
        Nodes last_removed_nodes;
        Edges last_removed_edges;
        Nodes obstacle_node_set;
        Edges edge_set;
        rtrrt_ns::Node::Ptr root;
        NodesList leaf_nodes;
        geometry_msgs::Pose goal;
        geometry_msgs::Point *current_target;   // Current target node in the tree
        rtrrt_ns::Node::Ptr current_target_node;   // Current target node in the tree
        util::Pose robot_pose;
        bool newGoalReceived;
        unordered_map<rtrrt_ns::Node::Ptr, float> frontierNodes;    // Frontier nodes and its information gain
        grid_map::GridMap full_grid_map;
        // ROS subscriber
        ros::Subscriber sub_grid_map;

        rtrrt();
        ~rtrrt();
        void init();    // Initialization
        void clearAll();   // Clear whole tree data and cache
        rtrrt_ns::Node sample();  // Sample point using rrt, informed rrt, random approached within space  
        bool checkNewNode(rtrrt_ns::Node x_new);    // Check new node is valid? or not?
        bool checkEdge(rtrrt_ns::Edge edge); // Check edge is valid? or not?
        rtrrt_ns::Node steer(rtrrt_ns::Node x_s); // Steer from parent node to sample, and prolong a certain length
        bool addNodeEdge(rtrrt_ns::Node x_new, rtrrt_ns::Node::Ptr x_parent);  // Add node, edge, and N-E pair to current tree
        void reselectParent(rtrrt_ns::Node::Ptr x_new);   // Reselect parent node within a certain range
        void rewireRandomNode(); // Rewire the branch at one of node in tree
        void rewireFromRoot();   // Rewire the whole tree 
        void executeTreeUpdate();   // Execute one step for updating tree
        void updateRobotPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &new_pose); // Update robot pose through listening TF
        void publishTree(); // Vistualize tree in rviz
        rtrrt_ns::Node::Ptr findTarget();  // Find the best target node near the goal
        void changeRoot(rtrrt_ns::Node::Ptr new_node);  // Change current root to new node 
        void pruneTree(rtrrt_ns::Node::Ptr new_center); // Prune relevant edges and node as robot movement
        void updateGoal(geometry_msgs::PoseStamped goal_point);  // Update Goal position from external input
        void getFrontierNode(); // Get frontier node from all rtrrt leaf nodes
        Nodes getNeiborNodes(float point_x, float point_y, float point_z, float range);  // Get neibor nodes set of given point
        rtrrt_ns::Node::Ptr getNearestNode(float point_x, float point_y);  // Get nearest node of given point
        rtrrt_ns::Node::Ptr getObstacleNode(float point_x, float point_y);  // Get nearest obstacle node of given point
        void updateOrientationArray(rtrrt_ns::Node new_node, rtrrt_ns::Node::Ptr rrt_node);  // Update the value of node's orientation array
        void publishRemovedItem(); // Publish last removed node and edges
        void gridMapHandler(const grid_map_msgs::GridMapConstPtr &raw_map);  // Grid map callback func
        void graphVerticesHandler(const sensor_msgs::PointCloud2ConstPtr &vertices_msgs); // Received graph vertices from graph_planner
        void visualizationCb();   // Timer callback for visualize the whole tree(node + edge)
        void visualizeSample(rtrrt_ns::Node sample_node);   // Visualization sample point pose
        
};

#endif  // RTRRT_H_