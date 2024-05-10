#ifndef GRAPH_PLANNER_H_
#define GRAPH_PLANNER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <utility>
#include <map>
#include <list>
#include <limits>  
#include <queue>
#include <random>
#include <cstdlib>
#include "sea_planner/Vertex.h"
#include "sea_planner/Edge.h"
#include "sea_planner/Graph.h"
#include "sea_planner/get_path.h"
#include "graph_base.h"
#include "kd_search/kd_search.h"

using namespace std;
using namespace util;
using namespace termcolor;

typedef vector<graph_ns::Vertex::Ptr> Vertices;
typedef vector<graph_ns::Edge::Ptr> Edges;

class graph_planner : public ParamLoader
{
    private:
        // ROS parameter
        ros::NodeHandle nh;
        string global_frame;
        string base_frame;
        // ROS publisher for visualization
        ros::Publisher pub_root_marker;
        ros::Publisher pub_frontier_node;
        ros::Publisher pub_subgoal_marker;
        ros::Publisher pub_path;
        ros::Publisher pub_edge;
        ros::Publisher pub_vertices;
        ros::Subscriber sub_goal;
        ros::Subscriber sub_robotpose;
        ros::ServiceServer planningFinder;
        ros::ServiceServer nearestFinder;

        ros::Timer poseUpdater;

        std::vector<sea_planner::Vertex> rawVertices;
        ros::Publisher treePub;
        std::mt19937_64 gen_;
        std::uniform_real_distribution<double> uniform_rand_;
        std::uniform_real_distribution<double> uniform_rand1_;
        unordered_map<size_t, graph_ns::Vertex::Ptr> id_vertex;
        float path_length;
        float map_x;
        float map_y;
        float map_resolution;
        bool receivedNewVertices;
        bool receivedGridMap;
        bool updatedPose;
        bool homeSaved;
        // KDTree
        //! for vertex_id sorting
        std::shared_ptr<KDTreePt::PointCloud<float>> kdtree_pts; // Total point list (include removed point)
        std::shared_ptr<nanoKDTree> kdtree_;

        pcl::PointCloud<PointFrontier>::Ptr graphVertices;
        pcl::PointCloud<PointFrontier>::Ptr treeNodes;

    public:
        Vertices node_set;
        Edges edge_set;
        graph_ns::Vertex::Ptr origin;
        geometry_msgs::Pose goal;
        geometry_msgs::Point *current_target;   // Current target node in the tree
        graph_ns::Vertex::Ptr current_target_node;   // Current target node in the tree
        util::Pose robot_pose;
        bool newGoalReceived;
        unordered_map<graph_ns::Vertex::Ptr, float> GlobalFrontier;    // Frontier nodes and its information gain
        grid_map::GridMap full_grid_map;
        // ROS subscriber
        ros::Subscriber sub_grid_map;
        ros::Subscriber sub_new_vertices;
        ros::Subscriber sub_local_frontier;

        graph_planner();
        ~graph_planner();
        void init();    // Initialization
        void clearAll();   // Clear whole graph data and cache
        bool removeInvlidVertex(Vertices new_vertices);    // Check new node whether has edge connection? if not, remove it
        bool checkEdge(graph_ns::Edge edge, graph_ns::Vertex::Ptr v1, graph_ns::Vertex::Ptr v2); // Check edge is valid? or not?
        bool addEdge(graph_ns::Vertex::Ptr x_new, graph_ns::Vertex::Ptr x_parent);  // Add node, edge
        bool addVertex(vector<sea_planner::Vertex> raw_vertices, Vertices& new_vertices);    // Add new vertices to kdtree
        void executeGraphUpdate();  // Execute one step for updating tree
        void findPath(); // Find global path with given two points
        graph_ns::Vertex::Ptr findTarget();  // Find the best target node near the goal
        void updateGoal(geometry_msgs::PoseStamped goal_point);  // Update Goal position from external input
        void getFrontierVertex(); // Get frontier node from all rtrrt leaf nodes
        void getNeiborVertex(float query_pt[3], float radius, vector<graph_ns::Vertex::Ptr>& neibor_vertices);  // Get neibor nodes set of given point
        void publishEdges();  // Publisher all edge for Visulization
        graph_ns::Vertex::Ptr getNearestVertex(float query_pt[3]);  // Get nearest node of given point
        Vertices findBestPath(graph_ns::Vertex::Ptr x_start, graph_ns::Vertex::Ptr x_end); // Find the Dijistra best path from start to end
        void gridMapHandler(const grid_map_msgs::GridMapConstPtr &raw_map);  // Grid map callback func
        void newVerticesHandler(const sea_planner::GraphConstPtr &vertices);  // New vertices callback func
        void goalHandler(const visualization_msgs::MarkerConstPtr &goal_mark);  // Final goal callback func
        void updateRobotPoseHandle(const geometry_msgs::PoseWithCovarianceStampedConstPtr &new_pose); // Update robot pose through listening TF
        void treeNodesHandler(const sensor_msgs::PointCloud2ConstPtr &node_msgs); // Received local frontiers from rtrrt
        graph_ns::Vertex::Ptr getNearestVertex(float point_x, float point_y); // Find the nearest vertex in the graph
        bool findPath(sea_planner::get_path::Request& req_target,
                    sea_planner::get_path::Response& res_path);     // Find a best path through graph
        bool getNearest(sea_planner::get_path::Request& req_target,
                    sea_planner::get_path::Response& res_path);     // Find a best path through graph

};

#endif  // GRAPH_PLANNER_H_