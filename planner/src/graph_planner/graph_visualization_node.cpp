#include <ros/ros.h>
#include <thread>
#include "sea_planner/utility.h"
#include "sea_planner/Graph.h"
#include "sea_planner/Vertex.h"
#include "sea_planner/Edge.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher *pub_edge_marker;
sea_planner::Graph edge_set; 

void edgeHandler(const sea_planner::GraphConstPtr &edges){
    if(!edges->edges.empty()){
        edge_set.edges.clear();
        edge_set = *edges;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_visualization_node");
    ros::NodeHandle nh;

    // Publisher and Subscriber definition
    ros::Publisher pub_edge_marker_ = nh.advertise<visualization_msgs::Marker>("graph_edge_marker", 10);
    pub_edge_marker = &pub_edge_marker_;
    ros::Publisher pub_vertices_marker_ = nh.advertise<visualization_msgs::Marker>("graph_vertices_marker", 10);
    ros::Subscriber sub_edge_ = nh.subscribe<sea_planner::Graph>("graph_edge", 10, edgeHandler);

    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        visualization_msgs::Marker edge_marker, node_marker;
        // Marker init
        edge_marker.header.frame_id = edge_set.header.frame_id;
        edge_marker.header.stamp = ros::Time::now();
        edge_marker.type = edge_marker.LINE_LIST;
        edge_marker.action = edge_marker.ADD;
        edge_marker.lifetime = ros::Duration();
        edge_marker.color.a = 1.0;
        edge_marker.color.r = 0.0 / 255.0;
        edge_marker.color.g = 255.0 / 255.0;
        edge_marker.color.b = 255.0 / 255.0;
        edge_marker.scale.x = 0.015;
        edge_marker.scale.y = 0.015;
        edge_marker.scale.z = 0.015;
        edge_marker.pose.orientation.w = 1;
        edge_marker.points.clear();

        node_marker.header.frame_id = edge_set.header.frame_id;
        node_marker.header.stamp = ros::Time::now();
        node_marker.type = node_marker.SPHERE_LIST;
        node_marker.action = node_marker.ADD;
        node_marker.lifetime = ros::Duration();
        node_marker.color.a = 1.0;
        node_marker.color.r = 245.0/255.0;
        node_marker.color.g = 121.0/255.0;
        node_marker.color.b = 0.0/255.0;
        node_marker.scale.x = 0.08;
        node_marker.scale.y = 0.08;
        node_marker.scale.z = 0.08;
        node_marker.pose.orientation.w = 1;
        node_marker.points.clear();

        // Add edge data 
        for(int i=0; i<edge_set.edges.size(); i++){
            edge_marker.points.push_back(edge_set.edges[i].vertex_1.location);
            edge_marker.points.push_back(edge_set.edges[i].vertex_2.location);
        }
        for(int j=0; j<edge_set.vertices.size(); j++){
            node_marker.points.push_back(edge_set.vertices[j].location);
        }
        pub_edge_marker->publish(edge_marker);
        pub_vertices_marker_.publish(node_marker);
        r.sleep();
    }
    return 0;
}
