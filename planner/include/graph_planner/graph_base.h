#ifndef GRAPH_BASE_H_
#define GRAPH_BASE_H_

#include <vector>
#include <geometry_msgs/Point.h>
#include <map>
#include "sea_planner/utility.h"

namespace graph_ns{
    // enum NodeState{
    //     ROOT=0,
    //     NORMAL=1,
    //     LEAF=2,
    //     OBSTACLE=3,
    // };

    // Graph Vertex
    struct Vertex{
        using Ptr = shared_ptr<Vertex>;
        size_t vertex_id;   // Unique ID
        
        geometry_msgs::Point location;             // Vertex position
        
        map<size_t, std::shared_ptr<struct Edge>> vertex_edge_list; // Relative connection with vertices and edges
        
        float information_gain; // information gain for reselection
        
        PointFrontier pcl_pt;   // PCL point for calculating frontier
        
        Vertex(){}
        // Init Func
        Vertex(sea_planner::Vertex vertex_msg)
        {
            location.x = vertex_msg.location.x;
            location.y = vertex_msg.location.y;
            location.z = vertex_msg.location.z;
            information_gain = vertex_msg.information_gain;
        }

        Vertex(float x_, float y_,float z_, float info_gain_=0.0)
        {
            location.x = x_;
            location.y = y_;
            location.z = z_;
            information_gain = info_gain_;
            pcl_pt.x = x_;
            pcl_pt.x = y_;
            pcl_pt.x = z_;
            pcl_pt.intensity = info_gain_;
        }

    };
    // Graph Edge
    struct Edge{
        using Ptr = shared_ptr<Edge>;

        std::shared_ptr<struct Vertex> Vertex_1;               // Relative vertex 
        std::shared_ptr<struct Vertex> Vertex_2;               
        float length;                        // The length of edge
        float elevation_diff;                // The difference of Z value
        float gradient;                      // The gradient of Edge, rad form
        
        Edge(){};
        // Init
        Edge(std::shared_ptr<struct Vertex> Vertex_1_, std::shared_ptr<struct Vertex> Vertex_2_){
            Vertex_1 = Vertex_1_;
            Vertex_2 = Vertex_2_;
            length = util::distance(Vertex_1->location.x, Vertex_2->location.x,
                                    Vertex_1->location.y, Vertex_2->location.y);
                                 
        }
        bool operator == (const Edge & edge){
            if (this->Vertex_1 == edge.Vertex_1 &&
                this->Vertex_2 == edge.Vertex_2)
            {
                return true;
            }
            return false;
        }
    };

    typedef struct Vertex Vertex;
    typedef struct Edge Edge;
}

#endif  // GRAPH_BASE_H_