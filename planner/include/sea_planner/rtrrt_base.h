#ifndef RTRRT_BASE_H_
#define RTRRT_BASE_H_

#include <vector>
#include <geometry_msgs/Point.h>
#include "utility.h"

namespace rtrrt_ns{
    enum NodeState{
        ROOT=0,
        NORMAL=1,
        LEAF=2,
        OBSTACLE=3,
    };

    // RTRRT node
    struct Node{
        using Ptr = shared_ptr<Node>;

        size_t node_id;                         // Node unique id
        geometry_msgs::Point state;             // Node position
        
        Node::Ptr parent=NULL, prevParent=NULL;    // Parent node
        std::shared_ptr<struct Edge> endEdge;                          // The Edge connected with parant node
        std::list<std::shared_ptr<struct Edge>> startEdges;            // The Edge connected with child node      
        std::list<Node::Ptr> children;              // Child nodes
        int obstacle_orientation[8] = {0, 0, 0, 0, 0, 0, 0, 0};    // This array is used to certify obstacle node, if element num(=1) > threshold,
                                        //  this node is considered as obstacle

        float disToRoot;                        // The sum of branch length from root to this node in tree
        float gradientToRoot;                  // The sum of gradient from root to this node in tree
        float penaltyFactor = 1.0;
        NodeState nodeState;                    // Node state: ROOT, NORMAL, LEAF, OBSTACLE
        PointFrontier pcl_pt;                  // PCL point for calculating frontier

        Node(){}
        // Init Func
        Node(float x_, float y_,float z_, 
             float disToStart_, float gradientToRoot_, 
             Node::Ptr parent_ = NULL, NodeState nodeState_= NodeState::NORMAL, 
             float info_gain_ = 0.0)
        {
            state.x = x_;
            state.y = y_;
            state.z = z_;
            disToRoot = disToStart_;
            gradientToRoot = gradientToRoot_;
            parent = parent_;
            nodeState = nodeState_;
            pcl_pt.x = x_;
            pcl_pt.y = y_;
            pcl_pt.z = z_;
            pcl_pt.intensity = info_gain_;
        }

    };
    // RTRRT edge
    struct Edge{
        using Ptr = shared_ptr<Edge>;
        std::shared_ptr<struct Node> fromNode;               // Parent Node 
        std::shared_ptr<struct Node> toNode;                 // Child Node
        float length;                        // The length of edge
        float elevation_diff;                // The difference of Z value
        float gradient;                      // The gradient of Edge, rad form
        
        Edge(){};
        // Init
        Edge(std::shared_ptr<struct Node> fromNode_, std::shared_ptr<struct Node> toNode_){
            fromNode = fromNode_;
            toNode = toNode_;
            length = util::distance(fromNode_->state.x, toNode_->state.x,
                                    fromNode_->state.y, toNode_->state.y);
                                 
        }
        bool operator == (const Edge & edge){
            if (this->fromNode == edge.fromNode &&
                this->toNode == edge.toNode)
            {
                return true;
            }
            return false;
        }
    };

    typedef struct Node Node;
    typedef struct Edge Edge;
    // typedef Node_Ptr Node::Ptr;
    // typedef Edge_Ptr Edge::Ptr;


    // inline bool rtrrt_ns::Edge::operator==(const rtrrt_ns::Edge& edge) const
    // {
    //     if (this->fromNode == edge.fromNode &&
    //         this->toNode == edge.toNode)
    //     {
    //         return true;
    //     }
    //     return false;
    // }

}

#endif  // RTRRT_BASE_H_