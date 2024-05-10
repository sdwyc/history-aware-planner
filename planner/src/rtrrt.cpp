#include <unordered_set>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "sea_planner/rtrrt.h"

using namespace rtrrt_ns;
using namespace termcolor;

void rtrrt::init(){
	root.reset();
	current_target = NULL;
	current_target_node.reset();
	node_set.clear();
	edge_set.clear();
	leaf_nodes.clear();
	tree.clear();
    global_frame = "map";
    base_frame = "base_link";
	receivedGridMap = false;
    updatedPose = false;
	newGoalReceived = false;
	robot_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	// Tree initialization
	// Kdtree Init
	kdtree_pts.reset(new KDTreePt::PointCloud<float>());
    kdtree_.reset(new nanoKDTree(3, 10, kdtree_pts));
    obstacle_pts.reset(new KDTreePt::PointCloud<float>());
    obstacleKdtree_.reset(new nanoKDTree(3, 10, obstacle_pts));
	pcl_kdtree.reset(new pcl::KdTreeFLANN<PointFrontier>());

	// pcl cloud init
	graphVertices.reset(new pcl::PointCloud<PointFrontier>());
	treeNodes.reset(new pcl::PointCloud<PointFrontier>());

	// Publisher init
	pub_rtrrt_edge = nh.advertise<visualization_msgs::Marker>("/rtrrt_edges", 10);
	pub_rtrrt_node = nh.advertise<visualization_msgs::Marker>("/rtrrt_nodes", 10);
	pub_removed_edge = nh.advertise<visualization_msgs::Marker>("/rtrrt_last_removed_edges", 10);
	pub_removed_node = nh.advertise<visualization_msgs::Marker>("/rtrrt_last_removed_nodes", 10);
	pub_root_marker = nh.advertise<visualization_msgs::Marker>("/rtrrt_root", 10);
	pub_leaf_node = nh.advertise<visualization_msgs::Marker>("/leaf_nodes", 10);
	pub_frontier_node = nh.advertise<visualization_msgs::Marker>("/frontier_nodes", 10);
	pub_path = nh.advertise<nav_msgs::Path>("/rtrrt_path",10);
	pub_goal_marker = nh.advertise<visualization_msgs::Marker>("/goal_marker",10);
	pub_subgoal_marker = nh.advertise<visualization_msgs::Marker>("/subgoal_marker",10);
	pub_range_map = nh.advertise<grid_map_msgs::GridMap>("range_grid_map",10);
	pub_sample_node = nh.advertise<visualization_msgs::Marker>("sample_node", 10);
	pub_obstacle_node = nh.advertise<visualization_msgs::Marker>("obstacle_node", 10);
	treeNodesPub = nh.advertise<sensor_msgs::PointCloud2>("tree_nodes", 10);
	// Subscriber init
	sub_grid_map = nh.subscribe("/elevation_mapping/elevation_map", 1, &rtrrt::gridMapHandler, this);
	sub_graph_vertice = nh.subscribe("graph_vertices", 1, &rtrrt::graphVerticesHandler, this);
	sub_robot_pose = nh.subscribe("base_link_pose", 1, &rtrrt::updateRobotPose, this);
	// TimerVis = nh.createTimer(ros::Duration(0.2), &rtrrt::visualizationCb, this);
	last_time = ros::Time::now();
}

rtrrt::rtrrt(){
	init();
	ROS_INFO("RTRRT initialization accomplished");
}

rtrrt::~rtrrt(){
	clearAll();
}

void rtrrt::clearAll(){
	root = NULL;
	id_nodes.clear();
    leaf_nodes.clear();
	node_set.clear();
	edge_set.clear();
	tree.clear();
    kdtree_.reset(new nanoKDTree(3, 10, kdtree_pts));
}

rtrrt_ns::Node rtrrt::sample(){
	rtrrt_ns::Node sample_node;
	sample_node.state.z = 0.0;
	bool pass_flag = false; // Avoid node is too close with others
	while(!pass_flag){
		float rand_num = (((double)rand()) / ((double)RAND_MAX));
		// std::cout << "rand_num: " << rand_num << std::endl;
		if (rand_num > 1 - alpha && current_target_node != NULL)
		{
			// Sample between robot and current goal.		
			double x = (((double)rand()) / ((double)RAND_MAX))*(root->state.x - current_target_node->state.x) + root->state.x;
			double y = (((double)rand()) / ((double)RAND_MAX))*(root->state.y - current_target_node->state.y) + root->state.y;

			sample_node.state.x = x;
			sample_node.state.y = y;
		}
		else if (rand_num >= (1 - alpha) / beta && current_target_node != NULL)
		{
			// Using informed rrt method
			float major_axis = path_length;
			float focal_length = util::distance(robot_pose.x, current_target_node->state.x, robot_pose.y, current_target_node->state.y);
			Eigen::Vector2d ellipse_centre;
			ellipse_centre[0] = (robot_pose.x+current_target_node->state.x)/2;
			ellipse_centre[1] = (robot_pose.y+current_target_node->state.y)/2;
		
			float angle = std::atan2(-(current_target_node->state.y-robot_pose.y),current_target_node->state.x-robot_pose.x); //Frame is with y pointing downwards

			float r1 = major_axis / 2;
			float r2 = sqrt(SQ(major_axis) - SQ(focal_length)) / 2;

			float x = (((double)rand()) / ((double)RAND_MAX));
			float y = (((double)rand()) / ((double)RAND_MAX));

			float x2 =  x * r1 * std::cos(angle) + y * r2 * std::sin(angle);
			float y2 = -x * r1 * std::sin(angle) + y * r2 * std::cos(angle);

			Eigen::Vector2d rot_sample, rot_trans_sample;
			rot_sample << x2, y2;
			rot_trans_sample = rot_sample + ellipse_centre; //! rotation first, then locomotion

			sample_node.state.x = rot_trans_sample[0];
			sample_node.state.y = rot_trans_sample[1];
		}
		else if (rand_num <= ((1 - alpha) / beta * Gamma)){
			// Using sensor range sample method in grid map
			float local_x = map_x*sqrt(2)/2 *(((double)rand()) / ((double)RAND_MAX));
			float local_y = local_x * TAN60 * (((double)rand()) / ((double)RAND_MAX) - 0.5)*2;
			// Transform Cartesian coordinates to polar coordinates
			float r = sqrt(SQ(local_x)+SQ(local_y));
			float theta = atan2(local_y, local_x);
			theta += robot_pose.yaw;
			// Transform Polar coordinates to Cartesian coordinates
			local_x = r * cos(theta);
			local_y = r * sin(theta);
			// Transform frame : global -> base
			sample_node.state.x = local_x + robot_pose.x;
			sample_node.state.y = local_y + robot_pose.y;
			visualizeSample(sample_node);
		}
		else{
			// Using random sample method in the whole map
			float x = map_x*(((double)rand()) / ((double)RAND_MAX) - 0.5);
			float y = map_y*(((double)rand()) / ((double)RAND_MAX) - 0.5);
			sample_node.state.x = x + robot_pose.x;
			sample_node.state.y = y + robot_pose.y;
			// visualizeSample(sample_node);
		}
		// Update flag
		Nodes neibor_nodes = getNeiborNodes(sample_node.state.x, sample_node.state.y, 0.0, minNodeDistance);
		if(neibor_nodes.empty()){
			// No any too close node
			pass_flag = true;
		}
	}
	return sample_node;

}

void rtrrt::visualizeSample(rtrrt_ns::Node sample_node){
	visualization_msgs::Marker sample_node_marker;
	sample_node_marker.header.frame_id = global_frame;
	sample_node_marker.header.stamp = ros::Time::now();
	sample_node_marker.type = sample_node_marker.SPHERE;
	sample_node_marker.action = sample_node_marker.ADD;
	sample_node_marker.lifetime = ros::Duration();
	sample_node_marker.color.a = 1.0;
	sample_node_marker.color.r = 255.0 / 255.0;
	sample_node_marker.color.g = 199.0 / 255.0;
	sample_node_marker.color.b = 65.0 / 255.0;
	sample_node_marker.scale.x = 0.5;
	sample_node_marker.scale.y = 0.5;
	sample_node_marker.scale.z = 0.5;
	sample_node_marker.pose.orientation.x = 0;
	sample_node_marker.pose.orientation.y = 0;
	sample_node_marker.pose.orientation.z = 0;
	sample_node_marker.pose.orientation.w = 1;
	sample_node_marker.points.clear();
	sample_node_marker.pose.position.x = sample_node.state.x;
	sample_node_marker.pose.position.y = sample_node.state.y;
	sample_node_marker.pose.position.z = 1.0;
	pub_sample_node.publish(sample_node_marker);
}

rtrrt_ns::Node rtrrt::steer(rtrrt_ns::Node x_s){
	rtrrt_ns::Node x_new;
	// Find the nearest node using kd-tree
	rtrrt_ns::Node::Ptr nearestNode = getNearestNode(x_s.state.x, x_s.state.y);
	if(nearestNode == NULL){
		return x_new;
	}
	float dis = distance(x_s.state.x, nearestNode->state.x, x_s.state.y, nearestNode->state.y);
	if(dis >= epsilon){
		x_new.state.x = nearestNode->state.x+(x_s.state.x-nearestNode->state.x) * epsilon/dis;
		x_new.state.y = nearestNode->state.y+(x_s.state.y-nearestNode->state.y) * epsilon/dis;
		x_new.state.z = 0.0;
		return x_new;
	}
	else{
		return x_s;
	}
}

void rtrrt::updateRobotPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &new_pose)
{
	tf::StampedTransform transform;
	tf::poseMsgToTF(new_pose->pose.pose, transform);
	double roll, pitch, yaw;
	tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
	robot_pose.x = transform.getOrigin().x();
	robot_pose.y = transform.getOrigin().y();
	robot_pose.z = transform.getOrigin().z()-0.11;
	robot_pose.roll = roll;
	robot_pose.pitch = pitch;
	robot_pose.yaw = yaw;
	updatedPose = true;

	// tf::StampedTransform transform;
	// tf::TransformListener listener;
	// try
	// {
	// 	listener.waitForTransform("map","base_link",ros::Time::now(),ros::Duration(0.5));
	// 	listener.lookupTransform("map","base_link",ros::Time(0),transform);	
	// 	double roll, pitch, yaw;
	// 	tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
	// 	robot_pose.x = transform.getOrigin().x();
	// 	robot_pose.y = transform.getOrigin().y();
	// 	robot_pose.z = transform.getOrigin().z()-0.11;
	// 	robot_pose.roll = roll;
	// 	robot_pose.pitch = pitch;
	// 	robot_pose.yaw = yaw;
	// 	updatedPose = true;
	// }
	// catch(const std::exception& e)
	// {
	// 	std::cerr << e.what() << '\n';
	// }
}

bool rtrrt::checkNewNode(rtrrt_ns::Node x_new){
	Position node_state{x_new.state.x, x_new.state.y};
	if(!full_grid_map.isInside(node_state)){
		return false;
	}
	float elevation_value = getElevation(x_new.state.x, x_new.state.y, full_grid_map);
	if(isnan(elevation_value)){
		return false;
	}
	return true;
}

bool rtrrt::checkEdge(rtrrt_ns::Edge edge){
	if(fabs(edge.gradient) > gradientDiffThre){
		return false;
	} 
	else return true;
}

bool rtrrt::addNodeEdge(rtrrt_ns::Node x_new, rtrrt_ns::Node::Ptr x_parent){
	rtrrt_ns::Node::Ptr new_node = make_shared<rtrrt_ns::Node>();
	bool outcome = false;	// Whether or not add node-edge successfully
	if(checkNewNode(x_new)){
		float dis_new_parent = distance(x_new.state.x, x_parent->state.x, x_new.state.y, x_parent->state.y);

		// Check neibor nodes
		Nodes neibor_nodes = getNeiborNodes(x_new.state.x, x_new.state.y, 0.0, minNodeDistance);
		if(!neibor_nodes.empty()){
			//? delete new_node;
			new_node = NULL;
			return false;
		}

		x_new.state.z = getElevation(x_new.state.x, x_new.state.y, full_grid_map)+0.1;
		rtrrt_ns::Edge::Ptr parent_to_new = make_shared<rtrrt_ns::Edge>();
		parent_to_new->elevation_diff = x_new.state.z - x_parent->state.z;
		parent_to_new->gradient = util::gradient(dis_new_parent, parent_to_new->elevation_diff);
		
		if(checkEdge(*parent_to_new)){
			// Add node and edge to Tree
			new_node->state = x_new.state;
			new_node->children.clear();
			new_node->disToRoot = dis_new_parent+x_parent->disToRoot;
			new_node->gradientToRoot = parent_to_new->gradient + x_parent->gradientToRoot;
			new_node->parent = x_parent;
			new_node->prevParent = NULL;
			new_node->nodeState = NodeState::LEAF;

			// Add to PCL pointcloud
			new_node->pcl_pt.x = new_node->state.x;
			new_node->pcl_pt.y = new_node->state.y;
			new_node->pcl_pt.z = 0.0;
			new_node->pcl_pt.intensity = 0.0;
			treeNodes->points.push_back(new_node->pcl_pt);

			// Add to node set
			node_set.push_back(new_node);
			parent_to_new->fromNode = x_parent;
			parent_to_new->toNode = new_node;
			parent_to_new->length = dis_new_parent;
			// Add to edge set 
			edge_set.push_back(parent_to_new);
			new_node->endEdge = edge_set.back();
			// Add to tree
			KDTreePt::Point<float> pt(new_node->state.x, new_node->state.y, 0.0);
			kdtree_pts->points.push_back(pt);
			kdtree_->AddPoint(kdtree_pts->points.size()-1, kdtree_pts->points.size()-1);
			id_nodes.insert(make_pair(kdtree_pts->points.size()-1, new_node));
			new_node->node_id = kdtree_pts->points.size()-1;
			NodePair new_parent_pair{x_parent, new_node};
			tree.insert(make_pair(parent_to_new, new_parent_pair));
			leaf_nodes.push_back(new_node);

			x_parent->startEdges.push_back(parent_to_new);
			x_parent->children.push_back(new_node);
			if(x_parent->nodeState == NodeState::LEAF){
				x_parent->nodeState = NodeState::NORMAL;
				leaf_nodes.remove(x_parent);
			}
			// Get node penalty factor

			outcome = true;
		}
		else {
			//? delete parent_to_new;
			//? delete new_node;
			updateOrientationArray(x_new, x_parent);
			if(util::getOrientation(x_parent->obstacle_orientation) > obstacleSectorNumThre){
				auto find_result = find(obstacle_node_set.begin(), obstacle_node_set.end(), x_parent);
				if(find_result == obstacle_node_set.end() && x_parent->nodeState == NodeState::LEAF){	// No found
					// Delete all nodes which are relevant with x_parent
					// Delete x_parent's parent connection
					if(x_parent->nodeState != NodeState::ROOT){
						x_parent->parent->startEdges.remove(x_parent->endEdge);
						x_parent->parent->children.remove(x_parent);
						if(x_parent->parent->children.size() == 0 && 
						   x_parent->parent->nodeState!=NodeState::ROOT &&
						   x_parent->parent->nodeState!=NodeState::LEAF){

							x_parent->parent->nodeState = NodeState::LEAF;
							leaf_nodes.push_back(x_parent->parent);
						}
						x_parent->parent = NULL;
						x_parent->prevParent = NULL;
						// Cost value is close to Inf for avoid obstacle
						x_parent->disToRoot = inf;
						x_parent->gradientToRoot = inf;
						x_parent->penaltyFactor = maxPenalty;
					}
					node_set.remove(x_parent);
					edge_set.remove(x_parent->endEdge);
					tree.erase(x_parent->endEdge);
					x_parent->endEdge = NULL;
					leaf_nodes.remove(x_parent);					
					x_parent->nodeState = NodeState::OBSTACLE;
					// Remove from Kdtree
					kdtree_->RemovePoint(x_parent->node_id);
					id_nodes.erase(x_parent->node_id);

					obstacle_node_set.push_back(x_parent);
					// Add to obstacle tree
					KDTreePt::Point<float> pt(x_parent->state.x, x_parent->state.y, 0.0);
					obstacle_pts->points.push_back(pt);
					obstacleKdtree_->AddPoint(obstacle_pts->points.size()-1, obstacle_pts->points.size()-1);
					obs_id_nodes.insert(make_pair(obstacle_pts->points.size()-1, x_parent));
					x_parent->node_id = obstacle_pts->points.size()-1;

					// Update neibor nodes property
					Nodes near_nodes = getNeiborNodes(x_parent->state.x, x_parent->state.y, 0.0, inflationRadius);
					if(!near_nodes.empty()){
						auto iter = near_nodes.begin();
						while(iter!=near_nodes.end()){
							rtrrt_ns::Node::Ptr node = *iter;
							// Except OBSTACLE nodes
							if(node->nodeState == NodeState::OBSTACLE || node->nodeState == NodeState::ROOT){
								iter++;
								continue;
							}
							float disToObs = util::distance(node->state.x, x_parent->state.x, node->state.y, x_parent->state.y);
							// Update penalty factor
							float new_penalty = util::getPenalty(disToObs);
							if(new_penalty < node->penaltyFactor){
								iter++;
								continue;
							}
							node->penaltyFactor = util::getPenalty(disToObs);
							queue<rtrrt_ns::Node::Ptr> q_update;
							q_update.push(node);
							while(!q_update.empty()){
								rtrrt_ns::Node::Ptr n = q_update.front();
								q_update.pop();
								n->disToRoot = n->parent->disToRoot + n->endEdge->length*(n->penaltyFactor);
								n->gradientToRoot = n->parent->gradientToRoot + n->endEdge->gradient*(n->penaltyFactor);
								if(n->nodeState != NodeState::LEAF){
									for(auto it=n->children.begin(); it!=n->children.end();it++){
										q_update.push((*it));
									}
								}
							}
							iter++;
						}
					}
				}	
			}
		}
	}
	else new_node = NULL;
	return outcome;
}

void rtrrt::reselectParent(rtrrt_ns::Node::Ptr x_new){

	float dis_min = x_new->disToRoot;
	rtrrt_ns::Node::Ptr best_neibor = NULL;
	rtrrt_ns::Edge::Ptr best_neibo_edge = make_shared<rtrrt_ns::Edge>();
	// Find near nodes using kd-tree within a range
	Nodes neibor_nodes = getNeiborNodes(x_new->state.x, x_new->state.y, 0.0, neiborRangeThre);
	if(neibor_nodes.size()<=1){
		// If none or only one node, no reselection
		best_neibo_edge = NULL;
		return;
	}
	else{
		// Iterate all neibor nodes to find shortest branch
		auto iter = neibor_nodes.begin();
		while(iter != neibor_nodes.end()){
			rtrrt_ns::Node::Ptr neibor = *iter;
			// Except OBSTACLE nodes
			if(neibor->nodeState == NodeState::OBSTACLE){
				iter++;
				continue;
			}
			rtrrt_ns::Edge neibor_new_edge;
			neibor_new_edge.fromNode = neibor;
			neibor_new_edge.toNode = x_new;
			neibor_new_edge.elevation_diff = x_new->state.z - neibor->state.z;
			neibor_new_edge.length = distance(x_new->state.x, neibor->state.x, x_new->state.y, neibor->state.y);
			neibor_new_edge.gradient = util::gradient(neibor_new_edge.length, neibor_new_edge.elevation_diff);			
			if(checkEdge(neibor_new_edge)){
				float dis = neibor->disToRoot + neibor_new_edge.length*x_new->penaltyFactor;
				if(dis < dis_min){
					// Save best node info
					dis_min = dis;
					best_neibor = neibor;
					best_neibo_edge->fromNode = neibor_new_edge.fromNode;
					best_neibo_edge->toNode = neibor_new_edge.toNode;
					best_neibo_edge->elevation_diff = neibor_new_edge.elevation_diff;
					best_neibo_edge->gradient = neibor_new_edge.gradient;
					best_neibo_edge->length = neibor_new_edge.length;
				}
			}
			iter++;
		}
		// kd_res_free(neibor_nodes);
		if(x_new->parent != best_neibor && best_neibor != NULL){
			// Change current parent node, select neibor node as parent
			// For old parent, remove items
			x_new->prevParent = x_new->parent;
			x_new->parent->startEdges.remove(x_new->endEdge);
			x_new->parent->children.remove(x_new);
			if(x_new->parent->nodeState!=NodeState::LEAF && 
			   x_new->parent->nodeState!=NodeState::ROOT && 
			   x_new->parent->children.size() == 0){
				x_new->parent->nodeState = NodeState::LEAF;
				leaf_nodes.push_back(x_new->parent);
			}
			// Delete specific edge
			edge_set.remove(x_new->endEdge);
			tree.erase(x_new->endEdge);
			// For new parent, add new item
			x_new->parent = best_neibor;
			x_new->parent->children.push_back(x_new);
			if(x_new->parent->nodeState != NodeState::ROOT){
				if(x_new->parent->nodeState == LEAF){
					leaf_nodes.remove(x_new->parent);
				}
				x_new->parent->nodeState = NodeState::NORMAL;
			}
			edge_set.push_back(best_neibo_edge);
			x_new->endEdge = best_neibo_edge;
			x_new->disToRoot = x_new->parent->disToRoot + best_neibo_edge->length*x_new->penaltyFactor;
			x_new->gradientToRoot = x_new->parent->gradientToRoot+best_neibo_edge->gradient*x_new->penaltyFactor;
			NodePair new_parent_pair{best_neibor, x_new};
			tree.insert(make_pair(best_neibo_edge, new_parent_pair));
		}
		else best_neibo_edge = NULL;
	}
}

void rtrrt::rewireRandomNode(){
	float timeKeeper = ros::Time::now().toSec();
	while(q_rewire_from_rand.size()!=0 && (ros::Time::now().toSec() - timeKeeper) < allowRewiringFromRand){
		// Take the front of RTRRTstarrewireRand to rewire
		Node::Ptr x_rand = q_rewire_from_rand.front();
		q_rewire_from_rand.pop();
		// Find near nodes using kd-tree within a range
		Nodes neibor_nodes = getNeiborNodes(x_rand->state.x, x_rand->state.y, 0.0, neiborRangeThre);
		if(neibor_nodes.size()<=1)
		{
			continue;
		}
		else{
			// Iterate all neibor nodes
			auto iter = neibor_nodes.begin();
			while(iter != neibor_nodes.end()){
				rtrrt_ns::Node::Ptr neibor = *iter;
				// Except OBSTACLE nodes
				if(neibor->nodeState == NodeState::OBSTACLE){
					iter++;
					continue;
				}
				rtrrt_ns::Edge::Ptr rand_neibor_edge = make_shared<rtrrt_ns::Edge>();
				rand_neibor_edge->fromNode = x_rand;
				rand_neibor_edge->toNode = neibor;
				rand_neibor_edge->elevation_diff = neibor->state.z - x_rand->state.z;
				rand_neibor_edge->length = distance(neibor->state.x, x_rand->state.x, neibor->state.y, x_rand->state.y);
				rand_neibor_edge->gradient = util::gradient(rand_neibor_edge->length, rand_neibor_edge->elevation_diff);
				if(checkEdge(*rand_neibor_edge)){
					float dis = x_rand->disToRoot + rand_neibor_edge->length*neibor->penaltyFactor;
					if(dis < neibor->disToRoot && neibor->parent != x_rand){
						// X_rand become neibor parent
						// For old parent, remove item
						neibor->parent->children.remove(neibor);
						if(neibor->parent->children.size() ==0 && neibor->parent->nodeState!= NodeState::LEAF){
							neibor->parent->nodeState= NodeState::LEAF;
							leaf_nodes.push_back(neibor->parent);
						}
						neibor->parent->startEdges.remove(neibor->endEdge);
						// Delete specific edge
						edge_set.remove(neibor->endEdge);
						tree.erase(neibor->endEdge);
						// For new parent(x_rand), update item
						edge_set.push_back(rand_neibor_edge);
						neibor->prevParent = neibor->parent;
						neibor->parent = x_rand;
						neibor->disToRoot = dis;
						neibor->gradientToRoot = x_rand->gradientToRoot+rand_neibor_edge->gradient*neibor->penaltyFactor;
						neibor->endEdge = rand_neibor_edge;
						x_rand->children.push_back(neibor);
						x_rand->startEdges.push_back(rand_neibor_edge);
						if(x_rand->nodeState == NodeState::LEAF){
							x_rand->nodeState = NodeState::NORMAL;
							leaf_nodes.remove(x_rand);
						}
						NodePair rand_neibor_pair{x_rand, neibor};
						tree.insert(make_pair(rand_neibor_edge, rand_neibor_pair));
					}
					else rand_neibor_edge = NULL;
				}
				else rand_neibor_edge = NULL;
				iter++;
			}
		}
	}
}

void rtrrt::rewireFromRoot(){
	unordered_set<rtrrt_ns::Node::Ptr> close_node_set;
	// Empty queue will start at root node
	if (q_rewire_from_root.empty()){
		q_rewire_from_root.push(root);
	}
	float timeKeeper = ros::Time::now().toSec();
	while(!q_rewire_from_root.empty() && (ros::Time::now().toSec() - timeKeeper) < allowRewiringFromRoot){
		// Take the front of RTRRTstarrewireRand to rewire
		Node::Ptr x_r = q_rewire_from_root.front();
		q_rewire_from_root.pop();
		// Find near nodes using kd-tree within a range
		Nodes neibor_nodes = getNeiborNodes(x_r->state.x, x_r->state.y, 0.0, neiborRangeThre);
		if(neibor_nodes.size()<=1)
		{
			continue;
		}
		else{
			// Iterate all neibor nodes
			auto iter = neibor_nodes.begin();
			while(iter != neibor_nodes.end()){
				rtrrt_ns::Node::Ptr neibor = *iter;
				// Avoid residual edge-node
				if(neibor->parent == NULL && neibor->nodeState!=NodeState::ROOT){
					queue<rtrrt_ns::Node::Ptr> q_remove;
					q_remove.push(neibor);
					while(!q_remove.empty()){
						// Get top element
						rtrrt_ns::Node::Ptr node = q_remove.front();
						q_remove.pop();
						// Delete all parent items
						if(node->parent != NULL){
							node->parent->children.remove(node);
							node->parent->startEdges.remove(node->endEdge);
						}
						// Add children to queue
						for(auto it_n=node->children.begin(); it_n!=node->children.end(); it_n++){
							q_remove.push(*it_n);
						}
						
						// Remove from node set and edge set
						node_set.remove(node);
						edge_set.remove(node->endEdge);
						tree.erase(node->endEdge);

						// Remove from leaf set
						if(node->nodeState == NodeState::LEAF){
							leaf_nodes.remove(node);
						}
						// Remove from kdtree
						kdtree_->RemovePoint(node->node_id);
						id_nodes.erase(node->node_id);
					}

					iter++;			
					continue;
				}
				// Except OBSTACLE nodes
				if(neibor->nodeState == NodeState::OBSTACLE){
					iter++;			
					continue;
				}
				rtrrt_ns::Edge::Ptr xr_neibor_edge = make_shared<rtrrt_ns::Edge>();
				xr_neibor_edge->fromNode = x_r;
				xr_neibor_edge->toNode = neibor;
				xr_neibor_edge->elevation_diff = neibor->state.z - x_r->state.z;
				xr_neibor_edge->length = distance(neibor->state.x, x_r->state.x, neibor->state.y, x_r->state.y);
				xr_neibor_edge->gradient = util::gradient(xr_neibor_edge->length, xr_neibor_edge->elevation_diff);
				if(checkEdge(*xr_neibor_edge)){
					float dis = x_r->disToRoot + xr_neibor_edge->length*neibor->penaltyFactor;
					if(dis < neibor->disToRoot && neibor->parent != x_r){
						// X_r become neibor new parent
						// For old parent, remove item
						neibor->parent->children.remove(neibor);
						if(neibor->parent->children.size() == 0 && neibor->parent->nodeState!= NodeState::LEAF){
							neibor->parent->nodeState= NodeState::LEAF;
							leaf_nodes.push_back(neibor->parent);
						}
						neibor->parent->startEdges.remove(neibor->endEdge);
						// Delete specific edge
						edge_set.remove(neibor->endEdge);
						tree.erase(neibor->endEdge);
						// For new parent(x_r), update item
						edge_set.push_back(xr_neibor_edge);
						neibor->prevParent = neibor->parent;
						neibor->parent = x_r;
						neibor->disToRoot = dis;
						neibor->gradientToRoot = x_r->gradientToRoot+xr_neibor_edge->gradient*neibor->penaltyFactor;
						neibor->endEdge = xr_neibor_edge;
						x_r->children.push_back(neibor);
						x_r->startEdges.push_back(xr_neibor_edge);
						if(x_r->nodeState == NodeState::LEAF){
							x_r->nodeState = NodeState::NORMAL;
							leaf_nodes.remove(x_r);
						}
						NodePair rand_neibor_pair{x_r, neibor};
						tree.insert(make_pair(xr_neibor_edge, rand_neibor_pair));
					}
					else xr_neibor_edge = NULL;
				}
				else xr_neibor_edge = NULL;
				if(close_node_set.find(neibor) == close_node_set.end()){ // Check in close-set, if found, no longer add to queue 
					q_rewire_from_root.push(neibor);
					close_node_set.insert(neibor);
				}
				iter++;		
			}
		}
	}
}

void rtrrt::changeRoot(rtrrt_ns::Node::Ptr new_root){
	if(new_root == root){
		// Avoid change root to repeat node
		return;
	}
	ROS_INFO("New root: %f, %f, %f", new_root->state.x, new_root->state.y, new_root->state.z);
	if(new_root->parent == root){
		root->startEdges.remove(new_root->endEdge);
		edge_set.remove(new_root->endEdge);
		tree.erase(new_root->endEdge);
	}
	rtrrt_ns::Edge::Ptr new_root_edge = make_shared<rtrrt_ns::Edge>();
	new_root_edge->fromNode = new_root;
	new_root_edge->toNode = root;
	new_root_edge->elevation_diff = root->state.z - new_root->state.z;
	new_root_edge->length = distance(root->state.x, new_root->state.x, root->state.y, new_root->state.y);
	new_root_edge->gradient = util::gradient(new_root_edge->length, new_root_edge->elevation_diff);
	//! New root must connect with old root, therfore no need to check edge, ortherwise algorithm will fail
	//	Update old root item
	root->children.remove(new_root);
	//	Update old root children's disToRoot and gradientToRoot
	queue<rtrrt_ns::Node::Ptr> q_old_root_children;
	q_old_root_children.push(root);
	while(!q_old_root_children.empty()){
		rtrrt_ns::Node::Ptr old_child = q_old_root_children.front();
		q_old_root_children.pop();
		if(old_child->children.empty()) continue;
		for(auto it = old_child->children.begin(); it!=old_child->children.end(); it++){
			q_old_root_children.push(*it);
			(*it)->disToRoot += new_root_edge->length*root->penaltyFactor;
			(*it)->gradientToRoot += new_root_edge->gradient*root->penaltyFactor;
		}
	}
	root->disToRoot = new_root_edge->length*root->penaltyFactor;
	root->endEdge = new_root_edge;
	root->gradientToRoot = new_root_edge->gradient*root->penaltyFactor;
	root->parent = new_root;
	root->prevParent = NULL;

	if(root->children.size()!=0) root->nodeState = NodeState::NORMAL;
	else{
		root->nodeState = NodeState::LEAF;
		leaf_nodes.push_back(root);
	}
	//	Update new root item
	//	Update new root children's disToRoot and gradientToRoot
	queue<rtrrt_ns::Node::Ptr> q_new_root_children;
	q_new_root_children.push(new_root);
	while(!q_new_root_children.empty()){
		rtrrt_ns::Node::Ptr child = q_new_root_children.front();
		q_new_root_children.pop();
		if(child->children.empty()) continue;
		for(auto it = child->children.begin(); it!=child->children.end(); it++){
			q_new_root_children.push(*it);
			(*it)->disToRoot -= new_root_edge->length*new_root->penaltyFactor;
			(*it)->gradientToRoot -= new_root_edge->gradient*new_root->penaltyFactor;
		}
	}
	new_root->children.push_back(root);
	new_root->disToRoot = 0;
	new_root->gradientToRoot = 0;
	new_root->endEdge = NULL;
	if(new_root->nodeState == NodeState::LEAF) leaf_nodes.remove(new_root);
	new_root->nodeState = NodeState::ROOT;
	new_root->parent = NULL;
	new_root->prevParent = NULL;
	new_root->startEdges.push_back(new_root_edge);
	edge_set.push_back(new_root_edge);
	NodePair new_root_pair{new_root, root};
	tree.insert(make_pair(new_root_edge, new_root_pair));
	root = new_root;
	
	rewireFromRoot();
}

void rtrrt::pruneTree(rtrrt_ns::Node::Ptr new_center){
	pcl::KdTreeFLANN<PointFrontier> remove_kdtree;
	// remove_kdtree.setInputCloud(treeNodes);

	// Clear cache
	last_removed_nodes.clear();
	last_removed_edges.clear();
	treeNodes->points.clear();
	// Get center point
	// std::cout << "11111111111111111" << std::endl;
	float center_x = new_center->state.x;
	float center_y = new_center->state.y;

	// Iterate all node, judge them is whether inside local map
	queue<rtrrt_ns::Node::Ptr> q_query;
	q_query.push(root);
	mtx.lock();
	while(!q_query.empty()){
		// Get top element
		rtrrt_ns::Node::Ptr queried_node = q_query.front();
		q_query.pop();
		// If has children, add to queue
		if(!queried_node->children.empty()){
			for(auto it=queried_node->children.begin(); it!=queried_node->children.end(); it++){
				q_query.push(*it);
			}
		}
		// Judge inside 
		float delta_x_abs = fabs(queried_node->state.x - center_x);
		float delta_y_abs = fabs(queried_node->state.y - center_y);
		// Position queried_pose(queried_node->state.x, queried_node->state.x);
		Position queried_pose{queried_node->state.x, queried_node->state.y};		
		// if(delta_x_abs < map_x/2 && delta_y_abs < map_y/2){
		if(full_grid_map.isInside(queried_pose)){
			// Inside local map, save it
			treeNodes->points.push_back(queried_node->pcl_pt);
			continue;
		}
		// std::cout << "222222222223333333333" << std::endl;

		// Outside, remove relevant node and edges
		rtrrt_ns::Node::Ptr removed_node = queried_node;

		// Delete kdtree
		kdtree_->RemovePoint(removed_node->node_id);
		id_nodes.erase(removed_node->node_id);
		// Delete x_parent item
		removed_node->parent->startEdges.remove(removed_node->endEdge);
		// std::cout << "33333333333333333333333" << std::endl;
		removed_node->parent->children.remove(removed_node);
		if(removed_node->parent->children.empty()&&
		   removed_node->parent->nodeState != NodeState::ROOT){
			removed_node->parent->nodeState = NodeState::LEAF;
			leaf_nodes.push_back(removed_node->parent);
		}

		// Delete node and edges from the node set
		node_set.remove(removed_node);
		edge_set.remove(removed_node->endEdge);
		tree.erase(removed_node->endEdge);

		// Add removed set for visualization
		last_removed_nodes.push_back(removed_node);
		last_removed_edges.push_back(removed_node->endEdge);
		// Remove from leaf set
		if(removed_node->nodeState == NodeState::LEAF){
			leaf_nodes.remove(removed_node);
		}
		else{
			queue<rtrrt_ns::Node::Ptr> q_remove;
			for(auto it_r=removed_node->children.begin(); it_r!=removed_node->children.end(); it_r++){
				q_remove.push(*it_r);
			}
			while(!q_remove.empty()){
				// Get top element
				rtrrt_ns::Node::Ptr node = q_remove.front();
				q_remove.pop();
				// Delete all parent items
				node->parent->children.remove(node);
				node->parent->startEdges.remove(node->endEdge);
				// Add children to queue
				for(auto it_n=node->children.begin(); it_n!=node->children.end(); it_n++){
					q_remove.push(*it_n);
				}
				
				// Remove from node set and edge set
				node_set.remove(node);
				edge_set.remove(node->endEdge);
				tree.erase(node->endEdge);

				// Remove from leaf set
				if(node->nodeState == NodeState::LEAF){
					leaf_nodes.remove(node);
				}
				// Remove from kdtree
				kdtree_->RemovePoint(node->node_id);
				id_nodes.erase(node->node_id);
				last_removed_nodes.push_back(node);
				last_removed_edges.push_back(node->endEdge);
			}
		}
	}
	mtx.unlock();
	publishCloud(treeNodesPub, treeNodes, ros::Time::now(), global_frame);

}

rtrrt_ns::Node::Ptr rtrrt::findTarget(){
	if(newGoalReceived){
		float totalDistance = 0;
		float totalGradient = 0;
		float best_cost = inf;
		rtrrt_ns::Node::Ptr best_node = NULL;
		// Find near node to decide use final_goal or subgoal
		Nodes candidate_targets = getNeiborNodes(goal.position.x, goal.position.y, 0.0, targetTolerance);
		if(candidate_targets.empty())
		{
			return NULL;
		}
		else{	// Candidate nodes found, use goal.and select best one(smallest costVal) as target
			// Firstly, calculate total distance and total gradient
			auto iter = candidate_targets.begin();
			while(iter != candidate_targets.end()){
				rtrrt_ns::Node::Ptr node = *iter;
				// Except OBSTACLE nodes
				if(node->nodeState == NodeState::OBSTACLE){
					iter++;
					continue;
				}
				totalDistance += node->disToRoot;
				totalGradient += node->gradientToRoot;
				// kd_res_next(candidate_targets);
				iter++;
			}
			iter = candidate_targets.begin();
			while(iter != candidate_targets.end()){
				rtrrt_ns::Node::Ptr node = *iter;
				// Except OBSTACLE nodes
				if(node->nodeState == NodeState::OBSTACLE){
					iter++;
					continue;
				}
				float costVal = costFunc(node->disToRoot, 0.0, node->gradientToRoot, totalDistance, totalGradient);
				if(costVal < best_cost){
					best_cost = costVal;
					best_node = node;
				}
				// kd_res_next(candidate_targets);
				iter++;
			}
			// kd_res_free(candidate_targets);
			current_target = &(goal.position);
			current_target_node = best_node;
			return best_node;
		}
	}
}


void rtrrt::executeTreeUpdate(){
	// If tree is empty, then add root node
	// std::cout << "11111111111111111" << std::endl;
	if(tree.empty() && node_set.size() == 0 && updatedPose){
		rtrrt_ns::Node::Ptr root_node = make_shared<rtrrt_ns::Node>();
		root_node->state.x = robot_pose.x;
		root_node->state.y = robot_pose.y;
		root_node->state.z = robot_pose.z;
		root_node->parent = NULL;
		root_node->prevParent = NULL;
		root_node->disToRoot = 0.0;
		root_node->gradientToRoot = 0.0;
		root_node->nodeState = NodeState::ROOT;
		root_node->endEdge = NULL;
		node_set.push_back(root_node);
		root = node_set.back();
		// Add to PCL points
		root_node->pcl_pt.x = robot_pose.x;
		root_node->pcl_pt.y = robot_pose.y;
		root_node->pcl_pt.z = 0.0;
		root_node->pcl_pt.intensity = 0.0;
		treeNodes->points.push_back(root_node->pcl_pt);
		// kd_insert3(kdTree_, root_node->state.x, root_node->state.y, 0.0, root_node);
		KDTreePt::Point<float> pt(root_node->state.x, root_node->state.y, 0.0);
		kdtree_pts->points.push_back(pt);
		kdtree_->AddPoint(kdtree_pts->points.size()-1, kdtree_pts->points.size()-1);
		id_nodes.insert(make_pair(kdtree_pts->points.size()-1, root_node));
		root_node->node_id = kdtree_pts->points.size()-1;
	}
	// Assume root is existed
	//	Sampling
	// std::cout << "222222222222222222" << std::endl;
	sampleNode = sample();

	// Find new node
	// std::cout << "33333333333333333" << std::endl;
	newNode = steer(sampleNode);

	// Find nearest node in the tree
	// std::cout << "44444444444444444" << std::endl;

	rtrrt_ns::Node::Ptr nearestNode = getNearestNode(newNode.state.x, newNode.state.y);
	if(nearestNode == NULL){
		return;
	}
	// std::cout << "55555555555555555" << std::endl;

	if(nearestNode->nodeState!=NodeState::OBSTACLE && addNodeEdge(newNode, nearestNode)){
		// std::cout << "Add node edge successfully!" << std::endl;
		reselectParent(node_set.back());	// New added node need to reselect parent
		q_rewire_from_rand.push(node_set.back());  // New added node need to reselect parent
	}
	// std::cout << "666666666666666666" << std::endl;

	rewireRandomNode();
	// std::cout << "777777777777777777" << std::endl;

	skipCountRewiring--;
	if (skipCountRewiring < 0) {
		// std::cout << "88888888888888888888" << std::endl;
		rewireFromRoot();
		// std::cout << "99999999999999999999" << std::endl;
		skipCountRewiring = 3;
	}

	// visualization
	auto cur_time = ros::Time::now();
	if((cur_time-last_time).toSec() > 1.0){
		visualizationCb();
	}
}

void rtrrt::updateGoal(geometry_msgs::PoseStamped goal_point){

	if(!updatedPose)
	{
		ROS_WARN("Cannot get the robot pose.");
		return;
	}
	std::cout << "goal_point: " << goal_point.pose.position.x << ", " << goal_point.pose.position.y << std::endl;
	goal.position.x = goal_point.pose.position.x;
	goal.position.y = goal_point.pose.position.y;
	goal.position.z = goal_point.pose.position.z;
	goal.orientation = goal_point.pose.orientation;

	newGoalReceived = true;
}

void rtrrt::getFrontierNode(){
	frontierNodes.clear();
    auto pt = leaf_nodes.begin();
    while(pt != leaf_nodes.end()){
		// Only leaf nodes inside current grid map can be considered
		Position node_posi{(*pt)->state.x, (*pt)->state.y};
		if(full_grid_map.isInside(node_posi)){
			// The nodes close to robot are abandoned
			float dis = util::distance((*pt)->state.x, robot_pose.x, (*pt)->state.y, robot_pose.y);
			if(dis > frontierDisThre){
				Nodes neibor_nodes = getNeiborNodes((*pt)->state.x, (*pt)->state.y, 0.0, exploredAreaRadius);
				int neibor_nodes_size = neibor_nodes.size();
				// For avoiding explored area
				if(neibor_nodes_size < exploredAreaThre){
					// Avoid collision with obstacle nodes
					float query_pt[3]={(*pt)->state.x, (*pt)->state.y, 0.0};
					vector<size_t> indices;
					vector<float> dists_sq;
					obstacleKdtree_->RadiusSearch(query_pt, inflationRadius*1.5, indices, dists_sq);

					if(indices.empty()){
						Position leaf_posi((*pt)->state.x, (*pt)->state.y);
						Index leaf_index;
						Length leng(frontierRadius, frontierRadius);
						bool isSuccessful;
						auto sub_map = full_grid_map.getSubmap(leaf_posi, leng, leaf_index ,isSuccessful);
						Size submap_size = sub_map.getSize();
						if(submap_size[0] != submap_size[1]){
							// Check exist global vertices
							if(!graphVertices->empty()){
								pcl_kdtree->setInputCloud(graphVertices);
								pcl_kdtree->setSortedResults(false);
								std::vector<int> indices;
								std::vector<float> dist_list;
								pcl_kdtree->radiusSearch((*pt)->pcl_pt, neiborRangeThre*2, indices, dist_list);
								if(!indices.empty()){
									// Explored area
									pt++;
									continue;
								}
							}
							// Add to frontier set
							float information_gain = util::getInfoGain((*pt)->state.x, (*pt)->state.y, full_grid_map, "elevation");
							frontierNodes.insert(make_pair(*pt, information_gain));
							(*pt)->pcl_pt.intensity = information_gain;
						}
						else{
							// Check exist global vertices
							if(!graphVertices->empty()){
								pcl_kdtree->setInputCloud(graphVertices);
								pcl_kdtree->setSortedResults(false);
								std::vector<int> indices;
								std::vector<float> dist_list;
								pcl_kdtree->radiusSearch((*pt)->pcl_pt, neiborRangeThre, indices, dist_list);
								if(!indices.empty()){
									// Explored area
									pt++;
									continue;
								}
							}
							// Add to frontier set
							float information_gain = util::getInfoGain((*pt)->state.x, (*pt)->state.y, full_grid_map, "elevation");
							if(information_gain > informationThre){
								frontierNodes.insert(make_pair(*pt, information_gain));
								(*pt)->pcl_pt.intensity = information_gain;
							}
						}
					}
				}
			}
		}
        pt++;
    }
}

Nodes rtrrt::getNeiborNodes(float point_x, float point_y, float point_z, float range){
	Nodes result_set;
	float query_pt[3]={point_x, point_y, point_z};
	vector<size_t> indices;
	vector<float> dists_sq;
	kdtree_->RadiusSearch(query_pt, range, indices, dists_sq);
	if(indices.empty()){
		return result_set;
	}
	for(int i=0; i<indices.size(); i++){
		if(id_nodes.at(indices[i])->nodeState != NodeState::OBSTACLE){
			result_set.push_back(id_nodes.at(indices[i]));
		}
	}
	return result_set;
}

rtrrt_ns::Node::Ptr rtrrt::getNearestNode(float point_x, float point_y){
	float query_pt[3]={point_x, point_y, 0.0};
	vector<size_t> indices;
	vector<float> dists_sq;
	kdtree_->NearestSearch(query_pt, 1, indices, dists_sq);
	if(indices.empty()){
		return NULL;
	}

	rtrrt_ns::Node::Ptr nearestNode = id_nodes.at(indices[0]);
	return nearestNode;
}

rtrrt_ns::Node::Ptr rtrrt::getObstacleNode(float point_x, float point_y){
	float query_pt[3]={point_x, point_y, 0.0};
	vector<size_t> indices;
	vector<float> dists_sq;
	obstacleKdtree_->NearestSearch(query_pt, 1, indices, dists_sq);
	if(indices.empty()){
		return NULL;
	}

	rtrrt_ns::Node::Ptr nearestNode = obs_id_nodes.at(indices[0]);
	return nearestNode;
}


void rtrrt::updateOrientationArray(rtrrt_ns::Node new_node, rtrrt_ns::Node::Ptr rrt_node){
	// Get ralative x-y angle between two input point
	float deltX = new_node.state.x - rrt_node->state.x;
	float deltY = new_node.state.y - rrt_node->state.y;
	float deltAngle = atan2(deltY, deltX) * 180/M_PI;	// Degree form
	// 8 sector for rrt_node, begin with positive area
	if (deltAngle>0 && deltAngle<=45){
		rrt_node->obstacle_orientation[0] = 1;
	}
	if (deltAngle>45 && deltAngle<=90){
		rrt_node->obstacle_orientation[1] = 1;
	}
	if (deltAngle>90 && deltAngle<=135){
		rrt_node->obstacle_orientation[2] = 1;
	}
	if (deltAngle>135 && deltAngle<=180){
		rrt_node->obstacle_orientation[3] = 1;
	}
	if (deltAngle>-180 && deltAngle<=-135){
		rrt_node->obstacle_orientation[4] = 1;
	}
	if (deltAngle>-135 && deltAngle<=-90){
		rrt_node->obstacle_orientation[5] = 1;
	}
	if (deltAngle>-90 && deltAngle<=-45){
		rrt_node->obstacle_orientation[6] = 1;
	}
	if (deltAngle>-45 && deltAngle<=0){
		rrt_node->obstacle_orientation[7] = 1;
	}
}

void rtrrt::publishRemovedItem(){
	// Publish last removed nodes and edges
	if(pub_removed_edge.getNumSubscribers()!=0 || pub_removed_node.getNumSubscribers()!=0){
		
		visualization_msgs::Marker removed_edge_marker;	// last removed edge
		visualization_msgs::Marker removed_node_marker;	// last removed node
		removed_edge_marker.header.frame_id = global_frame;
		removed_edge_marker.header.stamp = ros::Time::now();
		removed_edge_marker.type = removed_edge_marker.LINE_LIST;
		removed_edge_marker.action = removed_edge_marker.ADD;
		removed_edge_marker.lifetime = ros::Duration();
		removed_edge_marker.color.a = 1.0;
		removed_edge_marker.color.r = 255.0/255.0;
		removed_edge_marker.color.g = 51.0/255.0;
		removed_edge_marker.color.b = 202.0/255.0;
		removed_edge_marker.scale.x = 0.02;
		removed_edge_marker.scale.y = 0.02;
		removed_edge_marker.scale.z = 0.02;
		removed_edge_marker.pose.orientation.w = 1;
		removed_edge_marker.points.clear();

		removed_node_marker.header.frame_id = global_frame;
		removed_node_marker.header.stamp = ros::Time::now();
		removed_node_marker.type = removed_node_marker.CUBE_LIST;
		removed_node_marker.action = removed_node_marker.ADD;
		removed_node_marker.lifetime = ros::Duration();
		removed_node_marker.color.a = 1.0;
		removed_node_marker.color.r = 169.0/255.0;
		removed_node_marker.color.g = 51.0/255.0;
		removed_node_marker.color.b = 255.0/255.0;
		removed_node_marker.scale.x = 0.06;
		removed_node_marker.scale.y = 0.06;
		removed_node_marker.scale.z = 0.06;
		removed_node_marker.pose.orientation.w = 1;
		removed_node_marker.points.clear();

		//	Add data
		geometry_msgs::Point point_3d;
		for(auto iter2=last_removed_edges.begin(); iter2!=last_removed_edges.end(); iter2++){
			point_3d.x = (*iter2)->fromNode->state.x;
			point_3d.y = (*iter2)->fromNode->state.y;
			point_3d.z = (*iter2)->fromNode->state.z;
			removed_edge_marker.points.push_back(point_3d);
			point_3d.x = (*iter2)->toNode->state.x;
			point_3d.y = (*iter2)->toNode->state.y;
			point_3d.z = (*iter2)->toNode->state.z;		
			removed_edge_marker.points.push_back(point_3d);
		}

		geometry_msgs::Point removed_n;
		auto iter = last_removed_nodes.begin();
		while(iter != last_removed_nodes.end()){
			removed_n.x = (*iter)->state.x;
			removed_n.y = (*iter)->state.y;
			removed_n.z = (*iter)->state.z;
			removed_node_marker.points.push_back(removed_n);
			iter++;
		}
		pub_removed_edge.publish(removed_edge_marker);
		pub_removed_node.publish(removed_node_marker);
	}
}

void rtrrt::gridMapHandler(const grid_map_msgs::GridMapConstPtr &raw_map){
	GridMapRosConverter::fromMessage(*raw_map, full_grid_map);
	GridMapRosConverter::fromMessage(*raw_map, sensor_range_map);
	map_x = raw_map->info.length_x;
	map_y = raw_map->info.length_y;
	map_resolution = raw_map->info.resolution;
	receivedGridMap = true;
	tf::TransformListener tempTFListener;
	tf::StampedTransform tempTransform;
	try
	{
		ros::Time tempTime;
		tempTime = ros::Time::now();
		
		tempTFListener.waitForTransform(base_frame,global_frame,ros::Time::now(),ros::Duration(0.5));
		tempTFListener.lookupTransform(base_frame,global_frame,ros::Time(0),tempTransform);	
		geometry_msgs::PointStamped point_in, point_out;
		point_out.header.frame_id = base_frame;
		point_in.header.frame_id = global_frame;
		for(GridMapIterator it(sensor_range_map); !it.isPastEnd(); ++it)
		{
			Position post;
			sensor_range_map.getPosition(*it, post);
			point_in.point.x = post.x();
			point_in.point.y = post.y();
			tempTFListener.transformPoint(base_frame,point_in, point_out);
			float h = fabs(point_out.point.x/(point_out.point.y+0.01));
			if(!(point_out.point.x>0 && h > 1.0/TAN60)){
				try{
					sensor_range_map.at("elevation", *it) = NAN;
				}				
				catch(const std::out_of_range& exception){
					std::cout << "sensor_range_map no found" << std::endl;
				}

			}
		}
        grid_map_msgs::GridMap rangeGridMapMsg;
        GridMapRosConverter::toMessage(sensor_range_map, rangeGridMapMsg);
        pub_range_map.publish(rangeGridMapMsg);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
}

void rtrrt::graphVerticesHandler(const sensor_msgs::PointCloud2ConstPtr &vertices_msgs){
	graphVertices->clear();
	pcl::fromROSMsg(*vertices_msgs, *graphVertices);
}

void rtrrt::visualizationCb(){
	if(!tree.empty() && !node_set.empty() && !edge_set.empty()){
		// if all topics of visualizing tree item have no subscriber, then don't publish
		if(pub_rtrrt_edge.getNumSubscribers()!=0 || pub_rtrrt_node.getNumSubscribers()!=0
		|| pub_leaf_node.getNumSubscribers()!=0 || pub_root_marker.getNumSubscribers()!=0){

			visualization_msgs::Marker edge_marker;	// edge represent by linelist
			visualization_msgs::Marker node_marker;	// node represent by cube
			visualization_msgs::Marker root_marker;	// root_node represent by cube
			visualization_msgs::Marker leaf_marker;	// leaf represent by cube
			visualization_msgs::Marker obstacle_node_marker; // Transparent sphere list
			
			// Marker init
			// Edge
			edge_marker.header.frame_id = global_frame;
			edge_marker.header.stamp = ros::Time::now();
			edge_marker.type = edge_marker.LINE_LIST;
			edge_marker.action = edge_marker.ADD;
			edge_marker.lifetime = ros::Duration();
			edge_marker.color.a = 1.0;
			edge_marker.color.r = 1.0;
			edge_marker.color.g = 1.0;
			// edge_marker.color.g = 0.6;
			edge_marker.color.b = 0.0;
			edge_marker.scale.x = 0.02;
			edge_marker.scale.y = 0.02;
			edge_marker.scale.z = 0.02;
			edge_marker.pose.orientation.w = 1;
			edge_marker.points.clear();
			// Node
			node_marker.header.frame_id = global_frame;
			node_marker.header.stamp = ros::Time::now();
			node_marker.type = node_marker.CUBE_LIST;
			node_marker.action = node_marker.ADD;
			node_marker.lifetime = ros::Duration();
			node_marker.color.a = 1.0;
			node_marker.color.r = 0.0;
			node_marker.color.g = 0.0;
			node_marker.color.b = 1.0;
			node_marker.scale.x = 0.06;
			node_marker.scale.y = 0.06;
			node_marker.scale.z = 0.06;
			node_marker.pose.orientation.w = 1;
			node_marker.points.clear();
			// Leaf
			leaf_marker.header.frame_id = global_frame;
			leaf_marker.header.stamp = ros::Time::now();
			leaf_marker.type = leaf_marker.CUBE_LIST;
			leaf_marker.action = leaf_marker.ADD;
			leaf_marker.lifetime = ros::Duration();
			leaf_marker.color.a = 1.0;
			leaf_marker.color.r = 1.0;
			leaf_marker.color.g = 0.0;
			leaf_marker.color.b = 0.0;
			leaf_marker.scale.x = 0.08;
			leaf_marker.scale.y = 0.08;
			leaf_marker.scale.z = 0.08;
			leaf_marker.pose.orientation.w = 1;
			leaf_marker.points.clear();
			// Root node
			root_marker.header.frame_id = global_frame;
			root_marker.header.stamp = ros::Time::now();
			root_marker.type = root_marker.SPHERE;
			root_marker.action = root_marker.ADD;
			root_marker.lifetime = ros::Duration();
			root_marker.color.a = 1.0;
			root_marker.color.r = 1.0;
			root_marker.color.g = 0.6;
			root_marker.color.b = 0.0;
			root_marker.scale.x = 0.5;
			root_marker.scale.y = 0.5;
			root_marker.scale.z = 0.5;
			root_marker.pose.orientation.w = 1;
			// Obstacle node
			obstacle_node_marker.header.frame_id = global_frame;
			obstacle_node_marker.header.stamp = ros::Time::now();
			obstacle_node_marker.type = obstacle_node_marker.SPHERE_LIST;
			obstacle_node_marker.action = obstacle_node_marker.ADD;
			obstacle_node_marker.lifetime = ros::Duration();
			obstacle_node_marker.color.a = 0.8;
			obstacle_node_marker.color.r = 1.0;
			obstacle_node_marker.color.g = 0.0;
			obstacle_node_marker.color.b = 0.0;
			obstacle_node_marker.scale.x = inflationRadius;
			obstacle_node_marker.scale.y = inflationRadius;
			obstacle_node_marker.scale.z = 0.1;
			obstacle_node_marker.pose.orientation.w = 1;
			obstacle_node_marker.points.clear();
			//	Add data
			geometry_msgs::Point point_3d;
			for(auto iter2=edge_set.begin(); iter2!=edge_set.end(); iter2++){
				point_3d.x = (*iter2)->fromNode->state.x;
				point_3d.y = (*iter2)->fromNode->state.y;
				point_3d.z = (*iter2)->fromNode->state.z;
				edge_marker.points.push_back(point_3d);
				point_3d.x = (*iter2)->toNode->state.x;
				point_3d.y = (*iter2)->toNode->state.y;
				point_3d.z = (*iter2)->toNode->state.z;		
				edge_marker.points.push_back(point_3d);
			}

			for(auto iter1=node_set.begin(); iter1!=node_set.end(); iter1++){
				point_3d.x = (*iter1)->state.x;
				point_3d.y = (*iter1)->state.y;
				point_3d.z = (*iter1)->state.z;
				if((*iter1)->nodeState == NodeState::LEAF){
					leaf_marker.points.push_back(point_3d);
					node_marker.points.push_back(point_3d);
					continue;
				}
				// if((*iter1)->nodeState == NodeState::OBSTACLE){
				// 	obstacle_node_marker.points.push_back(point_3d);
				// 	continue;
				// }
				if((*iter1)->nodeState == NodeState::ROOT){
					root_marker.pose.position = point_3d;
					continue;
				}
				if((*iter1)->nodeState == NodeState::NORMAL){
					node_marker.points.push_back(point_3d);
					continue;
				}
			}

			for(auto iter2=obstacle_node_set.begin(); iter2!=obstacle_node_set.end(); iter2++){
				point_3d.x = (*iter2)->state.x;
				point_3d.y = (*iter2)->state.y;
				point_3d.z = (*iter2)->state.z;
				obstacle_node_marker.points.push_back(point_3d);
			}

			pub_rtrrt_edge.publish(edge_marker);
			pub_rtrrt_node.publish(node_marker);
			pub_leaf_node.publish(leaf_marker);
			pub_root_marker.publish(root_marker);
			pub_obstacle_node.publish(obstacle_node_marker);
		}
	}

	if(pub_frontier_node.getNumSubscribers()!=0 && !frontierNodes.empty()){
		visualization_msgs::Marker frontier_marker;	// frontier represent by sphere
		frontier_marker.header.frame_id = global_frame;
		frontier_marker.header.stamp = ros::Time::now();
		frontier_marker.type = frontier_marker.SPHERE_LIST;
		frontier_marker.action = frontier_marker.ADD;
		frontier_marker.lifetime = ros::Duration();
		frontier_marker.color.a = 1.0;
		frontier_marker.color.r = 0.0;
		frontier_marker.color.g = 1.0;
		frontier_marker.color.b = 0.0;
		frontier_marker.scale.x = 0.3;
		frontier_marker.scale.y = 0.3;
		frontier_marker.scale.z = 0.3;
		frontier_marker.pose.orientation.w = 1;
		frontier_marker.points.clear();

		geometry_msgs::Point frontier_p;
		auto iter = frontierNodes.begin();
		while(iter != frontierNodes.end()){
			frontier_p.x = iter->first->state.x;
			frontier_p.y = iter->first->state.y;
			frontier_p.z = iter->first->state.z;
			frontier_marker.points.push_back(frontier_p);
			iter++;
		}
		pub_frontier_node.publish(frontier_marker);
	}

	publishRemovedItem();

}


















