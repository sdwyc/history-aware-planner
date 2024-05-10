#include <ros/ros.h>
#include "graph_planner/graph_planner.h"

graph_planner::graph_planner():
global_frame(ParamLoader::globalFrame), base_frame(ParamLoader::baseFrame),
receivedGridMap(false), updatedPose(false), receivedNewVertices(false),
homeSaved(false),
kdtree_pts(NULL), kdtree_(NULL),
robot_pose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
{
	init();
	ROS_INFO("Global Planner initialization accomplished");
}

graph_planner::~graph_planner(){}

void graph_planner::init(){
	// KDtree init
    kdtree_pts = make_shared<KDTreePt::PointCloud<float>>();
    kdtree_ = make_shared<nanoKDTree>(3, 10, kdtree_pts);
	
	graphVertices.reset(new pcl::PointCloud<PointFrontier>());
	treeNodes.reset(new pcl::PointCloud<PointFrontier>());

	// Publisher init
	pub_root_marker = nh.advertise<visualization_msgs::Marker>("/graph_root", 10);
	pub_frontier_node = nh.advertise<visualization_msgs::Marker>("/global_frontier", 10);
	pub_path = nh.advertise<nav_msgs::Path>("/reselection_path",10);
	pub_subgoal_marker = nh.advertise<visualization_msgs::Marker>("/subgoal_marker",10);
	pub_vertices = nh.advertise<sensor_msgs::PointCloud2>("graph_vertices", 10);
	pub_edge = nh.advertise<sea_planner::Graph>("/graph_edge",10);
	// Subscriber init
	sub_grid_map = nh.subscribe("/elevation_mapping/elevation_map", 1, &graph_planner::gridMapHandler, this);
	sub_goal = nh.subscribe("/goal_marker", 1, &graph_planner::goalHandler, this);
	sub_new_vertices = nh.subscribe("/new_vertices", 1, &graph_planner::newVerticesHandler, this);
	sub_local_frontier = nh.subscribe("/tree_nodes", 1, &graph_planner::treeNodesHandler, this);
	sub_robotpose = nh.subscribe("/base_link_pose", 1, &graph_planner::updateRobotPoseHandle, this);
	// Timer init
	ros::Rate Rater(15);
	// poseUpdater = nh.createTimer(Rater, &graph_planner::updateRobotPoseHandle, this);
    
	// Service init
    planningFinder = nh.advertiseService("global_graph/get_global_path", &graph_planner::findPath, this);
    nearestFinder = nh.advertiseService("global_graph/get_nearest_info", &graph_planner::getNearest, this);

}

void graph_planner::publishEdges(){
	if(!edge_set.empty() && pub_edge.getNumSubscribers()!=0){
		vector<sea_planner::Edge> edge_set_;
		vector<sea_planner::Vertex> vertices_set_;
		for(int i=0; i<edge_set.size(); i++){
			sea_planner::Vertex v1, v2;
			v1.location = edge_set[i]->Vertex_1->location;
			v2.location = edge_set[i]->Vertex_2->location;
			sea_planner::Edge e_;
			e_.vertex_1 = v1;
			e_.vertex_2 = v2;
			edge_set_.push_back(e_);
		}
		for(int j=0; j<node_set.size(); j++){
			sea_planner::Vertex v;
			v.location = node_set[j]->location;
			vertices_set_.push_back(v);
		}
		sea_planner::Graph edge_;
		edge_.header.frame_id = global_frame;
		edge_.header.stamp = ros::Time::now();
		edge_.edges = edge_set_;
		edge_.vertices = vertices_set_;
		pub_edge.publish(edge_);
	}
}

void graph_planner::executeGraphUpdate(){
	// TODO saveHome
	// receied new vertices
	if(receivedNewVertices && !rawVertices.empty()){
		// Iterate all vertices, add to G one by one
		if(node_set.empty()){
			// The first iteration, new vertex is origin
			// Add to kdtree for search
			KDTreePt::Point<float> pt(rawVertices[0].location.x, rawVertices[0].location.y, 0.0);
			kdtree_pts->points.push_back(pt);
			kdtree_->AddPoint(kdtree_pts->points.size()-1, kdtree_pts->points.size()-1);
			// Graph initialization, add origin node
			graph_ns::Vertex::Ptr origin_vertex = make_shared<graph_ns::Vertex>();
			origin_vertex->vertex_id = kdtree_pts->points.size();
			origin_vertex->location.x = rawVertices[0].location.x;
			origin_vertex->location.y = rawVertices[0].location.y;
			origin_vertex->location.z = rawVertices[0].location.z;
			origin_vertex->information_gain = rawVertices[0].information_gain;
			node_set.push_back(origin_vertex);
			origin = origin_vertex;
			// Add to PCL pointcloud
			origin_vertex->pcl_pt.x = origin_vertex->location.x;
			origin_vertex->pcl_pt.y = origin_vertex->location.y;
			origin_vertex->pcl_pt.z = 0.0;
			origin_vertex->pcl_pt.intensity = origin_vertex->information_gain;
			graphVertices->points.push_back(origin_vertex->pcl_pt);
			return;
		}
		
		// Add graph vertices
		Vertices new_vertices;
		std::cout << "Before add vertex " << std::endl;
		addVertex(rawVertices, new_vertices);
		std::cout << "node_set: " << node_set.size() << std::endl;

		// Add edge for each vertex
		if(!new_vertices.empty()){
			for(auto it=new_vertices.begin(); it!=new_vertices.end(); it++){
				// Find Neibor
				vector<graph_ns::Vertex::Ptr> neibor_set;
				float query_pt[3] = {(*it)->location.x, (*it)->location.y, 0.0};
				getNeiborVertex(query_pt, neiborRangeThre, neibor_set);
				// std::cout << "neibor_set: " << neibor_set.size() << std::endl;
				// Add and Check vertex & edge
				for(int i=0; i<neibor_set.size(); i++){
					if(addEdge(*it, neibor_set[i])){
						// Publish edge information
						// std::cout << "addVertexEdge successful! " << std::endl;
						publishEdges();
					}
				}
				// std::cout << "Added edges " << std::endl;
				neibor_set.clear();
			}
			std::cout << "Before removeInvlidVertex" << std::endl;
			
			// Remove invalid vertices
			removeInvlidVertex(new_vertices);
		}
	}
	util::publishCloud(pub_vertices, graphVertices, ros::Time::now(), global_frame);
	receivedNewVertices = false;
}

void graph_planner::getNeiborVertex(float query_pt[3], float radius, vector<graph_ns::Vertex::Ptr>& neibor_vertices){
	vector<size_t> indices;
    vector<float> dists_sq;
	// Radius search by kd-tree
	kdtree_->RadiusSearch(query_pt, radius, indices, dists_sq);
	// indices index sequence is same to Vertices set
	for(int i=0; i<indices.size(); i++){
		neibor_vertices.push_back(node_set[indices[i]]);
	}
}
// Path Sequence: start -> end
Vertices graph_planner::findBestPath(graph_ns::Vertex::Ptr x_start, graph_ns::Vertex::Ptr x_end){
	// Start point != end
	Vertices result_path;
	if(x_start == x_end){
		printf("Start point and end points are same! No executable path\n");
		return result_path;
	}
	if(x_start->vertex_edge_list.find(x_end->vertex_id) != x_start->vertex_edge_list.end()){
		result_path.push_back(x_start);
		result_path.push_back(x_end);
		return result_path;
	}
	// Cost to the vertex and corresponding vertex
	typedef pair<float, graph_ns::Vertex::Ptr> cost_pair;
	
	// Priority queue of vertices
  	priority_queue<cost_pair, vector<cost_pair>, greater<cost_pair>> cost_q;
  
	// Vector of distances
	vector<float> dist(node_set.size(), INFINITY);

	// Vector of backpointers
	vector<size_t> backpointers(node_set.size(), std::numeric_limits<size_t>::max());

	// Add start point to queue
	cost_q.push(make_pair(0, x_start));
	auto iter1 = find(node_set.begin(), node_set.end(), x_start);
	dist[iter1-node_set.begin()] = 0;

	// Find path
	while(!cost_q.empty()){
		// Pop top element
		graph_ns::Vertex::Ptr u = cost_q.top().second;
		auto iter_u = find(node_set.begin(), node_set.end(), u);
		size_t ind_u = iter_u-node_set.begin();
		cost_q.pop();

		// Check all neibor nodes
		for(auto it=u->vertex_edge_list.begin(); it!=u->vertex_edge_list.end(); it++){
			// Get corresponding edge and cost
			graph_ns::Edge::Ptr e = (*it).second;
			graph_ns::Vertex::Ptr v = e->Vertex_1==u? e->Vertex_2 : e->Vertex_1;
			auto iter_v = find(node_set.begin(), node_set.end(), v);
			size_t ind_v = iter_v-node_set.begin();
			float traversal_cost = e->length + fabs(e->elevation_diff);
			
      		// If there is a shorter path to node through u
			if(dist[ind_v] > dist[ind_u]+traversal_cost){
				// Updating v
				dist[ind_v] = dist[ind_u]+traversal_cost;
				cost_q.push(make_pair(dist[ind_v], v));
				backpointers[ind_v] = ind_u;
			}
		}

		// Arrve end point, Stop
		if (u == x_end){
			break;
		}
	}
	
	// Retrace to best path
	auto iter_end = find(node_set.begin(), node_set.end(), x_end);
	size_t current = iter_end-node_set.begin();
	// if(backpointers[current] == INF){
	if(dist[current] == INFINITY){
		// No feasible Path
		result_path.clear();
		return result_path;
	}
	else{
		// Path found
		while(current != std::numeric_limits<size_t>::max()){
			result_path.insert(result_path.begin(), node_set[current]);
			current = backpointers[current];
		}
		return result_path;
	}
}

bool graph_planner::addVertex(vector<sea_planner::Vertex> raw_vertices, Vertices& new_vertices){
	
	for(int i=0; i< raw_vertices.size(); i++){
		float query_pt[3] = {raw_vertices[i].location.x, raw_vertices[i].location.y, 0.0};
		vector<size_t> indices;
		vector<float> dists_sq;
		kdtree_->NearestSearch(query_pt, 1, indices, dists_sq);
		// if(dists_sq[0] < 1.0e-4){
		if(dists_sq[0] < minNodeDistance/4){
		// if(dists_sq[0] < minNodeDistance){
			// Avoid too close vertex
			// if(dists_sq[0] < 1.0e-4){
			// 	// Update corresponding vertex information gain
			// 	graph_ns::Vertex* v = id_vertex.at(indices[0]);
			// 	v->information_gain = raw_vertices[i].information_gain;
			// 	// TODO Check this correct
			// }
			continue;
		}
		// Add to Kdtree
		KDTreePt::Point<float> pt(raw_vertices[i].location.x, raw_vertices[i].location.y, 0.0);
		kdtree_pts->points.push_back(pt);
		kdtree_->AddPoint(kdtree_pts->points.size()-1, kdtree_pts->points.size()-1);
		// Add vertices to node set
		graph_ns::Vertex::Ptr new_v = make_shared<graph_ns::Vertex>();
		new_v->location.x = raw_vertices[i].location.x;
		new_v->location.y = raw_vertices[i].location.y;
		new_v->location.z = raw_vertices[i].location.z;
		new_v->information_gain = raw_vertices[i].information_gain;
		new_v->vertex_id = kdtree_pts->points.size();
		node_set.push_back(new_v);
		new_vertices.push_back(new_v);
		id_vertex.insert(make_pair(kdtree_pts->points.size()-1, new_v));
		
		new_v->pcl_pt.x = raw_vertices[i].location.x;
		new_v->pcl_pt.y = raw_vertices[i].location.y;
		new_v->pcl_pt.z = 0.0;
		new_v->pcl_pt.intensity = raw_vertices[i].information_gain;
		
		// If new_v is global frontier, add to unique set
		if(new_v->information_gain != 0 ){
			GlobalFrontier.insert(make_pair(new_v, new_v->information_gain));
		}
	}
	return true;
}

bool graph_planner::addEdge(graph_ns::Vertex::Ptr x_new, graph_ns::Vertex::Ptr x_parent){
	bool outcome = false;	// Whether or not add node-edge successfully
	graph_ns::Edge::Ptr parent_to_new = std::make_shared<graph_ns::Edge>();
	parent_to_new->elevation_diff = x_new->location.z - x_parent->location.z;
	float dis_new_parent = distance(x_new->location.x, x_parent->location.x, x_new->location.y, x_parent->location.y);
	parent_to_new->length = dis_new_parent;
	parent_to_new->gradient = util::gradient(dis_new_parent, parent_to_new->elevation_diff);

	// Check edge quality and repeat
	if(checkEdge(*parent_to_new, x_parent, x_new)){
		// Check pass, add edge
		// Edge setting
		parent_to_new->Vertex_1 = x_parent;
		parent_to_new->Vertex_2 = x_new;
		x_new->vertex_edge_list.insert(make_pair(x_parent->vertex_id, parent_to_new));
		x_parent->vertex_edge_list.insert(make_pair(x_new->vertex_id, parent_to_new));
		edge_set.push_back(parent_to_new);
		outcome = true;
	}
	else parent_to_new = NULL;
	return outcome;
}

bool graph_planner::removeInvlidVertex(Vertices new_vertices){
	// Iterate all new vertices
	int count = 0;
	for(int i=0; i<new_vertices.size(); i++){
		// Check connection
		if(new_vertices[i]->vertex_edge_list.empty()){
			// Remove it from kdtree
			KDTreePt::Point<float> pt(new_vertices[i]->location.x, new_vertices[i]->location.y, 0.0);
			size_t ind = kdtree_pts->kdtree_get_point_index(pt);
			kdtree_->RemovePoint(ind);
			// Remove it from node_set
			auto iter = find(node_set.begin(), node_set.end(), new_vertices[i]);
			node_set.erase(iter);
			count++;
		}
		else{
			graphVertices->points.push_back(new_vertices[i]->pcl_pt);
		}
	}
	if(count){
		// printf("Removed %d invalid graph vertices!\n", count);
	}
	return true;
}

bool graph_planner::checkEdge(graph_ns::Edge edge, graph_ns::Vertex::Ptr v1, graph_ns::Vertex::Ptr v2){
	// Check repeat
	auto v1_res = v1->vertex_edge_list.find(v2->vertex_id);
	auto v2_res = v2->vertex_edge_list.find(v1->vertex_id);
	if(v1_res != v1->vertex_edge_list.end() || v2_res != v2->vertex_edge_list.end()){
		// Repeat edge
		return false;
	}

	// Check gradient
	if(fabs(edge.gradient) > gradientDiffThre){
		return false;
	}
	// Avoid edge is too short
	float d = distance(v1->location.x, v2->location.x, v1->location.y, v2->location.y);
	if(d>neiborRangeThre || d<minNodeDistance){
		return false;
	}
	
	return true;
}

void graph_planner::updateRobotPoseHandle(const geometry_msgs::PoseWithCovarianceStampedConstPtr &new_pose)
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

void graph_planner::gridMapHandler(const grid_map_msgs::GridMapConstPtr &raw_map){
	GridMapRosConverter::fromMessage(*raw_map, full_grid_map);
	map_x = raw_map->info.length_x;
	map_y = raw_map->info.length_y;
	map_resolution = raw_map->info.resolution;
	receivedGridMap = true;
}

void graph_planner::newVerticesHandler(const sea_planner::GraphConstPtr &vertices){
	rawVertices.clear();
	// save new vertices
	// std::cout << "vertices: " << vertices->vertices.size() << std::endl;
	rawVertices = vertices->vertices;
	receivedNewVertices = true;
	executeGraphUpdate();
}

void graph_planner::goalHandler(const visualization_msgs::MarkerConstPtr &goal_mark){
	newGoalReceived = true;
	goal = goal_mark->pose;
}

void graph_planner::treeNodesHandler(const sensor_msgs::PointCloud2ConstPtr &node_msgs){
	treeNodes->points.clear();
	pcl::fromROSMsg(*node_msgs, *treeNodes);
}

graph_ns::Vertex::Ptr graph_planner::getNearestVertex(float point_x, float point_y){
	float query_pt[3]={point_x, point_y, 0.0};
	vector<size_t> indices;
	vector<float> dists_sq;
	kdtree_->NearestSearch(query_pt, 1, indices, dists_sq);
	if(indices.empty()){
		return NULL;
	}

	graph_ns::Vertex::Ptr nearestVertex = id_vertex.at(indices[0]);
	return nearestVertex;
}

bool graph_planner::findPath(sea_planner::get_path::Request& req_target,
			sea_planner::get_path::Response& res_path){
	printf("11111111111111111!\n");
	auto start_vertex = getNearestVertex(robot_pose.x, robot_pose.y);
	printf("2222222222222222!\n");
	auto target_vertex = getNearestVertex(req_target.target_point.x, req_target.target_point.y);
	printf("3333333333333333333!\n");
	std::cout << start_vertex <<", " << target_vertex << std::endl;
	Vertices shortest_path = findBestPath(start_vertex, target_vertex);
	std::cout << shortest_path.size() << std::endl;
	nav_msgs::Path target_path;
	target_path.header.frame_id = "map";
	target_path.header.stamp = ros::Time::now();
	for(auto vertex : shortest_path){
		// Start -> End
		geometry_msgs::PoseStamped v_pos;
		v_pos.pose.position = vertex->location;
		// target_path.poses.insert(target_path.poses.begin(), v_pos); 
		target_path.poses.push_back(v_pos); 
	}
	res_path.global_path = target_path;
	pub_path.publish(target_path);
	return true;
}

bool graph_planner::getNearest(sea_planner::get_path::Request& req_target,
			sea_planner::get_path::Response& res_path){

	return true;
}
