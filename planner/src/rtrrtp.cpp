#include "sea_planner/rtrrtp.h"
using namespace termcolor;

void rtrrtp::init(){
    rtRRT.reset(new rtrrt);
    target_node.reset();
    global_frame = globalFrame;
    base_frame = baseFrame;
    useFinalGoal = false;
    useSubGoal = false;
    HasNewVertices = false;
    cur_state = PlannerState::WAITING;
    // Publisher init
	pub_rtrrt_path = nh.advertise<nav_msgs::Path>("/rtrrt_path",10);
    pub_selected_branch = nh.advertise<visualization_msgs::MarkerArray>("selected_branch",10);
    pub_ref_line = nh.advertise<visualization_msgs::Marker>("reference_line",10);
    pub_fake_path = nh.advertise<visualization_msgs::Marker>("expected_path", 10);
	pub_waypoint = nh.advertise<geometry_msgs::PointStamped>("way_point", 10);
	pub_waypoint_marker = nh.advertise<visualization_msgs::Marker>("way_point_marker", 10);
    pub_goal_marker = nh.advertise<visualization_msgs::Marker>("Goal", 10);
    pub_target_marker = nh.advertise<visualization_msgs::Marker>("target", 10);
    pub_global_frontier = nh.advertise<visualization_msgs::Marker>("global_frontier", 10);
	pub_graph_vertices = nh.advertise<sea_planner::Graph>("new_vertices", 10);
    // Subscriber init
    sub_goal = nh.subscribe("/move_base_simple/goal",1, &rtrrtp::goalHandler,this);
    sub_robotpose = nh.subscribe("/base_link_pose",1, &rtrrtp::updateRobotPoseCB,this);
    // Service init
    stopServer = nh.advertiseService("sea_planner/stop_planning", &rtrrtp::stopPlanner, this);
    resetServer = nh.advertiseService("sea_planner/reset_planner", &rtrrtp::resetPlanner, this);
    getStateServ = nh.advertiseService("sea_planner/get_state", &rtrrtp::getStateHandler, this);
    reselectionTri = nh.serviceClient<sea_planner::get_path::Request, sea_planner::get_path::Response>("global_graph/get_global_path");

    ros::Rate Rater(15);
    ros::Rate visRater(2);
	// poseUpdater = nh.createTimer(Rater, &rtrrtp::updateRobotPoseCB, this);
	// visualizer = nh.createTimer(visRater, &rtrrtp::visualizationCB, this);
    last_time = ros::Time::now();
}

rtrrtp::rtrrtp(){
    init();
    std::cout << bold << green
              << "SEAP initialization accomplished"
              << reset << std::endl;
    std::thread executeThread(&rtrrtp::ExecuteThread, this);
    executeThread.detach(); // Detach the sub-thread for parallelism
}

rtrrtp::~rtrrtp(){
    rtRRT.reset(new rtrrt);
    closeList.clear();
    global_frame.clear();
    selectedBranch_nodes.clear();
}

void rtrrtp::resetResource(){
    std::cout << "Delete rtRRT" << std::endl;
    rtRRT.reset(new rtrrt);
    target_node.reset();
    global_frame = globalFrame;
    base_frame = baseFrame;
    useFinalGoal = false;
    useSubGoal = false;
    cur_state = PlannerState::WAITING;
}

bool rtrrtp::resetPlanner(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res){
    if(cur_state == PlannerState::ACTIVE){
        ROS_WARN("Planner is active! Please stop planner firstly.");
        return false;
    }
    std::cout << cyan 
              << "Reseting planner"
              << reset << std::endl;
    mtx.lock();
    rtRRT->sub_grid_map.shutdown(); // Close grid map spin Channel
    closeList.clear();
    globalFrontier.clear();
    resetResource();
    mtx.unlock();
    return true;
}

bool rtrrtp::stopPlanner(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res){
    std::cout << yellow 
              << "Stop path planning"
              << reset << std::endl;
    mtx.lock();
    target_node = NULL;
    cur_state = PlannerState::WAITING;
    newGoalReceived = false;
    rtRRT->newGoalReceived = false;
    mtx.unlock();
    stayCurrentPose();
    return true;
}

bool rtrrtp::getStateHandler(sea_planner::get_state::Request& req,
                             sea_planner::get_state::Response& state_res){
    state_res.stamp = ros::Time::now();
    if(cur_state == PlannerState::ACTIVE){
        state_res.state = "ACTIVE";
    }
    if(cur_state == PlannerState::WAITING){
        state_res.state = "WAITING";
    }
    state_res.estimated_distance = distance(robot_pose.x, rtRRT->goal.position.x, robot_pose.y, rtRRT->goal.position.y);
    return true;
}

void rtrrtp::ExecuteThread(){
    while(ros::ok()){
        if(cur_state != PlannerState::WAITING){
            float timeStart = ros::Time::now().toSec();
            executeCycle();
            float timeEnd = ros::Time::now().toSec();
            if(cur_state == PlannerState::WAITING){
                ROS_INFO("Navigation Duration: %f", timeEnd-timeStart);
                std::cout << bold << green 
                            << "Arrived Goal" 
                            << reset << std::endl;
            }
            mtx.lock();
            newGoalReceived = false;
            rtRRT->newGoalReceived = false;
            cur_state = PlannerState::WAITING;  // Reset flag
            mtx.unlock();
        }
    }
}

void rtrrtp::stayCurrentPose(){
    way_point.header.frame_id = global_frame;
    way_point.header.stamp = ros::Time::now();
    way_point.point.x = robot_pose.x;
    way_point.point.y = robot_pose.y;
    way_point.point.z = robot_pose.z;
    pub_waypoint.publish(way_point);
}

void rtrrtp::executeCycle(){
    if(updatedPose){
        // Navigate to final goal
        // bool record_gen_time = false;
	    // float record_generation_start = ros::Time::now().toSec();
        int random_counter = 0;
        nav_msgs::Path target_path;
        float disToGoal = distance(robot_pose.x, rtRRT->goal.position.x, robot_pose.y, rtRRT->goal.position.y);
        while(cur_state != PlannerState::WAITING && 
              disToGoal > goalTolerance && ros::ok()){
            // For Initializing Tree with enough nodes and edge
            while(rtRRT->node_set.size()<=minTreeNodeNum && ros::ok()){
                if(rtRRT->node_set.empty()){
                    HasNewVertices = true;
                }
                // ros::Duration(0.01).sleep(); // Control rrt grow speed
                rtRRT->executeTreeUpdate();
                publishNewVertices();
            }
            printf("ExecuteCycle iteration start\n");
            // Select navigation type: goal or subgoal
            float timeKeeper = ros::Time::now().toSec();
            selectNavType();
            printf("Selected target type\n");
            if(!globalFrontier.empty()){
                validateFrontier();
            }
            printf("Select subgoal\n");
            if(useSubGoal && !useFinalGoal && rtRRT->current_target_node==NULL){
                // while((ros::Time::now().toSec() - timeKeeper) < 0.1){
                //     // For exploring env, prolong the time for tree growing 
                // }
                rtRRT->executeTreeUpdate();

                //! Random node as target exp2
                // random_counter--;
                // if(random_counter < -5){
                //     while(rtRRT->current_target_node==NULL){
                //         globalFrontier.clear();
                //         rtRRT->getFrontierNode();
                //         if(rtRRT->frontierNodes.size()!=0){
                //             int index = round((((double)rand()) / ((double)RAND_MAX))*(rtRRT->frontierNodes.size()-1));
                //             for(auto it=rtRRT->frontierNodes.begin(); it!=rtRRT->frontierNodes.end(); it++){
                //                 index--;
                //                 if(index<=0){
                //                     rtRRT->current_target_node = (*it).first;
                //                     break;
                //                 }
                //             }
                //         }
                //     }
                //     random_counter = 1;
                // }
                // else{
                    //! Ours frontier method
                // std::cout << "3333333333333333333" << std::endl;
                rtRRT->getFrontierNode();
                // std::cout << "4444444444444444444" << std::endl;
                localFrontier.clear();
                localFrontier = rtRRT->frontierNodes;
                unordered_map<rtrrt_ns::Node::Ptr, float>* CANDIDATA_SET = NULL;
                // Try to select subgoal from local frontier
                if(!localFrontier.empty()){
                    HasNewVertices = true;
                    publishNewVertices();

                    for(auto iter1=rtRRT->frontierNodes.begin(); iter1!=rtRRT->frontierNodes.end(); iter1++){
                        // No found, add to global frontier
                        if(globalFrontier.find((*iter1).first) == globalFrontier.end()){
                            globalFrontier.insert(*iter1);
                        }
                        // Found, add to global frontier after delete operation
                        else{
                            globalFrontier.erase((*iter1).first);
                            globalFrontier.insert(*iter1);
                        }
                    }
                    //! No longer delete all global frontier
                    // globalFrontier.clear();
                    // globalFrontier = localFrontier;
                    // std::cout << "55555555555555555" << std::endl;

                    // Find best frontier as target node
                    float totalDis = 0;
                    float totalGradient = 0;
                    for(auto iter2=localFrontier.begin(); iter2!=localFrontier.end(); iter2++){
                        // Except nodes in closeList
                        auto res = find(closeList.begin(), closeList.end(), (*iter2).first);
                        if(res != closeList.end()){
                            continue;
                        }
                        totalDis += (*iter2).first->disToRoot;
                        totalGradient += (*iter2).first->gradientToRoot;
                    }
                    float best_cost = inf;
                    rtrrt_ns::Node::Ptr best_node;
                    for(auto iter3=localFrontier.begin(); iter3!=localFrontier.end(); iter3++){
                        // Except nodes in closeList
                        //TODO evaluation Func should be desiged independently
                        float distance_heuristic = util::distance((*iter3).first->state.x, final_goal.pose.position.x, (*iter3).first->state.y, final_goal.pose.position.y);
                        float costVal = costFunc((*iter3).first->disToRoot, distance_heuristic, (*iter3).first->gradientToRoot, totalDis, totalGradient);
                        //! Nearest node for Goal (Greedy)
                        // float costVal = util::distance((*iter3).first->state.x, final_goal.pose.position.x, (*iter3).first->state.y, final_goal.pose.position.y);
                        //! Max info method
                        // float costVal = -1 * 0.9 * util::getInfoGain((*iter3).first->state.x, (*iter3).first->state.y, rtRRT->full_grid_map, "elevation") + 0.1* distance_heuristic;
                        //! Nearest frontier for robot
                        // float costVal = util::distance((*iter3).first->state.x, robot_pose.x, (*iter3).first->state.y, robot_pose.y);
                        if(costVal < best_cost){
                            best_cost = costVal;
                            best_node = (*iter3).first;
                        }
                    }
                    rtRRT->current_target_node = best_node;
                }
                    // std::cout << "66666666666666666" << std::endl;                }
                // Try to select subgoal from global frontier
                else if(!globalFrontier.empty()){
                    ROS_WARN("No local subgoals! Relocalzing in global!");
                    // Relocate the other global frontier
                    float total_dis;
                    float total_info;
                    unordered_map<rtrrt_ns::Node::Ptr, pair<float, float>> GFs;
                    // Calculate total information
                    for(auto gf=globalFrontier.begin(); gf!=globalFrontier.end(); gf++){
                        float disToGoal = distance(gf->first->state.x, rtRRT->goal.position.x, gf->first->state.y, rtRRT->goal.position.y);
                        GFs.insert(make_pair(gf->first, make_pair(disToGoal, gf->second))); // <distance, info_gain>
                        total_dis += disToGoal;
                        total_info += gf->second;
                    }
                    rtrrt_ns::Node::Ptr best_frontier = NULL;
                    double best_cost = inf;
                    double cost_per_gf;
                    for(auto GF=GFs.begin(); GF!=GFs.end(); GF++){
                        cost_per_gf = 5.0 * (GF->second.first);// + 1.0 * gf->second.second/total_info;
                        if(cost_per_gf < best_cost){
                            best_cost = cost_per_gf;
                            best_frontier = GF->first;
                        }
                    }

                    // std::cout << "get_path_srv.request.target_point: " << best_frontier << std::endl;
                    sea_planner::get_path get_path_srv;
                    get_path_srv.request.target_point = best_frontier->state;
                    // std::cout << "get_path_srv.request.target_point: " << get_path_srv.request.target_point.x << ", " << get_path_srv.request.target_point.y << std::endl;
                    // if(!reselectionTri.waitForExistence(ros::Duration(3.0))){
                    //     printf("Service no exist! Waiting! \n");
                    // }
                    // else{
                    if(reselectionTri.call(get_path_srv)){
                        printf("Service call successful! Path size:%d \n",get_path_srv.response.global_path.poses.size());
                        target_path.poses.clear();
                        target_path = get_path_srv.response.global_path;
                        target_path.header.frame_id = globalFrame;
                        rtRRT->current_target_node = best_frontier;
                        cur_state = PlannerState::RESELECTION;
                    }
                    // }
                }
                // No frontier. Stay
                else{
                    ROS_WARN("No any subgoals pointing to target!\n");
                    // Reset all resource
                    target_node = NULL;
                    rtRRT->current_target = NULL;
                    rtRRT->current_target_node = NULL;
                    return;
                }

            }
            // printf("Select Final goal\n");
            // If have node close enough to final_goal, it will become target
            if(!useSubGoal && useFinalGoal){
                // Update frontiers
                rtRRT->getFrontierNode();
                localFrontier.clear();
                localFrontier = rtRRT->frontierNodes;
                HasNewVertices = true;
                publishNewVertices();
                for(auto iter1=rtRRT->frontierNodes.begin(); iter1!=rtRRT->frontierNodes.end(); iter1++){
                    // No found, add to global frontier
                    if(globalFrontier.find((*iter1).first) == globalFrontier.end()){
                        globalFrontier.insert(*iter1);
                    }
                    // Found, add to global frontier after delete operation
                    else{
                        globalFrontier.erase((*iter1).first);
                        globalFrontier.insert(*iter1);
                    }
                }
                //! No longer delete all global frontier
                // globalFrontier.clear();
                // globalFrontier = localFrontier;

                rtrrt_ns::Node::Ptr closeNode = rtRRT->findTarget();
                if(closeNode != NULL){
                    rtRRT->current_target_node = closeNode;
                }
            }
            rtRRT->executeTreeUpdate();
            // Driving to target_node
            float DisToTarget = inf;
            if(cur_state!=PlannerState::WAITING){
                target_node = rtRRT->current_target_node;
                DisToTarget = util::distance(robot_pose.x, target_node->state.x, robot_pose.y, target_node->state.y);
            }
            // printf("Reselection step\n");
            bool state = ros::ok();
            while(cur_state == PlannerState::RESELECTION && DisToTarget > goalTolerance && state){
                // Release all tree resource
                if(target_node!=NULL && cur_state!=PlannerState::WAITING){
                    DisToTarget = util::distance(robot_pose.x, target_node->state.x, robot_pose.y, target_node->state.y);
                }
                // printf("111111111111111\n");
                mtx.lock();
                selectedBranch_nodes.clear();
                for(auto pos: target_path.poses){
                    rtrrt_ns::Node::Ptr path_node = make_shared<rtrrt_ns::Node>();
                    path_node->state = pos.pose.position;
                    // According to find parent one by one until retrace root node(positive sequence)
                    selectedBranch_nodes.push_back(path_node);
                }
                mtx.unlock();
                // printf("222222222222222222\n");
                // rtrrt_path direction = selected branch
                if(!selectedBranch_nodes.empty()){
                    publishPath();
                    publishWaypoint(rtrrt_path);
                }
                // printf("3333333333333333333\n");

                float disToWaypoint = util::distance(robot_pose.x, rtrrt_path.poses[1].pose.position.x, 
                                                     robot_pose.y, rtrrt_path.poses[1].pose.position.y);

                if(disToWaypoint < waypointTolerance){
                    if(target_path.poses.size()>2) target_path.poses.erase(target_path.poses.begin());
                }
                // printf("4444444444444444444\n");

                state = ros::ok();
            }
            // printf("Local planning step\n");
            while(cur_state == PlannerState::ACTIVE && DisToTarget > goalTolerance && state && target_node!=NULL && target_node->nodeState!=NodeState::OBSTACLE){
                // std::cout << "66666667777777777" << std::endl;
                rtRRT->executeTreeUpdate();
                // std::cout << "7777777777777777777" << std::endl;

                if(target_node != NULL){
                    // Found target node, output path
                    // if(!record_gen_time){
                    //     float record_generation_end = ros::Time::now().toSec();
                    //     std::cout << "The first path generation time: " << record_generation_end-record_generation_start << std::endl;
                    //     record_gen_time = true;
                    // }
                    searchPath();
                }
                // std::cout << "888888888888888888888" << std::endl;

                if(target_node!=NULL && cur_state!=PlannerState::WAITING){
                    DisToTarget = util::distance(robot_pose.x, target_node->state.x, robot_pose.y, target_node->state.y);
                    rtRRT->executeTreeUpdate();
                }
                // std::cout << "9999999999999999999999" << std::endl;
                // publishNewVertices();
                // std::cout << "10000000000000000000000" << std::endl;

                state = ros::ok();
            }
            // printf("Iteration end! Reset resource\n");
            if(useSubGoal && !useFinalGoal && rtRRT->current_target_node!=NULL){
                closeList.push_back(rtRRT->current_target_node);
            }
            if(useSubGoal && !useFinalGoal && cur_state == PlannerState::RESELECTION && rtRRT->current_target_node!=NULL){
                // Reset tree resource
                rtRRT->clearAll();
                cur_state = PlannerState::ACTIVE;
            }
            useFinalGoal = false;
            useSubGoal = false;
            target_node = NULL;
            rtRRT->current_target = NULL;
            rtRRT->current_target_node = NULL;
            
            // ros::Duration(0.02).sleep(); // Control rrt grow speed
            disToGoal = distance(robot_pose.x, rtRRT->goal.position.x, robot_pose.y, rtRRT->goal.position.y);
        }
        if(cur_state == PlannerState::ACTIVE){
            std::cout << "Node Num: " << rtRRT->node_set.size() << ", Edge Num: " << rtRRT->edge_set.size() << std::endl;
            std::cout << "Executed Path node num: " << Path_node.size() << std::endl;
        }
        // Reset all resource
        target_node = NULL;
        rtRRT->current_target = NULL;
        rtRRT->current_target_node = NULL;
        cur_state = PlannerState::WAITING;
    }
}

void rtrrtp::searchPath(){
    // Empty selectedBranch_nodes set for updating path
    selectedBranch_nodes.clear();
    if(target_node != NULL){
        rtrrt_ns::Node::Ptr current_iterator = target_node;
        while(current_iterator != NULL){
            // According to find parent one by one until retrace root node(positive sequence)
            if(selectedBranch_nodes.empty()){
                selectedBranch_nodes.push_back(current_iterator);
            }
            else{
                selectedBranch_nodes.insert(selectedBranch_nodes.begin(), current_iterator);
            }
            current_iterator = current_iterator->parent;
        }
        if(!selectedBranch_nodes.empty()){
            publishPath();
            publishWaypoint(rtrrt_path);
        }
    }
    // Change rtrrt root
    if(!selectedBranch_nodes.empty()){
        if(selectedBranch_nodes.size()>1){
            float dis = util::distance(robot_pose.x, selectedBranch_nodes[1]->state.x, robot_pose.y, selectedBranch_nodes[1]->state.y);
            if(dis < changRootTolerance && rtRRT->root != selectedBranch_nodes[1]){
                rtRRT->changeRoot(selectedBranch_nodes[1]);
                rtRRT->pruneTree(selectedBranch_nodes[1]);
                HasNewVertices = true;
                Path_node.push_back(rtRRT->root);
            }
        }
        else{
            float dis = util::distance(robot_pose.x, selectedBranch_nodes[0]->state.x, robot_pose.y, selectedBranch_nodes[0]->state.y);
            if(dis < changRootTolerance && rtRRT->root != selectedBranch_nodes[0]){
                rtRRT->changeRoot(selectedBranch_nodes[0]);
                rtRRT->pruneTree(selectedBranch_nodes[0]);
                HasNewVertices = true;
                Path_node.push_back(rtRRT->root);
            }
        }
    }
}

void rtrrtp::publishPath(){
    rtrrt_path.header.frame_id = global_frame;
    rtrrt_path.header.stamp = ros::Time::now();
    rtrrt_path.poses.clear();
    geometry_msgs::PoseStamped single_pose;
    if(selectedBranch_nodes.size()>1){
        // Add robot pose to rtrrt_path
        single_pose.header.frame_id = global_frame;
        single_pose.header.stamp = ros::Time::now();
        single_pose.pose.position.x = robot_pose.x;
        single_pose.pose.position.y = robot_pose.y;
        single_pose.pose.position.z = robot_pose.z; 
        tf::Quaternion tf_q = tf::createQuaternionFromYaw(robot_pose.yaw).normalized();
        single_pose.pose.orientation.x = tf_q.getX();
        single_pose.pose.orientation.y = tf_q.getY();
        single_pose.pose.orientation.z = tf_q.getZ();
        single_pose.pose.orientation.w = tf_q.getW();
        rtrrt_path.poses.push_back(single_pose);
        // Add rtrrt node except root node
        for(auto iter=selectedBranch_nodes.begin()+1; iter!=selectedBranch_nodes.end(); iter++){
            single_pose.header.frame_id = global_frame;
            single_pose.header.stamp = ros::Time::now();
            // Get position
            single_pose.pose.position.x = (*iter)->state.x;
            single_pose.pose.position.y = (*iter)->state.y;
            single_pose.pose.position.z = robot_pose.z;
            // Get quaternion
            if(iter == selectedBranch_nodes.end()-1){
                float deltY = (*iter)->state.y - (*(iter-1))->state.y;
                float deltX = (*iter)->state.x - (*(iter-1))->state.x;
                float yaw_angle = atan2(deltY, deltX);
                tf::Quaternion tf_q = tf::createQuaternionFromYaw(yaw_angle).normalized();
                single_pose.pose.orientation.x = tf_q.getX();
                single_pose.pose.orientation.y = tf_q.getY();
                single_pose.pose.orientation.z = tf_q.getZ();
                single_pose.pose.orientation.w = tf_q.getW();    
                rtrrt_path.poses.push_back(single_pose);

                // Add final_goal pose to path
                // single_pose.header.frame_id = global_frame;
                // single_pose.header.stamp = ros::Time::now();
                // // Get position
                // single_pose.pose.position.x = rtRRT->current_target->x;
                // single_pose.pose.position.y = rtRRT->current_target->y;
                // single_pose.pose.position.z = robot_pose.z;
                // rtrrt_path.poses.push_back(single_pose);
            }
            else{
                float deltY = (*(iter+1))->state.y - (*iter)->state.y;
                float deltX = (*(iter+1))->state.x - (*iter)->state.x;
                float yaw_angle = atan2(deltY, deltX);
                tf::Quaternion tf_q = tf::createQuaternionFromYaw(yaw_angle).normalized();
                single_pose.pose.orientation.x = tf_q.getX();
                single_pose.pose.orientation.y = tf_q.getY();
                single_pose.pose.orientation.z = tf_q.getZ();
                single_pose.pose.orientation.w = tf_q.getW();    
                rtrrt_path.poses.push_back(single_pose);
            }
        }
    }
    else{
        // Add robot pose to rtrrt_path
        single_pose.header.frame_id = global_frame;
        single_pose.header.stamp = ros::Time::now();
        single_pose.pose.position.x = robot_pose.x;
        single_pose.pose.position.y = robot_pose.y;
        single_pose.pose.position.z = robot_pose.z; 
        tf::Quaternion tf_q = tf::createQuaternionFromYaw(robot_pose.yaw).normalized();
        single_pose.pose.orientation.x = tf_q.getX();
        single_pose.pose.orientation.y = tf_q.getY();
        single_pose.pose.orientation.z = tf_q.getZ();
        single_pose.pose.orientation.w = tf_q.getW();
        rtrrt_path.poses.push_back(single_pose);
        // Add final_goal pose to path
        single_pose.header.frame_id = global_frame;
        single_pose.header.stamp = ros::Time::now();
        // Get position
        single_pose.pose.position.x = target_node->state.x;
        single_pose.pose.position.y = target_node->state.y;
        single_pose.pose.position.z = target_node->state.z;
        rtrrt_path.poses.push_back(single_pose);
    }
    pub_rtrrt_path.publish(rtrrt_path);
}

void rtrrtp::publishWaypoint(const nav_msgs::Path &target_path){
        float disTopoint = util::distance(robot_pose.x, target_path.poses[1].pose.position.x, 
                                          robot_pose.y, target_path.poses[1].pose.position.y);
        if(disTopoint > waypointTolerance){
            if(target_path.poses.size()>2){
                way_point.header.frame_id = global_frame;
                way_point.header.stamp = ros::Time::now();
                way_point.point.x = target_path.poses[1].pose.position.x;
                way_point.point.y = target_path.poses[1].pose.position.y;
                way_point.point.z = target_path.poses[1].pose.position.z;
                pub_waypoint.publish(way_point);
            }
            else{
                way_point.header.frame_id = global_frame;
                way_point.header.stamp = ros::Time::now();
                way_point.point.x = target_path.poses.back().pose.position.x;
                way_point.point.y = target_path.poses.back().pose.position.y;
                way_point.point.z = target_path.poses.back().pose.position.z;
                pub_waypoint.publish(way_point);
            }
        }
}

void rtrrtp::publishNewVertices(){
    if(HasNewVertices && rtRRT->root != NULL){
        // Add root node
        sea_planner::Graph new_vs;
        sea_planner::Vertex root_vertex;
        root_vertex.location = rtRRT->root->state;
        root_vertex.information_gain = 0.0;
        new_vs.vertices.push_back(root_vertex);
        if(!localFrontier.empty()){
            // Add local frontier nodes
            auto iter = localFrontier.begin();
            while(iter != localFrontier.end()){
                Node::Ptr pt = iter->first;
                float pt_info = iter->second;
                auto res = find(rtRRT->node_set.begin(), rtRRT->node_set.end(), pt);
                if(res == rtRRT->node_set.end()){
                    // Frontier is outside the local bound, has removed
                    iter++;
                    continue;
                }
                while(pt != rtRRT->root && pt != NULL){
                    sea_planner::Vertex v;
                    v.location = pt->state;
                    v.information_gain = pt_info;
                    new_vs.vertices.insert(new_vs.vertices.begin(), v);
                    pt = pt->parent;
                    // only frontier has information gian
                    pt_info = 0.0;
                }
                iter++;
            }
        }
        new_vs.header.frame_id = global_frame;
        new_vs.header.stamp = ros::Time::now();
        pub_graph_vertices.publish(new_vs);
        HasNewVertices = false;
    }
}

void rtrrtp::selectNavType(){
    // Find near nodes of final goal
    float totalDistance = 0;
    float totalGradient = 0;
    float min_cost = inf;
    rtrrt_ns::Node::Ptr best_node = NULL;
    Nodes near_nodes = rtRRT->getNeiborNodes(final_goal.pose.position.x, final_goal.pose.position.y, 0.0, findNearNodeThre);
    if(near_nodes.empty())
    {
        useSubGoal = true;
        useFinalGoal = false;
        return;
    }
    else{
        auto iter = near_nodes.begin();
        while(iter != near_nodes.end()){
            rtrrt_ns::Node::Ptr node = *iter;
            // Except OBSTACLE nodes
            if(node->nodeState == NodeState::OBSTACLE){
                iter++;
                continue;
            }
            totalDistance += node->disToRoot;
            totalGradient += node->gradientToRoot;
            iter++;
        }
        iter = near_nodes.begin();
        // Iterate all neibor nodes
        while(iter != near_nodes.end()){
            useFinalGoal = true;
            useSubGoal = false;

            rtrrt_ns::Node::Ptr near = *iter;
            // Except OBSTACLE nodes
            if(near->nodeState == NodeState::OBSTACLE){
                iter++;
                continue;
            }
            float distance_heuristic = util::distance(near->state.x, final_goal.pose.position.x, near->state.y, final_goal.pose.position.y);
            float costVal = costFunc(near->disToRoot, distance_heuristic, near->gradientToRoot, totalDistance, totalGradient);
            // std::cout << "costVal: " << costVal << std::endl;
            if(costVal < min_cost){
                min_cost = costVal;
                best_node = near;
            }
            iter++;		
        }
        // kd_res_free(near_nodes);
        // Drive to final goal surround
        rtRRT->current_target_node = best_node;
    }
}

void rtrrtp::validateFrontier(){
    // Global frontier is on the graph
    // Iterate all global frontier
    std::unordered_map<rtrrt_ns::Node::Ptr, float> globalFrontier_copy;
    mtx.lock();
    for(const auto& gp : globalFrontier){
        bool collisionFLAG=false, leafFLAG=false, infoFLAG=false;
        // Check collision
        rtrrt_ns::Node::Ptr nearest_obstacle = rtRRT->getObstacleNode(gp.first->state.x, gp.first->state.y);
        float dis = util::distance(gp.first->state.x, nearest_obstacle->state.x, gp.first->state.y, nearest_obstacle->state.y);
        if(dis <= inflationRadius){
            collisionFLAG = false;
        }
        else collisionFLAG=true;
        // Only update frontiers within the local map
        Position frontier_pos{gp.first->state.x, gp.first->state.y};
        if(!rtRRT->full_grid_map.isInside(frontier_pos)){
            globalFrontier_copy.insert(gp);
            continue;
        }
        // If frontier node is not leaf, remove it
        if(gp.first->nodeState != NodeState::LEAF){
            leafFLAG = false;
        }
        else leafFLAG=true;
        // Check frontier information gain
        Nodes near_nodes = rtRRT->getNeiborNodes(gp.first->state.x, gp.first->state.y, 0.0, exploredAreaRadius);
        int neibor_nodes_size = near_nodes.size();
        
        // For avoiding explored area
        float information_gain;
        if(neibor_nodes_size < exploredAreaThre){
            Position leaf_posi(gp.first->state.x, gp.first->state.y);
            Index leaf_index;
            Length leng(frontierRadius, frontierRadius);
            bool isSuccessful;
            auto sub_map = rtRRT->full_grid_map.getSubmap(leaf_posi, leng, leaf_index ,isSuccessful);
            Size submap_size = sub_map.getSize();
            // No enough information gain will be erase
            if(submap_size[0] == submap_size[1]){
                infoFLAG = false;
            }
            else{
                information_gain = util::getInfoGain(gp.first->state.x, gp.first->state.y, rtRRT->full_grid_map, "elevation");
                if(information_gain < informationThre){
                    infoFLAG = false;
                }
                else infoFLAG = true;
            }
        }
        else{
            infoFLAG=false;
        }
        if(collisionFLAG && leafFLAG && infoFLAG){
            globalFrontier_copy.insert(make_pair(gp.first, information_gain));
        }
    }
    globalFrontier = std::move(globalFrontier_copy);
    mtx.unlock();
    
    // auto node=globalFrontier.begin();
    // mtx.lock();
    // while(node!=globalFrontier.end() && ros::ok()){
    //     // Check collision
    //     rtrrt_ns::Node::Ptr nearest_obstacle = rtRRT->getObstacleNode((*node).first->state.x, (*node).first->state.y);
    //     float dis = util::distance((*node).first->state.x, nearest_obstacle->state.x, (*node).first->state.y, nearest_obstacle->state.y);
    //     if(dis <= inflationRadius){
    //         node = globalFrontier.erase(node);
    //         continue;
    //     }
    //     // Only update frontiers within the local map
    //     Position frontier_pos{(*node).first->state.x, (*node).first->state.y};
    //     if(!rtRRT->full_grid_map.isInside(frontier_pos)){
    //         node++;
    //         continue;
    //     }
    //     // If frontier node is not leaf, remove it
    //     if((*node).first->nodeState != NodeState::LEAF){
    //         node = globalFrontier.erase(node); // For erase method will return next iterator after resleased
    //         continue;
    //     }

    //     // Check frontier information gain
    //     Nodes near_nodes = rtRRT->getNeiborNodes((*node).first->state.x, (*node).first->state.y, 0.0, exploredAreaRadius);
    //     int neibor_nodes_size = near_nodes.size();
        
    //     // For avoiding explored area
    //     if(neibor_nodes_size < exploredAreaThre){
    //         Position leaf_posi((*node).first->state.x, (*node).first->state.y);
    //         Index leaf_index;
    //         Length leng(frontierRadius, frontierRadius);
    //         bool isSuccessful;
    //         auto sub_map = rtRRT->full_grid_map.getSubmap(leaf_posi, leng, leaf_index ,isSuccessful);
    //         Size submap_size = sub_map.getSize();
    //         // No enough information gain will be erase
    //         if(submap_size[0] == submap_size[1]){
    //             node = globalFrontier.erase(node);
    //             continue;      
    //         }
    //         else{
    //             float information_gain = util::getInfoGain((*node).first->state.x, (*node).first->state.y, rtRRT->full_grid_map, "elevation");
    //             node->second = information_gain;
    //             if(information_gain < informationThre){
    //                 node = globalFrontier.erase(node);
    //                 continue;
    //             }
    //         }
    //     }
    //     else{
    //         node = globalFrontier.erase(node);
    //         continue;
    //     }
    //     node++;
    // }
    // mtx.unlock();
}

void rtrrtp::visualizationCB(){
	if(pub_selected_branch.getNumSubscribers()==0){
		return; // if all topics of visualizing tree item have no subscriber, then don't publish
	}
    visualization_msgs::MarkerArray selectedBranch_marker;
    visualization_msgs::Marker selectedBranch_edge_marker;
    visualization_msgs::Marker selectedBranch_node_marker;
    visualization_msgs::Marker ref_line_marker;
    visualization_msgs::Marker goal_marker;
    visualization_msgs::Marker waypoint_marker;
    visualization_msgs::Marker target_node_marker;
    visualization_msgs::Marker global_frontier_marker;

    // Edge Marker
	selectedBranch_edge_marker.header.frame_id = global_frame;
	selectedBranch_edge_marker.header.stamp = ros::Time::now();
	selectedBranch_edge_marker.type = selectedBranch_edge_marker.LINE_LIST;
	selectedBranch_edge_marker.action = selectedBranch_edge_marker.ADD;
	selectedBranch_edge_marker.lifetime = ros::Duration();
	selectedBranch_edge_marker.color.a = 1.0;
	selectedBranch_edge_marker.color.r = 0.0 / 255.0;
	selectedBranch_edge_marker.color.g = 255.0 / 255.0;
	selectedBranch_edge_marker.color.b = 255.0 / 255.0;
	selectedBranch_edge_marker.scale.x = 0.1;
	selectedBranch_edge_marker.scale.y = 0.1;
	selectedBranch_edge_marker.scale.z = 0.1;
    selectedBranch_edge_marker.id = 0;
	selectedBranch_edge_marker.pose.orientation.w = 1;
	selectedBranch_edge_marker.points.clear();
    // Node Marker
	selectedBranch_node_marker.header.frame_id = global_frame;
	selectedBranch_node_marker.header.stamp = ros::Time::now();
	selectedBranch_node_marker.type = selectedBranch_node_marker.SPHERE_LIST;
	selectedBranch_node_marker.action = selectedBranch_node_marker.ADD;
	selectedBranch_node_marker.lifetime = ros::Duration();
	selectedBranch_node_marker.color.a = 1.0;
	selectedBranch_node_marker.color.r = 255.0 / 255.0;
	selectedBranch_node_marker.color.g = 255.0 / 255.0;
	selectedBranch_node_marker.color.b = 0.0 / 255.0;
	selectedBranch_node_marker.scale.x = 0.2;
	selectedBranch_node_marker.scale.y = 0.2;
	selectedBranch_node_marker.scale.z = 0.2;
    selectedBranch_node_marker.id = 1;
	selectedBranch_node_marker.pose.orientation.w = 1;
	selectedBranch_node_marker.points.clear();
    // Reference Line
	ref_line_marker.header.frame_id = global_frame;
	ref_line_marker.header.stamp = ros::Time::now();
	ref_line_marker.type = ref_line_marker.LINE_LIST;
	ref_line_marker.action = ref_line_marker.ADD;
	ref_line_marker.lifetime = ros::Duration();
	ref_line_marker.color.a = 1.0;
	ref_line_marker.color.r = 255.0 / 255.0;
	ref_line_marker.color.g = 0.0 / 255.0;
	ref_line_marker.color.b = 0.0 / 255.0;
	ref_line_marker.scale.x = 0.1;
	ref_line_marker.scale.y = 0.1;
	ref_line_marker.scale.z = 0.1;
    ref_line_marker.id = 0;
	ref_line_marker.pose.orientation.w = 1;
	ref_line_marker.points.clear();

    // Goal Marker
	goal_marker.header.frame_id = global_frame;
	goal_marker.header.stamp = ros::Time::now();
	goal_marker.type = goal_marker.SPHERE;
	goal_marker.action = goal_marker.ADD;
	goal_marker.lifetime = ros::Duration();
	goal_marker.color.a = 1.0;
	goal_marker.color.r = 255.0 / 255.0;
	goal_marker.color.g = 0.0 / 255.0;
	goal_marker.color.b = 237.0 / 255.0;
	goal_marker.scale.x = 0.5;
	goal_marker.scale.y = 0.5;
	goal_marker.scale.z = 0.5;
	goal_marker.pose.orientation.w = 1;
    // Waypoint Marker
	waypoint_marker.header.frame_id = global_frame;
	waypoint_marker.header.stamp = ros::Time::now();
	waypoint_marker.type = waypoint_marker.SPHERE;
	waypoint_marker.action = waypoint_marker.ADD;
	waypoint_marker.lifetime = ros::Duration();
	waypoint_marker.color.a = 1.0;
	waypoint_marker.color.r = 43.0 / 255.0;
	waypoint_marker.color.g = 57.0 / 255.0;
	waypoint_marker.color.b = 255.0 / 255.0;
	waypoint_marker.scale.x = 0.3;
	waypoint_marker.scale.y = 0.3;
	waypoint_marker.scale.z = 0.3;
	waypoint_marker.pose.orientation.w = 1;

    // Target Marker
    if(target_node != NULL){
        target_node_marker.header.frame_id = global_frame;
        target_node_marker.header.stamp = ros::Time::now();
        target_node_marker.type = target_node_marker.SPHERE;
        target_node_marker.action = target_node_marker.ADD;
        target_node_marker.lifetime = ros::Duration();
        target_node_marker.color.a = 1.0;
        target_node_marker.color.r = 0.0 / 255.0;
        target_node_marker.color.g = 255.0 / 255.0;
        target_node_marker.color.b = 0.0 / 255.0;
        target_node_marker.scale.x = 0.5;
        target_node_marker.scale.y = 0.5;
        target_node_marker.scale.z = 0.5;
        target_node_marker.pose.orientation.x = 0;
        target_node_marker.pose.orientation.y = 0;
        target_node_marker.pose.orientation.z = 0;
        target_node_marker.pose.orientation.w = 1;
        target_node_marker.pose.position.x = target_node->state.x;
        target_node_marker.pose.position.y = target_node->state.y;
        target_node_marker.pose.position.z = target_node->state.z;
        pub_target_marker.publish(target_node_marker);
    }

    // Global frontier Marker
    if(!globalFrontier.empty()){
        global_frontier_marker.header.frame_id = global_frame;
        global_frontier_marker.header.stamp = ros::Time::now();
        global_frontier_marker.type = global_frontier_marker.SPHERE_LIST;
        global_frontier_marker.action = global_frontier_marker.ADD;
        global_frontier_marker.lifetime = ros::Duration();
        global_frontier_marker.color.a = 1.0;
        global_frontier_marker.color.r = 129.0 / 255.0;
        global_frontier_marker.color.g = 208.0 / 255.0;
        global_frontier_marker.color.b = 255.0 / 255.0;
        global_frontier_marker.scale.x = 0.2;
        global_frontier_marker.scale.y = 0.2;
        global_frontier_marker.scale.z = 0.2;
        global_frontier_marker.pose.orientation.w = 1;
        global_frontier_marker.points.clear();
        for(auto i=globalFrontier.begin(); i!=globalFrontier.end(); i++){
            geometry_msgs::Point single_point;
            single_point.x = i->first->state.x;
            single_point.y = i->first->state.y;
            single_point.z = i->first->state.z;
            global_frontier_marker.points.push_back(single_point);
        }
        pub_global_frontier.publish(global_frontier_marker);
    }

    // Add data information
    if(!selectedBranch_nodes.empty()){
        mtx.lock();
        for(auto iter=selectedBranch_nodes.begin(); iter!=selectedBranch_nodes.end()-1; iter++){
            geometry_msgs::Point single_point;
            single_point.x = (*iter)->state.x;
            single_point.y = (*iter)->state.y;
            single_point.z = (*iter)->state.z;
            selectedBranch_node_marker.points.push_back(single_point);
            selectedBranch_edge_marker.points.push_back(single_point);
            single_point.x = (*(iter+1))->state.x;
            single_point.y = (*(iter+1))->state.y;
            single_point.z = (*(iter+1))->state.z;        
            selectedBranch_edge_marker.points.push_back(single_point);
        }
        mtx.unlock();
        selectedBranch_marker.markers.push_back(selectedBranch_node_marker);
        selectedBranch_marker.markers.push_back(selectedBranch_edge_marker);
        pub_selected_branch.publish(selectedBranch_marker);
    }

    if(!selectedBranch_nodes.empty() && target_node!=NULL){
        float segment_length = 0.3;
        float disToGoal = util::distance(target_node->state.x, final_goal.pose.position.x, target_node->state.y, final_goal.pose.position.y);
        geometry_msgs::Point line_point;
        // It will connect directly if distance is short
        if(disToGoal <= segment_length*2){
            line_point.x = target_node->state.x;
            line_point.y = target_node->state.y;
            line_point.z = target_node->state.z;
            ref_line_marker.points.push_back(line_point);
            line_point = final_goal.pose.position;
            ref_line_marker.points.push_back(line_point); 
            pub_ref_line.publish(ref_line_marker);    
        }
        else{   // Connect using virtual line form
            float Theta = atan2(final_goal.pose.position.y-target_node->state.y, final_goal.pose.position.x-target_node->state.x);
            float segment_deltX = segment_length*cos(Theta);
            float segment_deltY = segment_length*sin(Theta);
            float PointX = target_node->state.x;
            float PointY = target_node->state.y;
            float PointZ = target_node->state.z;
            float last_disToEnd = inf;
            float curr_disToEnd = util::distance(PointX, final_goal.pose.position.x, PointY, final_goal.pose.position.y);
            while(curr_disToEnd <= last_disToEnd){
                last_disToEnd = curr_disToEnd;
                line_point.x = PointX;
                line_point.y = PointY;
                line_point.z = PointZ;
                ref_line_marker.points.push_back(line_point); 
                PointX += segment_deltX;
                PointY += segment_deltY;
                curr_disToEnd = util::distance(PointX, final_goal.pose.position.x, PointY, final_goal.pose.position.y);
            }
            if(ref_line_marker.points.size()%2 != 0){
                ref_line_marker.points.push_back(*(ref_line_marker.points.end()-1));
            }
            pub_ref_line.publish(ref_line_marker);
        }
        
    }

    goal_marker.pose = final_goal.pose;
    pub_goal_marker.publish(goal_marker);
    waypoint_marker.pose.position = way_point.point;
    pub_waypoint_marker.publish(waypoint_marker);
}

void rtrrtp::updateRobotPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &new_pose){
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
    
    // visuliazation
    auto cur_time = ros::Time::now();
    if((cur_time-last_time).toSec()>0.5){
        visualizationCB();
    }
}

void rtrrtp::goalHandler(const geometry_msgs::PoseStampedConstPtr &goal_point){
	if(!updatedPose)
	{
		ROS_WARN("Cannot get the robot pose.");
		return;
	}
	final_goal.pose.position.x = goal_point->pose.position.x;
	final_goal.pose.position.y = goal_point->pose.position.y;
	final_goal.pose.position.z = goal_point->pose.position.z+0.5;
	final_goal.pose.orientation = goal_point->pose.orientation;
    // rtRRT->updateGoal(final_goal);
	rtRRT->goal.position.x = goal_point->pose.position.x;
	rtRRT->goal.position.y = goal_point->pose.position.y;
	rtRRT->goal.position.z = goal_point->pose.position.z;
	rtRRT->goal.orientation = goal_point->pose.orientation;

    printf("\033[2J\033[1;1H"); // Clear all print msg
	std::cout << bold << magenta
              << "Received Goal: " << rtRRT->goal.position.x << ", " << rtRRT->goal.position.y 
              << reset << std::endl;
    if(cur_state == PlannerState::ACTIVE){
        mtx.lock();
        target_node = NULL;
        useSubGoal = false;
        useFinalGoal = false;
        cur_state = PlannerState::WAITING; // Set flag to stop
        mtx.unlock();
        stayCurrentPose(); // Stop locomotion
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for ending executeCycle
    }

    rtRRT->newGoalReceived = true;
	newGoalReceived = true;

    std::cout << green
              << "Start path planning"
              << reset << std::endl;
    cur_state = PlannerState::ACTIVE;   // Active planner
}


