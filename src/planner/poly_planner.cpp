#include <iostream>
#include "poly_planner.h"

#include "data_type/poly_candidate.h"

poly_planner::poly_planner(){
    path_x_.clear();
    path_y_.clear();
    path_s_.clear();
    path_n_.clear();
    path_.clear();

    bool is_config = config_parser.Init( "../src/planner/config/config_planner.ini");
}

void poly_planner::Run( Map* map,
			double path_dt,
			std::vector<double> ego_pose,
			double lookahead_time,
			double desired_spd,
			std::vector<std::vector<double>> objects	){
    ProcessINI();

    planning_object::object_manager object_list( *map, objects );

    // ================
    // path planning
    // ================
    // start poisition
    std::cout << "#####################################################" << std::endl;
    std::cout << ">> start position" << std::endl;
    sn_state start_sn = SelStartState(map, path_dt, path_,
					ego_pose, lookahead_time);
    std::vector<double> xy = map->ToCartesian( start_sn.s[0], start_sn.n[0] );
    std::cout << "start node (s/ds/dds): " << start_sn.s[0] << ", " << start_sn.s[1] << ", " << start_sn.s[2] << std::endl;

    // build candidates
    std::cout << ">>>> build candidates" << std::endl;
    candidate_weight weight = { 
	weight_s_jerk_, weight_s_time_, weight_s_stop_, weight_s_follow_, weight_s_keep_spd_, weight_s_violate_,
	weight_n_jerk_, weight_n_time_, weight_n_terminal_};
    poly_candidate_set s_candidates_poly, n_candidates_poly;
    BuildCandidate(map, start_sn, s_candidates_poly, n_candidates_poly, weight, desired_spd, &object_list);
    
    candidate_p_set s_candidates, n_candidates;
    for( int i=0; i<s_candidates_poly.size(); i++){
	s_candidates.push_back( (candidate*) &s_candidates_poly[i] );
    }
    for( int i=0; i<n_candidates_poly.size(); i++){
	n_candidates.push_back( (candidate*) &n_candidates_poly[i] );
    }

    // select optimal trajectory
    std::cout << ">>>>>> optimal selection" << std::endl;
    trajectory opt_trajectory;
    if( SelOptTrajectory( map, lookahead_time, &s_candidates, &n_candidates, s_weight_, n_weight_, lane_weight_, &object_list, desired_lane_, opt_trajectory ) == true){
	UpdateTrajectory( map, opt_trajectory, {ego_pose[5], ego_pose[6]}, {start_sn.s[0], start_sn.n[0]}, path_dt );
    }

    int size = path_x_.size();
    if( size > 50 ) size = 50;
    for( int i=0 ;i<size ; i++){
	double dx = path_x_[i+1] - path_x_[i];
	double dy = path_y_[i+1] - path_y_[i];
	double spd = sqrt( dx*dx + dy*dy ) / 0.02;
    }
}

sn_state poly_planner::SelStartState(Map* map,
	double path_dt,
	std::vector<cartesian_state> path,
	std::vector<double> ego_pose,
	double lookahead_time){
    // return start node
    return start_selector_.SelectStartNode( map, 
	    path_dt, 
	    path,
	    ego_pose,
	    lookahead_time);
}

void poly_planner::BuildCandidate(Map* map,
	sn_state start_state, 
	poly_candidate_set& s_candidate, poly_candidate_set& n_candidate, 
	candidate_weight weight, double desired_spd,
	planning_object::object_manager* object_list){
    s_candidate.clear();
    n_candidate.clear();
    
    candidate_builder_.BuildVariants( map, start_state.s, start_state.n,
		    s_candidate, n_candidate, weight,
		    desired_spd, object_list ); 
}

bool poly_planner::SelOptTrajectory(
	Map* map,
	double lookahead_time,
	candidate_p_set* s_candidates, candidate_p_set* n_candidates,
	double s_weight, double n_weight, double lane_weight,
	planning_object::object_manager* objects,
	int desired_lane,
	trajectory& opt_trajectory){

    return optimal_selector_.Optimization( map, lookahead_time, s_candidates, n_candidates,
	    s_weight, n_weight, lane_weight,
	    objects,
	    desired_lane,
	    opt_trajectory);
}

void poly_planner::UpdateTrajectory( Map* map, trajectory trj, std::vector<double> ego_sn, std::vector<double> start_sn, double time_resol ){
    std::cout << "Final trajectory: ";
    switch( trj.GetpTrajectoryS()->GetManeuver()){
	case STOP:
	    std::cout << "STOP";
	    break;
	case KEEPING:
	    std::cout << "KEEPING";
	    break;
	case FOLLOW:
	    std::cout << "FOLLOWING";
	    break;
	default:
	    break;
    }
    std::cout << std::endl;


    std::vector<double> path_x, path_y;
    std::vector<state> path_s, path_n;
    std::vector<cartesian_state> path;

    int start_node_id = FindPathNodeSN( start_sn );
    int ego_node_id = FindPathNodeSN( ego_sn );
    if( (start_node_id != -1) 
	&& (ego_node_id != -1) 
	&& (ego_node_id <= start_node_id )){
	for( int i=ego_node_id; i<start_node_id; i++){
	    path_x.push_back( path_x_[i] );
	    path_y.push_back( path_y_[i] );
	    path_s.push_back( path_s_[i] );
	    path_n.push_back( path_n_[i] );

	    path.push_back( path_[i] );
	}
    }
    double time_horizon = trj.GetTimeHorizon();
    if( time_horizon > 3.0) time_horizon = 3.0;
    int path_size = (time_horizon) / (time_resol);
    for( int i=0; i<path_size ; i++ ){
	sn_state sn_state = trj.GetNode( (double)i * time_resol );
	std::vector<double> xy = map->ToCartesian( sn_state.s[0], sn_state.n[0] );
	path_s.push_back( sn_state.s );
	path_n.push_back( sn_state.n );
	path_x.push_back( xy[0] );
	path_y.push_back( xy[1] );

	std::vector<double> cartesian = map->ToCartesianAllT({ sn_state.s[0], sn_state.s[1], sn_state.s[2],
							       sn_state.n[0], sn_state.n[1], sn_state.n[2] } );
	cartesian_state new_node;
	new_node.x = cartesian[0];
	new_node.y = cartesian[1];
	new_node.yaw = cartesian[2];
	new_node.k = cartesian[3];
	new_node.spd = cartesian[4];
	new_node.acc = cartesian[5];
	path.push_back( new_node );
    }

    std::cout << "### UPDATE ##### : " << path_x.size() << std::endl;

    // update member variables
    path_x_ = path_x;
    path_y_ = path_y;
    path_s_ = path_s;
    path_n_ = path_n;
    path_ = path;
}

int  poly_planner::FindPathNodeSN( std::vector<double> searching_sn){
    if( path_s_.size() == 0 ) return -1;
    double min_dist = 1000;
    double min_idx = -1;
    for( int i=0 ;i < path_s_.size() ; i++ ){
	state s = path_s_[i];
	state n = path_n_[i];

	double ds = s[0] - searching_sn[0];
	double dn = n[0] - searching_sn[1];
	double dist = sqrt( ds*ds + dn*dn );

	if( dist < min_dist ){
	    min_dist = dist;
	    min_idx = i;
	}	
    }
    return min_idx;   
}

void poly_planner::ProcessINI(){
    if( config_parser.IsFileUpdated() == true ){
	config_parser.ParseConfig( "planning", "desired_lane", desired_lane_ );
	config_parser.ParseConfig( "planning", "s_weight", s_weight_ );
	config_parser.ParseConfig( "planning", "n_weight", n_weight_ );
	config_parser.ParseConfig( "planning", "lane_weight", lane_weight_ );
	config_parser.ParseConfig( "planning", "weight_s_jerk", weight_s_jerk_ );
	config_parser.ParseConfig( "planning", "weight_s_time", weight_s_time_ );
	config_parser.ParseConfig( "planning", "weight_s_stop", weight_s_stop_ );
	config_parser.ParseConfig( "planning", "weight_s_follow", weight_s_follow_ );
	config_parser.ParseConfig( "planning", "weight_s_keep_spd", weight_s_keep_spd_ );
	config_parser.ParseConfig( "planning", "weight_s_violate", weight_s_violate_ );
	config_parser.ParseConfig( "planning", "weight_n_jerk", weight_n_jerk_ );
	config_parser.ParseConfig( "planning", "weight_n_time", weight_n_time_ );
	config_parser.ParseConfig( "planning", "weight_n_terminal", weight_n_terminal_ );
    }
}

std::vector<double> poly_planner::GetTrjX(){
    std::vector<double> x;
    for( int i=0; i<path_.size() ; i++){
	x.push_back( path_[i].x );
    }
    return x;
}

std::vector<double> poly_planner::GetTrjY(){
    std::vector<double> y;
    for( int i=0; i<path_.size() ; i++){
	y.push_back( path_[i].y );
    }
    return y;
}
