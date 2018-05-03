#include <iostream>
#include "poly_planner.h"

#include "data_type/poly_candidate.h"

poly_planner::poly_planner(){
    path_x_.clear();
    path_y_.clear();
    path_s_.clear();
    path_n_.clear();
}

void poly_planner::Run( Map* map,
			double path_dt,
			std::vector<double> path_x,
			std::vector<double> path_y,
			std::vector<double> ego_pose,
			double lookahead_time,
			double desired_spd,
			std::vector<std::vector<double>> objects	){
    planning_object::object_manager object_list( *map, objects );

    // ================
    // path planning
    // ================

    // start poisition
    int start_idx = -1;
    sn_state start_sn = SelStartState(map, path_dt,
					path_x_, path_y_,
					ego_pose, lookahead_time);
    start_sn = FindPathNodeSN( start_sn, start_idx );

    // build candidates
    poly_candidate_set s_candidates_poly, n_candidates_poly;
    BuildCandidate(map, start_sn, s_candidates_poly, n_candidates_poly, desired_spd, &object_list);

    candidate_p_set s_candidates, n_candidates;
    for( int i=0; i<s_candidates_poly.size(); i++){
	s_candidates.push_back( (candidate*) &s_candidates_poly[i] );
    }
    for( int i=0; i<n_candidates_poly.size(); i++){
	n_candidates.push_back( (candidate*) &n_candidates_poly[i] );
    }

    // select optimal trajectory
    trajectory opt_trajectory = SelOptTrajectory( &s_candidates, &n_candidates, desired_spd, &object_list );

    UpdateTrajectory( map, opt_trajectory, start_idx, path_dt );
}

sn_state poly_planner::SelStartState(Map* map,
	double path_dt,
	std::vector<double> path_x,
	std::vector<double> path_y,
	std::vector<double> ego_pose,
	double lookahead_time){
    // return start node
    return start_selector_.SelectStartNode( map, 
	    path_dt, 
	    path_x,
	    path_y,
	    ego_pose,
	    lookahead_time);
}

void poly_planner::BuildCandidate(Map* map,
	sn_state start_state, 
	poly_candidate_set& s_candidate, poly_candidate_set& n_candidate, double desired_spd,
	planning_object::object_manager* object_list){
    s_candidate.clear();
    n_candidate.clear();
    
    candidate_builder_.BuildVariants( map, start_state.s, start_state.n,
		   s_candidate, n_candidate,
		   desired_spd, object_list ); 
}

trajectory poly_planner::SelOptTrajectory(
	candidate_p_set* s_candidates, candidate_p_set* n_candidates,
	double desired_spd,
	planning_object::object_manager* objects){
    trajectory_weight weight;
    weight.s_comfort = 1.0;
    weight.n_comfort = 1.0;
    weight.s_desired_spd = 5.0;


    return optimal_selector_.Optimization( s_candidates, n_candidates, desired_spd, objects, weight);
}

void poly_planner::UpdateTrajectory( Map* map, trajectory trj, int start_idx, double time_resol ){
    std::vector<double> path_x, path_y;
    std::vector<state> path_s, path_n;

    if( start_idx != -1 ){
	for( int i=0; i<start_idx; i++){
	    path_x.push_back( path_x_[i] );
	    path_y.push_back( path_y_[i] );
	    path_s.push_back( path_s_[i] );
	    path_n.push_back( path_n_[i] );
	}
    }
    double time_horizon = trj.GetTimeHorizon();
    int path_size = (time_horizon) / (time_resol);
    for( int i=0; i<path_size ; i++ ){
	std::vector<state> sn_state = trj.GetNode( (double)i * time_resol );
	std::vector<double> xy = map->ToCartesian( sn_state[0][0], sn_state[1][0] );
	path_s.push_back( sn_state[0] );
	path_n.push_back( sn_state[1] );
	path_x.push_back( xy[0] );
	path_y.push_back( xy[1] );
    }

    // update member variables
    path_x_ = path_x;
    path_y_ = path_y;
    path_s_ = path_s;
    path_n_ = path_n;
}

sn_state poly_planner::FindPathNodeSN( sn_state searching_sn, int& idx ){
    if( path_s_.size() == 0 ) return searching_sn;
    double min_dist = 1000;
    double min_idx = -1;
    for( int i=0 ;i < path_s_.size() ; i++ ){
	state s = path_s_[i];
	state n = path_n_[i];

	double ds = s[0] - searching_sn.s[0];
	double dn = n[0] - searching_sn.n[0];
	double dist = sqrt( ds*ds + dn*dn );

	if( dist < min_dist ){
	    min_dist = dist;
	    min_idx = i;
	}	
    }

    sn_state closed_node_sn;
   closed_node_sn.s = path_s_[min_idx];
   closed_node_sn.n = path_n_[min_idx];

   idx = min_idx;
   return closed_node_sn;
}
