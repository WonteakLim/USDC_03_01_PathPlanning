#include "optimal_selector.h"

#include <iostream>
#include <algorithm>

optimal_selector::optimal_selector(){
    bool is_config = config_parser_.Init( "../src/planner/config/config_selection.ini" );
}

optimal_selector::~optimal_selector(){

}

bool optimal_selector::Optimization( 
			Map* map,
			double lookahead_time,
			candidate_p_set* s_candidate,
			 candidate_p_set* n_candidate,
			 double s_weight, double n_weight, double lane_weight,
			 planning_object::object_manager* objects,
			 int desired_lane,
			 trajectory& opt_trajectory	){

    ProcessINI();
    
    // trajectory candidate from s and n candidate
    trajectory_set candidates = SN2Trajectory( s_candidate, n_candidate, desired_lane, s_weight, n_weight, lane_weight );
    
    // optimal trajectory selection with collision check
    for( int i=0; i<10; i++){
	print( &candidates[i] );
    }

    bool is_valid = false;
    for( int i=0; i<candidates.size(); i++){
	std::vector<double> path_s, path_n;
	candidates[i].GetDiscretePathSN( collision_check_resol_, collision_check_time_, path_s, path_n );

	bool is_collision = objects->IsCollision( lookahead_time, collision_check_resol_, path_s, path_n, vehicle_length_, vehicle_width_ );
	bool is_valid_k = IsValidCurvature( map, &candidates[i], curvature_check_t_resol_, curvature_check_T_ );

	if( (is_collision == false)
	   && (is_valid_k == true ) ){
	    std::cout << "#### optimal ######" << std::endl;
	    print( &candidates[i] );
	    opt_trajectory = candidates[i];
	    is_valid = true;
	    break;
	}
    }
    return is_valid;
}

trajectory_set optimal_selector::SN2Trajectory(	candidate_p_set* s_candidate,
				candidate_p_set* n_candidate,
				int desired_lane,
				double s_weight, double n_weight, double lane_weight ){
    trajectory_set sn_candidates;

    // mix
    int idx_trj = 0;
    for( int is = 0 ; is<s_candidate->size() ; is++){
	for( int in = 0; in<n_candidate->size() ; in++ ){
	    if( ( (*s_candidate)[is]->GetLaneIndex() == (*n_candidate)[in]->GetLaneIndex())
	     || ( (*s_candidate)[is]->GetLaneIndex() == -1 ) ){
		trajectory sn_candidate( idx_trj, (*s_candidate)[is], (*n_candidate)[in], desired_lane, s_weight, n_weight, lane_weight );

		sn_candidates.push_back( sn_candidate );
		idx_trj++;
	    }
	}
    }

    // sort by cost
    std::sort( sn_candidates.begin(), sn_candidates.end() );

    return sn_candidates;
}

bool optimal_selector::IsValidCurvature( Map* map, trajectory* p_trajectory, double t_resol, double T ){
    int node_num = (int)(T/t_resol);
    for( int i=0; i<node_num; i++){
	double t = i * t_resol;
	state s = p_trajectory->GetpTrajectoryS()->GetState(t);
	state n = p_trajectory->GetpTrajectoryN()->GetState(t);
	std::vector<double> state = map->ToCartesianAllT( {s[0], s[1], s[2], n[0], n[1], n[2]} );
	double k = state[3];
	if( abs(k) > curvature_limit_ ){
	    return false;
	}
    }
    return true;
}

void optimal_selector::print( trajectory* trj ){
    candidate* candi = trj->GetpTrajectoryS();
    double T = candi->GetT();
    state s = candi->GetState( T );
    std::cout << "candidate(lane/T/s/ds): " << trj->GetLaneIdx();

    if( candi->GetManeuver() == FOLLOW){
	std::cout << "F: ";
    }
    else{
	std::cout << "K: ";
    }
    std::cout << T << "/" << s[0] << "/" << s[1];
    std:: cout << ": " << trj->GetCost() << std::endl;

}

void optimal_selector::ProcessINI(){
    if( config_parser_.IsFileUpdated() == true ){
	std::cout << "selection config is updated" << std::endl;
	config_parser_.ParseConfig("selection", "collision_check_resol", collision_check_resol_ );
	config_parser_.ParseConfig("selection", "collision_check_time", collision_check_time_ );
	config_parser_.ParseConfig("selection", "vehicle_length", vehicle_length_ );
	config_parser_.ParseConfig("selection", "vehicle_width", vehicle_width_ );
	config_parser_.ParseConfig("selection", "curvature_check_t_resol", curvature_check_t_resol_ );
	config_parser_.ParseConfig("selection", "curvature_check_T", curvature_check_T_ );
	config_parser_.ParseConfig("selection", "curvature_limit", curvature_limit_ );
    }
}
