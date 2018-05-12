#include "optimal_selector.h"

#include <iostream>
#include <algorithm>

optimal_selector::optimal_selector(){

}

optimal_selector::~optimal_selector(){

}

bool optimal_selector::Optimization( candidate_p_set* s_candidate,
			 candidate_p_set* n_candidate,
			 double s_weight, double n_weight,
			 planning_object::object_manager* objects,
			 trajectory& opt_trajectory	){

    
    // trajectory candidate from s and n candidate
    trajectory_set candidates = SN2Trajectory( s_candidate, n_candidate, s_weight, n_weight );
    
    // optimal trajectory selection with collision check
    for( int i=0; i<10; i++){
	print( &candidates[i] );
    }

    bool is_collision = true;
    for( int i=0; i<candidates.size(); i++){
	std::vector<double> path_s, path_n;
	candidates[i].GetDiscretePathSN( collision_check_resol_, collision_check_time_, path_s, path_n );
	if( objects->IsCollision( collision_check_resol_, path_s, path_n, vehicle_length_, vehicle_width_ ) == false){
	    std::cout << "#### optimal ######" << std::endl;
	    print( &candidates[i] );
	    opt_trajectory = candidates[i];
	    is_collision = false;
	    //return true;
	    break;
	}
    }
    return !is_collision;
}

trajectory_set optimal_selector::SN2Trajectory(	candidate_p_set* s_candidate,
				candidate_p_set* n_candidate,
				double s_weight, double n_weight ){
    trajectory_set sn_candidates;

    // mix
    int idx_trj = 0;
    for( int is = 0 ; is<s_candidate->size() ; is++){
	for( int in = 0; in<n_candidate->size() ; in++ ){
	    if( ( (*s_candidate)[is]->GetLaneIndex() == (*n_candidate)[in]->GetLaneIndex())
	     || ( (*s_candidate)[is]->GetLaneIndex() == -1 ) ){
		trajectory sn_candidate( idx_trj, (*s_candidate)[is], (*n_candidate)[in], s_weight, n_weight );

		sn_candidates.push_back( sn_candidate );
		idx_trj++;
	    }
	}
    }

    // sort by cost
    std::sort( sn_candidates.begin(), sn_candidates.end() );

    return sn_candidates;
}

void optimal_selector::print( trajectory* trj ){
    candidate* candi = trj->GetpTrajectoryS();
    double T = candi->GetT();
    state s = candi->GetState( T );
    std::cout << "candidate(T/s/ds): ";
    if( candi->GetManeuver() == FOLLOW){
	std::cout << "F: ";
    }
    else{
	std::cout << "K: ";
    }
    std::cout << T << "/" << s[0] << "/" << s[1];
    std:: cout << ": " << trj->GetCost() << std::endl;

}

