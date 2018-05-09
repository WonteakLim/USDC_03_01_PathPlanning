#include "optimal_selector.h"

#include <iostream>
#include <algorithm>

optimal_selector::optimal_selector(){

}

optimal_selector::~optimal_selector(){

}

bool optimal_selector::Optimization( candidate_p_set* s_candidate,
			 candidate_p_set* n_candidate,
			 double desired_spd,
			 planning_object::object_manager* objects,
			 trajectory_weight weight,
			 trajectory& opt_trajectory	){

    
    // trajectory candidate from s and n candidate
    trajectory_set candidates = SN2Trajectory( s_candidate, n_candidate, desired_spd, weight );
    
    // optimal trajectory selection with collision check
    double col_dt = 0.5;
    bool is_collision = true;
    for( int i=0; i<candidates.size(); i++){
	double t = candidates[i].GetS_T();
	sn_state sn = candidates[i].GetNode( t );
	std::cout << "canddates(T/D/S): " << t  << ", " << sn.s[0] << ", " << sn.s[1] << std::endl;
	std::vector<double> path_s, path_n;
	candidates[i].GetDiscretePathSN( col_dt, 3, path_s, path_n );
	if( objects->IsCollision( col_dt, path_s, path_n, 4.0, 2.0 ) == false){
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
				double desired_spd,
				trajectory_weight weight ){
    trajectory_set sn_candidates;

    // mix
    int idx_trj = 0;
    for( int is = 0 ; is<s_candidate->size() ; is++){
	for( int in = 0; in<n_candidate->size() ; in++ ){
	    if( ( (*s_candidate)[is]->GetLaneIndex() == (*n_candidate)[in]->GetLaneIndex())
	     || ( (*s_candidate)[is]->GetLaneIndex() == -1 ) ){
		trajectory sn_candidate( idx_trj, (*s_candidate)[is], (*n_candidate)[in], desired_spd, weight );

		sn_candidates.push_back( sn_candidate );
		idx_trj++;
	    }
	}
    }

    // sort by cost
    std::sort( sn_candidates.begin(), sn_candidates.end() );

    return sn_candidates;
}


