#include <iostream>
#include "poly_planner.h"

#include "data_type/poly_candidate.h"

void poly_planner::Run( Map* map,
			double path_dt,
			std::vector<double> path_x,
			std::vector<double> path_y,
			std::vector<double> ego_pose,
			double lookahead_time	){
    sn_state start_sn = SelStartState(map, path_dt,
					path_x, path_y,
					ego_pose, lookahead_time);

    BuildCandidate(start_sn);


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

void poly_planner::BuildCandidate(sn_state start_state){
    candidate_set s_candidates, n_candidates;
    candidate_builder_.BuildVariants( start_state.s, start_state.n,
		   s_candidates, n_candidates ); 
}

trajectory poly_planner::SelOptTrajectory(){
}
