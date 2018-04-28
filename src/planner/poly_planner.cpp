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
    // Test

    state start = {0, 0, 0};
    state end = {1, 0, 0};
    poly_candidate variant( start, end, 10.0);

    std::cout << "max acc: " << variant.GetMaxAcc() << std::endl;
    std::cout << "min acc: " << variant.GetMinAcc() << std::endl;
    std::cout << "max jerk: " << variant.GetMaxJerk() << std::endl;
    std::cout << "integral jerk: " << variant.GetIntegJerk() << std::endl;
    std::cout << "continuation: " << variant.GetT() << std::endl;
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

void poly_planner::BuildCandidate(){

}

trajectory poly_planner::SelOptTrajectory(){
}
