#pragma once
#include "utils/spline.h"
#include "data_type/planning_data_type.h"
#include "data_type/poly_candidate.h"

// planning module
#include "module/start_selector.h"
#include "module/candidate_builder.h"

class poly_planner{
    public:
	poly_planner() {}
	~poly_planner() {}
    public:
	void Run(Map* map,		    // reference map
		double path_dt,		    // time interval of path's nodes
		std::vector<double> path_x, // x position of path's nodes
		std::vector<double> path_y, // y position of path's nodes
		std::vector<double> ego_pose, // ego pose (x,y,yaw,spd,acc_
		double lookahead_time );    // seconds
    private:
	sn_state SelStartState(Map* map, 
		double path_dt,
		std::vector<double> path_x, 
		std::vector<double> path_y,
		std::vector<double> ego_pose,
		double lookahead_time);
	void BuildCandidate(
		sn_state start_state);
	trajectory SelOptTrajectory();

    private:
	start_selector start_selector_;
	candidate_builder candidate_builder_;
};
