#pragma once
#include "utils/spline.h"
#include "data_type/planning_data_type.h"
#include "data_type/poly_candidate.h"

// planning module
#include "module/start_selector.h"
#include "module/candidate_builder.h"
#include "module/optimal_selector.h"
#include "module/object_manager.h"

class poly_planner{
    public:
	poly_planner();
	~poly_planner() {}
    public:
	void Run(Map* map,		    // reference map
		double path_dt,		    // time interval of path's nodes
		std::vector<double> path_x, // x position of path's nodes
		std::vector<double> path_y, // y position of path's nodes
		std::vector<double> ego_pose, // ego pose (x,y,yaw,spd,acc_
		double lookahead_time,	      // seconds
	        double desired_spd,	      // desired speed (m/s)
		std::vector<std::vector<double>> objects	  // desired speed (m/s)

		);      
    private:
	sn_state SelStartState(Map* map, 
		double path_dt,
		std::vector<double> path_x, 
		std::vector<double> path_y,
		std::vector<double> ego_pose,
		double lookahead_time);

	void BuildCandidate(Map* map,
		sn_state start_state,
		std::vector<poly_candidate>& s_candidate,
		std::vector<poly_candidate>& n_candidate,
		double desired_spd,
		planning_object::object_manager* object_list);

	trajectory SelOptTrajectory(
		candidate_p_set* s_candidates,
		candidate_p_set* n_candidates,
		double desired_spd,
		planning_object::object_manager* objects);

	void UpdateTrajectory( Map* map, trajectory trj, int start_idx, double time_resol );
	sn_state FindPathNodeSN( sn_state searching_sn, int& idx );

    private:
	start_selector	    start_selector_;
	candidate_builder   candidate_builder_;
	optimal_selector    optimal_selector_;

    private:
	std::vector<state> path_s_;
	std::vector<state> path_n_;
	std::vector<double> path_x_;
	std::vector<double> path_y_;

    public:
	inline std::vector<double> GetTrjX() { return path_x_; }
	inline std::vector<double> GetTrjY() { return path_y_; }
};
