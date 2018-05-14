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
		std::vector<double> ego_pose, // ego pose (x,y,yaw,spd,acc_
		double lookahead_time,	      // seconds
	        double desired_spd,	      // desired speed (m/s)
		std::vector<std::vector<double>> objects	  // desired speed (m/s)

		);      
    private:
	sn_state SelStartState(Map* map, 
		double path_dt,
		std::vector<cartesian_state> path,
		std::vector<double> ego_pose,
		double lookahead_time);

	void BuildCandidate(Map* map,
		sn_state start_state,
		std::vector<poly_candidate>& s_candidate,
		std::vector<poly_candidate>& n_candidate,
		candidate_weight weigh,
		double desired_spd,
		planning_object::object_manager* object_list);

	bool SelOptTrajectory(
		Map* map,
		double lookahead_time,
		candidate_p_set* s_candidates,
		candidate_p_set* n_candidates,
		double s_weight, double n_weight,
		planning_object::object_manager* objects,
		trajectory& opt_trajectory);

	void UpdateTrajectory( Map* map, trajectory trj, std::vector<double> ego_sn, std::vector<double> start_sn, double time_resol );
	int FindPathNodeSN( std::vector<double> searching_sn );

    private:
	start_selector	    start_selector_;
	candidate_builder   candidate_builder_;
	optimal_selector    optimal_selector_;

    private:
	std::vector<state> path_s_;
	std::vector<state> path_n_;
	std::vector<double> path_x_;
	std::vector<double> path_y_;
	std::vector<cartesian_state> path_;

    private:
	// weight
	double s_weight_ = 1.0;
	double n_weight_ = 5.0;

	double weight_s_jerk_ = 10.0;
	double weight_s_time_ = 10.0;
	double weight_s_stop_ = 1.0;
	double weight_s_follow_ = 1.0;
	double weight_s_keep_spd_ =50.0;

	double weight_n_jerk_ = 1.0;
	double weight_n_time_ = 1.0;
	double weight_n_terminal_ = 1.0;

    public:
	std::vector<double> GetTrjX();	
	std::vector<double> GetTrjY();
};
