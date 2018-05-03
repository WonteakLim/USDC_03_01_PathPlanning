#pragma once
#include "../data_type/planning_data_type.h"
#include "../data_type/poly.h"
#include "../data_type/poly_candidate.h"
#include "../utils/spline.h"
#include "object_manager.h"

class candidate_builder{
    public:
	candidate_builder();
	~candidate_builder();

    public:
	void BuildVariants( Map* map, state start_state_s, state start_state_n,
			poly_candidate_set& s_variants, poly_candidate_set& n_variants,
			double desired_spd, planning_object::object_manager* object_list	);


    private:

	// s variants
	const double default_dist_gap_ = 10.0;

	poly_candidate_set KeepSpeedS(int lane_idx, 
			state start, 
			double target_spd, 
			std::vector<double> spd_disturb,
			std::vector<double> T);
	poly_candidate_set FollowObjectS(int lane_idx, 
			state start, 
			double time_gap,
			double obj_dist, double obj_spd,
		        std::vector<double> s_disturb,	
			std::vector<double> T);
	poly_candidate_set StopS(int lane_idx, state start, double dist, std::vector<double> T);

	// n variants
	poly_candidate_set GenerateVariantsN(
			int lane_idx,	
			state start_state,
			double n_offset_ref,
			std::vector<double> n_offsets, 
			std::vector<double> t_offsets);

	// feasibility check
    private:
	const double ACC_LIMIT_S = 1.0;
	const double DCC_LIMIT_S = -2.0;
	const double ACC_LIMIT_N = 0.5;
	const double JERK_LIMIT_S = 10.0;
	const double JERK_LIMIT_N = 10.0;
    private:
	poly_candidate_set FilterFeasibleS( poly_candidate_set* variants );
	poly_candidate_set FilterFeasibleN( poly_candidate_set* variants );

	// local functions
    private:
	void AddCandidate( poly_candidate_set& origin, poly_candidate_set added_set ); 
};
