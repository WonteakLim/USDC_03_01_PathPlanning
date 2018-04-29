#pragma once
#include "../data_type/planning_data_type.h"
#include "../data_type/poly.h"
#include "../utils/spline.h"

typedef std::vector<candidate> candidate_set;

class candidate_builder{
    public:
	candidate_builder();
	~candidate_builder();

    public:
	void BuildVariants( state start_state_s, state start_state_n,
			candidate_set& s_variants, candidate_set& n_variants );

    private:

    private:

	// s variants
	const double default_dist_gap_ = 10.0;

	candidate_set KeepSpeedS(int lane_idx, 
			state start, 
			double target_spd, 
			std::vector<double> spd_disturb,
			std::vector<double> T);
	candidate_set FollowObjectS(int lane_idx, 
			state start, 
			double time_gap,
			double obj_dist, double obj_spd,
		        std::vector<double> s_disturb,	
			std::vector<double> T);
	candidate_set StopS(int lane_idx, state start, double dist, std::vector<double> T);

	// n variants
	candidate_set GenerateVariantsN(
			int lane_idx,	
			state start_state,
			double n_offset_ref,
			std::vector<double> n_offsets, 
			std::vector<double> t_offsets);

	candidate_set FilterFeasibleS( candidate_set* variants );
	candidate_set FilterFeasibleN( candidate_set* variants );

//	std::vector<double> InLaneVariantN( 
	// local functions
    private:
	void AddCandidate( candidate_set& origin, candidate_set added_set ); 
};
