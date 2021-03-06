#pragma once
#include <map>

#include "../data_type/planning_data_type.h"
#include "../data_type/poly.h"
#include "../data_type/poly_candidate.h"
#include "../utils/spline.h"
#include "../utils/ini_parser/ini_parser.h"
#include "object_manager.h"

typedef std::map<int, planning_object::planning_object_t> veh_in_lane_t;

struct candidate_weight{
    double s_jerk;
    double s_time;
    double s_stop;
    double s_follow;
    double s_keep_spd;
    double s_violate;
    double n_jerk;
    double n_time;
    double n_offset;
};

class candidate_builder{
    public:
	candidate_builder();
	~candidate_builder();

    public:
	void BuildVariants( Map* map, state start_state_s, state start_state_n,
			poly_candidate_set& s_variants, poly_candidate_set& n_variants,
			candidate_weight weight,
			double desired_spd, planning_object::object_manager* object_list	);


    private:
	// s variants
	candidate_weight weight_;
	
	poly_candidate_set KeepSpeedS(int lane_idx, 
			state start, 
			double target_spd, 
			std::vector<double> spd_disturb,
			std::vector<double> T);
	poly_candidate_set KeepSpeedSWithPrecedingVeh(int lane_idx, 
			state start, 
			double target_spd, 
			std::vector<double> spd_disturb,
			std::vector<double> T,
			double acc_time_gap, double preceding_s, double preceding_spd);
	poly_candidate_set KeepSpeedSWithPreceding(int lane_idx, 
			state start, 
			double target_spd, 
			std::vector<double> spd_disturb,
			std::vector<double> T,
			double preceding_s,
			double preceding_spd);
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
	poly_candidate_set FilterFeasibleS( poly_candidate_set* variants );
	poly_candidate_set FilterFeasibleN( poly_candidate_set* variants );

	// local functions
    private:
	void AddCandidate( poly_candidate_set& origin, poly_candidate_set added_set );
	double CalFollowTargetS(double T, double default_dist, double time_gap, 
				    double preceding_s, double preceding_spd );

	// ========================================	
	// Configuration
    private:
	CINI_Parser config_parser_;
	void ProcessINI();
    private:
	double default_dist_gap_ = 10.0;
	double acc_time_gap_ = 2.0;
	double ACC_LIMIT_S = 10.0;
	double DCC_LIMIT_S = -10.0;
	double ACC_LIMIT_N = 2.0;
	double JERK_LIMIT_S = 80.0;
	double JERK_LIMIT_N = 80.0;
};
