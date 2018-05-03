#include "candidate_builder.h"

#include <iostream>

candidate_builder::candidate_builder(){

}

candidate_builder::~candidate_builder(){

}

void candidate_builder::BuildVariants( Map* map,
		state start_state_s, state start_state_n,
		std::vector<poly_candidate>& s_variants, std::vector<poly_candidate>& n_variants,
		double desired_spd, planning_object::object_manager* object_list	){
    // ============================================
    // build s variants
    poly_candidate_set s_candidates;

    // build s variants -------------------
    // 1. keep the desired speed
    std::vector<double> spd_offsets;
    double offset_resol = 1.0;
    int n_offset = desired_spd / offset_resol;
    for( int i=0; i<n_offset ; i++ ){
	spd_offsets.push_back( - i*offset_resol );
    }
    AddCandidate( s_candidates, KeepSpeedS( -1, start_state_s, desired_spd, spd_offsets, {5.0} ) );

    // 2. following the preceding vehicle
    int start_lane = map->GetLaneIndexN( start_state_n[0] ); 
    planning_object::planning_object_t preceding_vehicle;
    if( object_list->GetPrecedingVehicleSN( start_state_s[0], start_state_n[0], preceding_vehicle ) == true ){
	double dist = preceding_vehicle.s - start_state_s[0];
	double spd = preceding_vehicle.spd;

	AddCandidate( s_candidates, FollowObjectS( start_lane, start_state_s, 2.0, dist, spd, {0.0}, {5.0}) );
    }
    //AddCandidate( s_candidates, StopS( 1, start_state_s, 200, {10} ) );

    // build n variants ---------------------
    poly_candidate_set n_candidates;
    AddCandidate( n_candidates, GenerateVariantsN( 1, start_state_n, 6.0, {0.0}, {4.0} ) );
    //AddCandidate( n_candidates, GenerateVariantsN( 1, start_state_n, 2.0, {-0.5, 0.0, 0.5}, {2.0, 3.0, 4.0} ) );


    // =============================================
    // Feasible candidate
    poly_candidate_set feasible_s_variants = FilterFeasibleS( &s_candidates );
    poly_candidate_set feasible_n_variants =FilterFeasibleN( &n_candidates );

    s_variants = feasible_s_variants;
    n_variants = feasible_n_variants;
}

poly_candidate_set candidate_builder::KeepSpeedS(
		int lane_idx,
		state start, double target_spd,
		std::vector<double> spd_disturb,
		std::vector<double> T) {
    poly_candidate_set s_variants;
    for( int i = 0 ; i < T.size() ; i++ ){
	for( int j=0 ; j < spd_disturb.size(); j++ ){
	    poly_candidate variant( lane_idx, start, {target_spd+spd_disturb[j], 0.0}, T[i] );
	    s_variants.push_back( variant );
	}
    }   
    return s_variants;
}

poly_candidate_set candidate_builder::FollowObjectS( 
		int lane_idx,
		state start, 
		double time_gap,
		double obj_dist, double obj_spd,
		std::vector<double> s_disturb,
		std::vector<double> T){
    // constant time gap policy
    double target_dist = obj_dist - (default_dist_gap_ + obj_spd * time_gap);
    poly_candidate_set s_variants;
    for( int i=0; i<T.size(); i++){
	for( int j=0; j<s_disturb.size() ; j++){
	    poly_candidate variant( lane_idx, start, {target_dist + s_disturb[j], obj_spd, 0.0}, T[i]);
	    s_variants.push_back( variant );
	}
    }
    return s_variants;
}

poly_candidate_set candidate_builder::StopS(
		int lane_idx,
		state start,
		double dist,
		std::vector<double> T){
    poly_candidate_set s_variants;
    for( int i=0; i<T.size() ; i++){
	poly_candidate variant( lane_idx, start, {dist, 0.0, 0.0}, T[i] );
	s_variants.push_back( variant );
    }
    return s_variants;
}

poly_candidate_set candidate_builder::GenerateVariantsN(
		int lane_idx,
		state start_state,
		double n_offset_ref,
		std::vector<double> n_offsets,
		std::vector<double> t_offsets){
    poly_candidate_set n_variants;
    for( int i =0 ; i<n_offsets.size() ; i++){
	    for( int j=0 ; j<t_offsets.size(); j++ ){
		    state target_state = { n_offset_ref + n_offsets[i], 0.0, 0.0};
		    poly_candidate candidate( lane_idx, start_state, target_state, t_offsets[j]); 
		    n_variants.push_back( candidate );
	    }
    }
    return n_variants;
}

poly_candidate_set candidate_builder::FilterFeasibleS(poly_candidate_set* variants){
    poly_candidate_set feasible_set;
    for( int i=0 ; i<variants->size() ; i++){
	if( ( (*variants)[i].GetMaxAcc() < ACC_LIMIT_S )
		&& ( (*variants)[i].GetMinAcc() > DCC_LIMIT_S) 
		&& ( (*variants)[i].GetMaxJerk() < JERK_LIMIT_S )
		    ){

		feasible_set.push_back( (*variants)[i] );
	}
    }
    return feasible_set;
}

poly_candidate_set candidate_builder::FilterFeasibleN(poly_candidate_set* variants ){
    poly_candidate_set feasible_set;
    for( int i=0 ; i<variants->size() ; i++){
	if( ( (*variants)[i].GetMaxAcc() < ACC_LIMIT_N )
		&& ( (*variants)[i].GetMinAcc() > -ACC_LIMIT_N) 
		&& ( (*variants)[i].GetMaxJerk() < JERK_LIMIT_N )
		    ){

		feasible_set.push_back( (*variants)[i] );
	}
    }
    return feasible_set;
}

void candidate_builder::AddCandidate( poly_candidate_set& origin, poly_candidate_set added_set ){
    origin.insert( origin.end(), added_set.begin(), added_set.end() );
}
