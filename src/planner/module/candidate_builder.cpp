#include "candidate_builder.h"
#include "../data_type/poly_candidate.h"

candidate_builder::candidate_builder(){

}

candidate_builder::~candidate_builder(){

}

void candidate_builder::BuildVariants( state start_state_s, state start_state_n,
		candidate_set& s_variants, candidate_set& n_variants ){
    // build s variants
    candidate_set s_candidates;
    AddCandidate( s_candidates, KeepSpeedS( -1, start_state_s, 10.0, {0.0}, {5.0} ) );
    AddCandidate( s_candidates, FollowObjectS( 1, start_state_s, 2.0, 30, 10, {0.0}, {5.0}) );
    AddCandidate( s_candidates, StopS( 1, start_state_s, 200, {10} ) );

    candidate_set feasible_s_variants = FilterFeasibleS( &s_candidates );

    // build n variants
    candidate_set n_candidates;
    AddCandidate( n_candidates, GenerateVariantsN( 1, start_state_n, 2.0, {-0.5, 0.0, 0.5}, {2.0, 3.0, 4.0} ) );

    candidate_set feasible_n_variants =FilterFeasibleN( &n_candidates );

}

candidate_set candidate_builder::KeepSpeedS(
		int lane_idx,
		state start, double target_spd,
		std::vector<double> spd_disturb,
		std::vector<double> T) {
    candidate_set s_variants;
    for( int i = 0 ; i < T.size() ; i++ ){
	for( int j=0 ; j < spd_disturb.size(); j++ ){
	    poly_candidate variant( lane_idx, start, {target_spd+spd_disturb[j], 0.0}, T[i] );
	    s_variants.push_back( variant );
	}
    }   
    return s_variants;
}

candidate_set candidate_builder::FollowObjectS( 
		int lane_idx,
		state start, 
		double time_gap,
		double obj_dist, double obj_spd,
		std::vector<double> s_disturb,
		std::vector<double> T){
    // constant time gap policy
    double target_dist = obj_dist - (default_dist_gap_ + obj_spd * time_gap);
    candidate_set s_variants;
    for( int i=0; i<T.size(); i++){
	for( int j=0; j<s_disturb.size() ; j++){
	    poly_candidate variant( lane_idx, start, {target_dist + s_disturb[j], obj_spd, 0.0}, T[i]);
	    s_variants.push_back( variant );
	}
    }
    return s_variants;
}

candidate_set candidate_builder::StopS(
		int lane_idx,
		state start,
		double dist,
		std::vector<double> T){
    candidate_set s_variants;
    for( int i=0; i<T.size() ; i++){
	poly_candidate variant( lane_idx, start, {dist, 0.0, 0.0}, T[i] );
	s_variants.push_back( variant );
    }
    return s_variants;
}

candidate_set candidate_builder::GenerateVariantsN(
		int lane_idx,
		state start_state,
		double n_offset_ref,
		std::vector<double> n_offsets,
		std::vector<double> t_offsets){
    candidate_set n_variants;
    for( int i =0 ; i<n_offsets.size() ; i++){
	    for( int j=0 ; j<t_offsets.size(); j++ ){
		    state target_state = { n_offsets[i], 0.0, 0.0};
		    poly_candidate candidate( lane_idx, start_state, target_state, t_offsets[j]); 
		    n_variants.push_back( candidate );
	    }
    }
    return n_variants;
}

candidate_set candidate_builder::FilterFeasibleS(candidate_set* variants){
    return (*variants);
}

candidate_set candidate_builder::FilterFeasibleN(candidate_set* variants ){
    return (*variants);
}

void candidate_builder::AddCandidate( candidate_set& origin, candidate_set added_set ){
    origin.insert( origin.end(), added_set.begin(), added_set.end() );
}
