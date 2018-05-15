#include "candidate_builder.h"

#include <iostream>

candidate_builder::candidate_builder(){
    bool is_config = config_parser_.Init( "../src/planner/config/config_candidate.ini" );
}

candidate_builder::~candidate_builder(){

}

void candidate_builder::BuildVariants( Map* map,
		state start_state_s, state start_state_n,
		std::vector<poly_candidate>& s_variants, std::vector<poly_candidate>& n_variants,
		candidate_weight weight,
		double desired_spd, planning_object::object_manager* object_list	){
    ProcessINI();
    // ============================================
    // Save weight
    weight_ = weight;

    // ============================================
    // build s variants
    poly_candidate_set s_candidates;

    // build s variants -------------------
    int start_lane = map->GetLaneIndexN( start_state_n[0] ); 

    // find preceding vehicle
    veh_in_lane_t vehicle_in_lanes;
    for( int lane = 0; lane < map->GetNumLane(); lane++){
	planning_object::planning_object_t preceding_vehicle;
	if( (object_list->GetPrecedingVehicleLane( start_state_s[0], lane, preceding_vehicle ) == true)
	&& ( (preceding_vehicle.s - start_state_s[0]) < 150.0 ) ){
	    //std::cout << "preceding: " << lane << "/" << preceding_vehicle.s - start_state_s[0] << "/" << preceding_vehicle.spd << std::endl;
	    vehicle_in_lanes.insert( std::pair<int, planning_object::planning_object_t>( lane, preceding_vehicle ) );
	}
    }

    // 1. keep the desired speed
    for( int lane = 0 ; lane < map->GetNumLane(); lane++){
	double ref_spd = desired_spd;
	
	// make spd disturb
	std::vector<double> spd_offsets;
	double offset_resol = 1.0;
	int n_offset = ref_spd / offset_resol + 1;
	for( int i=0; i<n_offset ; i++ ){
	    spd_offsets.push_back( (-desired_spd) + i*offset_resol );
	}

	// check if there is a preceding vehicle
	std::pair< veh_in_lane_t::iterator, veh_in_lane_t::iterator > it_preceding;
	it_preceding = vehicle_in_lanes.equal_range( lane );
	if( it_preceding.first != it_preceding.second ){
	    double preceding_spd = (it_preceding.first->second).spd; 
	    double preceding_s = (it_preceding.first->second).s; 
	    //double max_spd = sqrt( preceding_spd*preceding_spd + 1.0 * 5.0 * (preceding_s - start_state_s[0]) );
	    //double max_spd = preceding_spd;
	    //if( max_spd < ref_spd) ref_spd = max_spd;

	    AddCandidate( s_candidates, KeepSpeedSWithPrecedingVeh( lane, start_state_s, desired_spd, spd_offsets, {2.0, 3.0, 4.0, 5.0} , acc_time_gap_, preceding_s, preceding_spd));
	}
	else{
	    AddCandidate( s_candidates, KeepSpeedS( lane, start_state_s, desired_spd, spd_offsets, {2.0, 3.0, 4.0, 5.0} ) );
	}


	// generate candidate
    }
   
    std::vector<double>  vari;
    for( int i=0; i<9; i++){
	vari.push_back( start_state_s[1] - desired_spd + (i-5)*0.5 );
    }
    //AddCandidate( s_candidates, KeepSpeedS( -1, start_state_s, desired_spd, vari, {2.0, 3.0, 4.0, 5.0} ) );

    // 2. following the preceding vehicle
    for( int lane = 0 ; lane < map->GetNumLane(); lane++){
	planning_object::planning_object_t preceding_vehicle;
	// check if there is a preceding vehicle
	std::pair< veh_in_lane_t::iterator, veh_in_lane_t::iterator > it_preceding;
	it_preceding = vehicle_in_lanes.equal_range( lane );
	if( it_preceding.first != it_preceding.second ){
	  double dist = (it_preceding.first->second).s;
	    double spd = (it_preceding.first->second).spd;

	    std::cout << "object(lane/dist/spd): " << lane << "/" << dist-start_state_s[0] << "/" << spd << std::endl;

	    AddCandidate( s_candidates, FollowObjectS( lane, start_state_s, acc_time_gap_, dist, spd, {0.0}, {1.0, 2.0, 3.0, 4.0, 5.0}) );
	}
    }
    //AddCandidate( s_candidates, StopS( 1, start_state_s, 200, {10} ) );

    // build n variants ---------------------
    poly_candidate_set n_candidates;
    for( int lane = 0; lane<map->GetNumLane(); lane++){
	AddCandidate( n_candidates, GenerateVariantsN( lane, start_state_n, map->GetOffset(lane), {0.0}, {1.0, 2.0, 3.0, 4.0, 5.0} ) );
    }
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
	    poly_candidate variant( lane_idx, start, {target_spd+spd_disturb[j], 0.0}, {target_spd, 0.0},  T[i] );
	    variant.CalTotalCost( weight_.s_jerk, weight_.s_time, weight_.s_keep_spd );
	    s_variants.push_back( variant );
	}
    }   
    return s_variants;
}

poly_candidate_set candidate_builder::KeepSpeedSWithPrecedingVeh(
		int lane_idx,
		state start, double target_spd,
		std::vector<double> spd_disturb,
		std::vector<double> T,
		double acc_time_gap,
		double preceding_s,
		double preceding_spd ) {
    poly_candidate_set s_variants;
    for( int i = 0 ; i < T.size() ; i++ ){
	for( int j=0 ; j < spd_disturb.size(); j++ ){
	    poly_candidate variant( lane_idx, start, {target_spd+spd_disturb[j], 0.0}, {target_spd, 0.0},  T[i] );
	    double acc_dist = CalFollowTargetS( T[i], default_dist_gap_, acc_time_gap, preceding_s, preceding_spd );
	    variant.SetViolateDist( acc_dist );
	    variant.CalTotalCost( weight_.s_jerk, weight_.s_time, weight_.s_keep_spd, weight_.s_violate );
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
    poly_candidate_set s_variants;
    for( int i=0; i<T.size(); i++){
	for( int j=0; j<s_disturb.size() ; j++){
	    double target_pos = CalFollowTargetS( T[i], default_dist_gap_, time_gap, obj_dist, obj_spd );
	    poly_candidate variant( lane_idx, start, {target_pos + s_disturb[j], obj_spd, 0.0}, {target_pos, obj_spd, 0.0}, T[i]);
	    variant.CalTotalCost( weight_.s_jerk, weight_.s_time, weight_.s_follow );
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
	poly_candidate variant( lane_idx, start, {dist, 0.0, 0.0}, {dist, 0.0}, T[i] );
	variant.CalTotalCost( weight_.s_jerk, weight_.s_time, weight_.s_stop );
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
		    poly_candidate candidate( lane_idx, start_state, target_state, {n_offset_ref, 0.0}, t_offsets[j]); 
		    candidate.CalTotalCost( weight_.n_jerk, weight_.n_time, weight_.n_offset );
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

double candidate_builder::CalFollowTargetS( double T, double default_dist, double time_gap, double preceding_s, double preceding_spd ){
    return ( preceding_s + preceding_spd*T ) - ( default_dist + preceding_spd * time_gap );
}

void candidate_builder::ProcessINI( ){
    if( config_parser_.IsFileUpdated() == true ){
	std::cout << "candidate config is updated" << std::endl;
	config_parser_.ParseConfig( "candidate", "acc_dist_gap", default_dist_gap_);
	config_parser_.ParseConfig( "candidate", "acc_time_gap", acc_time_gap_);
	config_parser_.ParseConfig( "candidate", "acc_limit_s", ACC_LIMIT_S);
	config_parser_.ParseConfig( "candidate", "dcc_limit_s", DCC_LIMIT_S);
	config_parser_.ParseConfig( "candidate", "acc_limit_n", ACC_LIMIT_N);
	config_parser_.ParseConfig( "candidate", "jerk_limit_s", JERK_LIMIT_S);
	config_parser_.ParseConfig( "candidate", "jerk_limit_n", JERK_LIMIT_N);
    }
}
