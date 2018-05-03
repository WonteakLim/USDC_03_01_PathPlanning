#include "poly_candidate.h"
#include <iostream>

poly_candidate::poly_candidate(int lane_idx, state start, state end, double T){
    poly_ = solve_poly( start, end, T);
    start_state_ = start;
    end_state_ = end;
    continuation_time_ = T;
    SetLaneIndex( lane_idx );
}

poly_candidate::~poly_candidate(){
}

double poly_candidate::GetMaxSpd(){
    return 0.0;
}

double poly_candidate::GetMaxAcc(){
    if( max_acc_ == DEFAULT_ACC ){
	max_acc_ = poly_max_acc_signed( poly_, 0.0, continuation_time_);
    }
    return max_acc_;
}

double poly_candidate::GetMinAcc(){
    if( min_acc_ == DEFAULT_ACC ){
	min_acc_ = poly_min_acc_signed( poly_, 0.0, continuation_time_ );
    }
    return min_acc_;
}

double poly_candidate::GetMaxJerk(){
    if( max_jerk_ < 0.0 ){
	max_jerk_ = poly_max_jerk( poly_, 0, continuation_time_ );
    }
    return max_jerk_;
}

double poly_candidate::GetIntegJerk(){
    if( integ_jerk_ < 0.0 ){
	integ_jerk_ = poly_jerk_integral( poly_, continuation_time_ );
    }
    return integ_jerk_;
}

double poly_candidate::GetT(){
    return continuation_time_;
}

state poly_candidate::GetState( double t ){
    return poly_eval_v( poly_, t );
}


