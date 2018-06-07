#include "poly_candidate.h"
#include <iostream>

poly_candidate::poly_candidate(int lane_idx, state start, state end, state desired, double T){
    poly_ = solve_poly( start, end, T);
    start_state_ = start;
    end_state_ = end;
    desired_state_ = desired;
    continuation_time_ = T;
    SetLaneIndex( lane_idx );

    if( end.size() == 2 ){
	maneuver_ = KEEPING;
    }
    else if( end.size() == 3){
	if( (end[1] < 1e-5) && (end[2]<1e-5) ){
	    maneuver_ = STOP;
	}
	else{
	    maneuver_ = FOLLOW;
	}
    }
}

poly_candidate::~poly_candidate(){
}

double poly_candidate::GetMaxSpd(){
    return poly_max_speed( poly_, 0.0, continuation_time_ );;
}

double poly_candidate::GetMaxAcc(){
    if( max_acc_ == DEFAULT_ACC ){
	max_acc_ = poly_max_acceleration( poly_, 0.0, continuation_time_);
    }
    return max_acc_;
}

double poly_candidate::GetMinAcc(){
    if( min_acc_ == DEFAULT_ACC ){
	min_acc_ = poly_min_acceleration( poly_, 0.0, continuation_time_ );
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
    if( t < continuation_time_ ){
	return poly_eval_v( poly_, t );
    }
    else{	
	state s_T = poly_eval_v( poly_, continuation_time_ );
	return {
	    s_T[0] + s_T[1]*(t-continuation_time_) + 0.5*s_T[2]*(t-continuation_time_)*(t-continuation_time_),
	    s_T[1] + s_T[2]*(t-continuation_time_),
	    s_T[2]
	};
    }
}

void poly_candidate::CalTotalCost( double w_jerk, double w_time, double w_terminal, double w_violate ){
    weight_jerk_ = w_jerk;
    weight_con_time_ = w_time;
    weight_terminal_ = w_terminal;
    weight_violate_ = w_violate;

    state state_T = GetState( continuation_time_ );
    double violate_s = state_T[0] - violate_dist_;
    if( violate_s < 0 ) violate_s = 0.0;    

    cost_ = w_jerk * GetIntegJerk() / continuation_time_ + 
	w_time * continuation_time_ + 
	w_terminal * 0.5*( end_state_[0] - desired_state_[0])*( end_state_[0] - desired_state_[0]) +
	w_violate * violate_s;
}
