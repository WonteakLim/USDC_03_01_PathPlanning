#include <iostream>
#include "planning_data_type.h"

trajectory::trajectory( 
	int idx, 
	candidate* s_candidate, candidate* n_candidate,
	int desired_lane,
	double s_weight, double n_weight, double lane_weight	)
: idx_( idx ),
lane_idx_( s_candidate->GetLaneIndex() ),
desired_lane_idx_( desired_lane ),
s_trajectory_( s_candidate ),
n_trajectory_( n_candidate ){
    time_horizon_ = CalTimeHorizon( );
    cost_ = CalCost( s_weight, n_weight, lane_weight );
}

double trajectory::CalCost(double s_weight, double n_weight, double lane_weight){
    return s_weight*s_trajectory_->GetCost()
	+ n_weight*n_trajectory_->GetCost()
	+ lane_weight*abs(lane_idx_ - desired_lane_idx_)
	+ 5000.0 * ( 21 - ( (s_trajectory_->GetState(1.0))[0]- (s_trajectory_->GetState(0.0))[0])) / 21;
}

double trajectory::CalTimeHorizon( ){
    double t_s = s_trajectory_->GetT();
    double t_n = n_trajectory_->GetT();
    double max_t = 0.0;
    if( t_s > t_n ) max_t = t_n;
    else	    max_t = t_s;
    return max_t; 
}

// ================================
// External interface
sn_state trajectory::GetNode( double t ){
    sn_state sn;
    sn.s = s_trajectory_->GetState( t );
    sn.n = n_trajectory_->GetState( t );

    return sn;
}

std::vector<double> trajectory::GetMaxSpd(){
    return { s_trajectory_->GetMaxSpd(), n_trajectory_->GetMaxSpd() };
}

int trajectory::GetDiscretePathSN( double dt, double T,
	std::vector<double>& path_s, std::vector<double>& path_n ){
    int num_node = (int)(T/dt);
    for( int i=0; i<num_node ; i++){
	double t = dt * i;
	path_s.push_back( s_trajectory_->GetState( t )[0] );
	path_n.push_back( n_trajectory_->GetState( t )[0] );;
    }
}
