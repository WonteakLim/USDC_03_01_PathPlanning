#include <iostream>
#include "planning_data_type.h"

trajectory::trajectory( 
	int idx, 
	candidate* s_candidate, candidate* n_candidate,
        double desired_spd,
	trajectory_weight weight	)
: idx_( idx ),
s_trajectory_( s_candidate ),
n_trajectory_( n_candidate ),
desired_spd_(desired_spd){
    time_horizon_ = CalTimeHorizon( );
    cost_ = CalCost( weight );
}

double trajectory::CalCost(trajectory_weight weight){
    double s_jerk = s_trajectory_->GetIntegJerk();
    double n_jerk = n_trajectory_->GetIntegJerk();
    double s_diff_spd = abs( s_trajectory_->GetState( s_trajectory_->GetT())[1] - desired_spd_);
    
    double cost = weight.s_comfort * s_jerk
	    	+ weight.n_comfort * n_jerk
		+ weight.s_desired_spd * s_diff_spd;
    std::cout << "id/spd/spd_r/cost: " << idx_ << "/" << s_trajectory_->GetState(s_trajectory_->GetT())[1] << "/" << desired_spd_ << "/" << cost << std::endl;
    return cost;
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
std::vector<state> trajectory::GetNode( double t ){
    state s_state = s_trajectory_->GetState( t );
    state n_state = n_trajectory_->GetState( t );

    return {s_state, n_state };
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
