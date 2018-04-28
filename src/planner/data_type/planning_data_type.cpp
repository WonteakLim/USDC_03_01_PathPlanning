#include "planning_data_type.h"

trajectory::trajectory( candidate s_candidate, candidate n_candidate,
       trajectory_weight weight	)
:s_trajectory_( s_candidate ),
n_trajectory_( n_candidate ){
    time_horizon_ = CalTimeHorizon( s_candidate, n_candidate );
    cost_ = CalCost( weight );
}

double trajectory::CalCost(trajectory_weight weight){
    return 0.0;
}

double trajectory::CalTimeHorizon( candidate s, candidate n ){
    double t_s = s_trajectory_.GetT();
    double t_n = n_trajectory_.GetT();
    double max_t = 0.0;
    if( t_s > t_n ) max_t = t_s;
    else	    max_t = t_n;
    return max_t; 
}

// ================================
// External interface
std::vector<state> trajectory::GetNode( double t ){
    state s_state = s_trajectory_.GetState( t );
    state n_state = n_trajectory_.GetState( t );

    return {s_state, n_state };
}
