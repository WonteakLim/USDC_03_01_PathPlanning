#include <iostream>
#include "start_selector.h"

// ==============================
// discrete_trajectory
discrete_trajectory::discrete_trajectory( double dt, std::vector<double> path_x, std::vector<double> path_y )
:time_interval_( dt ),
path_x_( path_x ),
path_y_( path_y ){
    if( (path_x.size() != 0 )
	    && (path_y.size() != 0 )
	    && (path_x.size() == path_y.size() )
	    && (dt > 0) ){
	num_node_ = path_x.size();
	valid_trajectory_ = true;
    }
    else{
	valid_trajectory_ = false;
    }
}

bool discrete_trajectory::IsValid(void){
    return valid_trajectory_;
}

bool discrete_trajectory::IsOnTrajectory(double pose_x, double pose_y){
    if( valid_trajectory_ == false ) return false;
    // Get the closest node
    std::vector<double> closest_node_xy = FindClosestNode( pose_x, pose_y );
    // Get a distance
    double dist = GetDistance( pose_x, pose_y, closest_node_xy[0], closest_node_xy[1] );
    std::cout << "Closet point: " << closest_node_xy[0] << ", " << closest_node_xy[1] << std::endl;
    std::cout << "Distance: " << dist << "!!!!!" << std::endl;
    // Determine if the offset is valid or not
    if( dist > THRESHOLD_VALID_OFFSET )
	return false;
    else
	return true;        
}

std::vector<double> discrete_trajectory::FindClosestNode( double pose_x, double pose_y ){
    int min_idx = FindClosestNodeIdx( pose_x, pose_y );
    return { path_x_[min_idx], path_y_[min_idx] };
}

int discrete_trajectory::FindClosestNodeIdx( double pose_x, double pose_y ){
    double min_dist = 10000.0;
    int min_idx = 0;
    for( int i=0; i<path_x_.size(); i++){
	double dist = GetDistance( path_x_[i], path_y_[i], pose_x, pose_y );
	if( min_dist > dist ){
	    min_dist = dist;
	    min_idx = i;
	}
    }
    return min_idx;
}

xy_state discrete_trajectory::GetNode( int idx ){
    // Get xy state
    xy_state source_xy;
    source_xy.x.push_back( path_x_[idx] );
    source_xy.y.push_back( path_y_[idx] );

    if( (idx!=0) && (idx != path_x_.size()-1 )  ){
	std::vector<double> xy_spd = GetSpd( time_interval_,
		{path_x_[idx], path_y_[idx]}, {path_x_[idx+1], path_y_[idx+1]} );
	std::vector<double> xy_acc = GetAcc( time_interval_,
		{ path_x_[idx-1], path_y_[idx-1] },
		{ path_x_[idx], path_y_[idx]},
		{ path_x_[idx+1], path_y_[idx+1]} );

	source_xy.x.push_back( xy_spd[0] );
	source_xy.x.push_back( xy_acc[0] );
	source_xy.y.push_back( xy_spd[1] );
	source_xy.y.push_back( xy_acc[1] );
    }
    else if( idx == 0 ){	
	std::vector<double> xy_spd = GetSpd( time_interval_,
		{path_x_[idx], path_y_[idx]}, {path_x_[idx+1], path_y_[idx+1]} );
	std::vector<double> xy_acc = GetAcc( time_interval_,
		{ path_x_[idx], path_y_[idx] },
		{ path_x_[idx+1], path_y_[idx+1]},
		{ path_x_[idx+2], path_y_[idx+2]} );
	source_xy.x.push_back( xy_spd[0] );
	source_xy.x.push_back( xy_acc[0] );
	source_xy.y.push_back( xy_spd[1] );
	source_xy.y.push_back( xy_acc[1] );
    }
    else{
	std::vector<double> xy_spd = GetSpd( time_interval_,
		{path_x_[idx-1], path_y_[idx-1]}, {path_x_[idx], path_y_[idx]} );
	std::vector<double> xy_acc = GetAcc( time_interval_,
		{ path_x_[idx-2], path_y_[idx-2] },
		{ path_x_[idx-1], path_y_[idx-1]},
		{ path_x_[idx], path_y_[idx]} );
	source_xy.x.push_back( xy_spd[0] );
	source_xy.x.push_back( xy_acc[0] );
	source_xy.y.push_back( xy_spd[1] );
	source_xy.y.push_back( xy_acc[1] );
    }
    return source_xy;
}

double discrete_trajectory::GetDistance( double x1, double y1, double x2, double y2 ){
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt( dx*dx + dy*dy );
}

std::vector<double> discrete_trajectory::GetSpd( double dt,
	std::vector<double> s1,
	std::vector<double> s2 ){
    if( dt < 1e-5 ) return {0.0, 0.0};
    double dx = s2[0] - s1[0];
    double dy = s2[1] - s1[1];
    return { dx/dt, dy/dt  };
}

std::vector<double> discrete_trajectory::GetAcc( double dt,
	std::vector<double> s1,
	std::vector<double> s2,
	std::vector<double> s3 ){    
    if( dt < 1e-5 ) return {0.0, 0.0};
    double a_x = ( s3[0] - 2*s2[0] + s1[0] ) / (dt*dt);
    double a_y = ( s3[1] - 2*s2[1] + s1[1] ) / (dt*dt);
    return {a_x, a_y}; 
}


// ====================================
// StartSelector
sn_state start_selector::SelectStartNode(
		Map* map,
		double prev_path_dt,
		std::vector<double> prev_path_x, 
		std::vector<double> prev_path_y,
		std::vector<double> ego_pose,
		double lookahead ){
    sn_state start_node;

    double ego_x = ego_pose[0];
    double ego_y = ego_pose[1];
    double ego_yaw = ego_pose[2];
    double ego_spd = ego_pose[3];
    double ego_acc = ego_pose[4];

    discrete_trajectory prev_trj( prev_path_dt, prev_path_x, prev_path_y );

    // check if previous trajectory is valid or not
    bool is_valid_trj = prev_trj.IsValid();
    bool is_on_trj = prev_trj.IsOnTrajectory( ego_x, ego_y );
      
    // start node selection
    if( (is_valid_trj == true ) && 
	    (is_on_trj == true) ){
	// from the previous trajectory
	start_node = FindLookaheadNode( map, &prev_trj,
		ego_x, ego_y, ego_yaw, lookahead );
	std::cout << "start from the previous path" << std::endl;
    }
    else{
	// from ego position
	state x = {ego_x, ego_spd*cos(ego_yaw), ego_acc*cos(ego_yaw)};
	state y = {ego_y, ego_spd*sin(ego_yaw), ego_acc*sin(ego_yaw)};
	start_node = ConvXY2SN( map, {x,y} );
	std::cout << "start from ego position" << std::endl;
	std::cout << "ego pos(x/y): " << ego_x << "/" << ego_y << std::endl;
    }
    return start_node;
}

sn_state start_selector::FindLookaheadNode( Map* map,
	discrete_trajectory* trj,
	double ego_x, double ego_y, double ego_yaw, double lookahead ){
    // find the closest node
    int ref_idx = trj->FindClosestNodeIdx( ego_x, ego_y );
    // find the index of the lookahead node
    double trj_time_interval = trj->GetTimeInterval();
    int lookahead_idx = lookahead / trj_time_interval;
    int effective_idx = ref_idx + lookahead_idx;
    
    int path_size = trj->GetNumNode();
    if( path_size <= lookahead_idx ){
	effective_idx = path_size - 1;
    }

    // get the lookahead node
    xy_state lookahead_node_xy = trj->GetNode( effective_idx );
    sn_state lookahead_node_sn = ConvXY2SN( map, lookahead_node_xy );

    return lookahead_node_sn;
}

sn_state start_selector::ConvXY2SN( Map* map, xy_state xy){
    std::cout << "conv (x/y): " << xy.x[0] << "/" << xy.y[0] << std::endl;
    std::vector<double> sn = map->ToFrenet( xy.x[0], xy.y[0] );
    double slop = map->GetSlope( sn[0] );
    double v_s = xy.x[1] * cos( slop ) + xy.y[1] * sin( slop ) ;
    double v_n = - xy.x[1] * sin( slop ) + xy.y[1] * cos( slop ) ;
    double a_s = xy.x[2] * cos( slop ) + xy.y[2] * sin( slop ) ;
    double a_n = - xy.x[2] * sin( slop ) + xy.y[2] * cos( slop ) ;

    state s_state = { sn[0], v_s, a_s };
    state n_state = { sn[1], v_n, a_n };
    std::cout << "conv (s/n): " << s_state[0] << "/" << n_state[0] << std::endl;

    return { s_state, n_state };
}

