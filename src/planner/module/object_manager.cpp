#include "object_manager.h"
#include <iostream>

using namespace planning_object;

object_manager::object_manager( Map map, std::vector< std::vector<double> > object_list ):
map_( map){
    AddObjects( object_list );
}

void object_manager::AddObjects( std::vector<std::vector<double>> objects ){
    // save object data
    int num_obj = objects.size();
    for( int i=0 ; i<num_obj; i++){
	planning_object_t planning_object;
	planning_object.x = objects[i][1];
	planning_object.y = objects[i][2];
	planning_object.yaw = atan2( objects[i][4], objects[i][3] );
	planning_object.spd = sqrt( objects[i][3]*objects[i][3]
				    + objects[i][4]*objects[i][4] );
	planning_object.s = objects[i][5];
	planning_object.n = objects[i][6];

	planning_object.length = 5.0;
	planning_object.width = 2.2;

	int lane_id = AssociateLane( planning_object.n );
	planning_object.lane_id = lane_id;
	planning_object.id = i;

	object_list_.push_back( planning_object );
    }

    // associate lane
    for( int i =0; i< object_list_.size(); i++){
	lane_index_.insert( std::pair<int, planning_object_t*>(object_list_[i].lane_id, &object_list_[i] ));
    }

}

int object_manager::AssociateLane( double n ){
    int num_lane = map_.GetNumLane();
    double min_dist = 10;
    double min_idx = -1;
    for( int i=0; i< num_lane ; i++){
	double lane_offset = map_.GetOffset( i );
	double dist = abs( n - lane_offset );
	if( dist < min_dist ){
	    min_dist = dist;
	    min_idx = i;
	}
    }
    return min_idx;
}

obj_predict_t object_manager::PredictMotion( obj_state_t state, double dt, double T){
    obj_predict_t predicted_trajectory;
    int num_steps = (int)(T / dt);
    for( int i=0; i<num_steps ; i++){
	double time = dt * i;
	double s = state.s + state.spd * time;
	double n = state.n;
	std::vector<double> xy = map_.ToCartesian( s, n );
	obj_stateT_t stateT;
	stateT.time = time;
	stateT.x = xy[0];
	stateT.y = xy[1];
	stateT.s = s;
	stateT.n = n;
	stateT.spd = state.spd;
	stateT.yaw = map_.GetSlope( s );

	predicted_trajectory.push_back( stateT );
    }
    return predicted_trajectory;
}

obj_predict_t object_manager::PredictMotion( obj_state_t state, double dt, double start_time, double end_time){
    obj_predict_t predicted_trajectory;
    int num_steps = (int)((end_time-start_time) / dt);
    
    for( int i=0; i<num_steps ; i++){
	double time = start_time + dt * i;
	double s = state.s + state.spd * time;
	double n = state.n;
	std::vector<double> xy = map_.ToCartesian( s, n );
	obj_stateT_t stateT;
	stateT.time = time;
	stateT.x = xy[0];
	stateT.y = xy[1];
	stateT.s = s;
	stateT.n = n;
	stateT.spd = state.spd;
	stateT.yaw = map_.GetSlope( s );

	predicted_trajectory.push_back( stateT );
    }
    return predicted_trajectory;
}

planning_object_list_t object_manager::GetWatchable( double s, double n ){
    const double max_view_dist = 100.0;
    const int max_view_lane = 1;

    int lane_idx = map_.GetLaneIndexN( n );

    double s_max = s + max_view_dist;
    double s_min = s - max_view_dist;
    double n_min = map_.GetOffset( lane_idx - 1 ) - map_.GetLaneWidth()/2;
    if( n_min < 0 ) n_min = 0.0;
    double n_max = map_.GetOffset( lane_idx + 1 ) + map_.GetLaneWidth()/2;    

    return  ObjectsInRange( s_min, s_max, n_min, n_max );
}

planning_object_list_t object_manager::ObjectsInRange( double s_min, double s_max, double n_min, double n_max){
    planning_object_list_t in_objects;
    for( int i =0; i<object_list_.size() ; i++){
	planning_object_t obj = object_list_[i];
	if( ( obj.s > s_min ) &&
		(obj.s < s_max ) &&
		(obj.n > n_min ) &&
		(obj.n < n_max ) ){
	    in_objects.push_back( obj );
	}
    }
    return in_objects;
}

bool object_manager::GetPrecedingVehicleLane( double s, int lane_idx, planning_object_t& preceding){
    // find object in the lane (lane_idx)
    if( lane_index_.count( lane_idx ) == 0 ) return false;
    std::pair< std::map<int, planning_object_t*>::iterator, std::map<int, planning_object_t*>::iterator> it;
    it = lane_index_.equal_range( lane_idx );

    // find precediing vehicle
    double min_dist = 1000;
    bool is_preceding = false;
    for( std::map<int, planning_object_t*>::iterator it_ob = it.first ; it_ob != it.second ; ++it_ob ){
	planning_object_t* object_p = it_ob->second;
	
	double dist = object_p->s - s;
	if( (dist < min_dist) && (s < object_p->s)){
		min_dist = dist;
		preceding = *object_p;
		is_preceding = true;
	}
    }

    if( is_preceding == true)	return true;
    else			return false;
}

bool object_manager::GetPrecedingVehicleXY( double x, double y, double yaw, 
	planning_object_t& preceding){
    std::vector<double> sn = map_.ToFrenet(x,y );
    return GetPrecedingVehicleSN( sn[0], sn[1], preceding );
}

bool object_manager::GetPrecedingVehicleSN( double s, double n, 
	planning_object_t& preceding){
    int lane_idx = map_.GetLaneIndexN( n );

    return GetPrecedingVehicleLane( s, lane_idx, preceding );
}

bool object_manager::GetObject( int object_id,
					    planning_object_t& object	){
    for( int i=0; i<object_list_.size() ; i++){
	if( object_id == object_list_[i].id ){
	    object = object_list_[i];
	    return true;
	}
    }	
    return false;
}

planning_object_list_t object_manager::GetAllObjects(){
    return object_list_;
}

planning_object_list_t object_manager::GetObjectsInLane( int lane_idx ){
    planning_object_list_t list;

    std::pair< std::map<int, planning_object_t*>::iterator, std::map<int, planning_object_t*>::iterator> it;
    it = lane_index_.equal_range( lane_idx );
    for( std::map<int, planning_object_t*>::iterator it_ob = it.first ; it_ob != it.second ; ++it_ob ){
	list.push_back( *it_ob->second );	
    }
    return list;
}

bool object_manager::IsCollision( double start_time,
	double dt,
	std::vector<double> trj_s, 
	std::vector<double> trj_n, 
	double length, double width){
    std::vector<int> size_list = { (int)trj_s.size(), (int)trj_n.size()};
    std::sort( size_list.begin(), size_list.end() );
    int num_node = size_list.front();
    if( num_node == 0 ) return false;
    double T = dt * num_node;

    std::vector<double> sn_start = { trj_s[0], trj_n[0] };
    planning_object_list_t watchable = GetWatchable( sn_start[0], sn_start[1] );

    // predict future motino of watchable objects and collision check
    bool is_collision = false;
    for( int i=0; i< watchable.size() ; i++){
        if( watchable[i].prediction_dt_ != dt ){
	    watchable[i].trajectory = PredictMotion( watchable[i], dt, start_time, start_time+T);
	}

	std::cout << "watchable(d/n): " << watchable[i].s-sn_start[0] << "/" << watchable[i].n << std::endl;

	std::vector<double> path_s;
	std::vector<double> path_n;
	for( int j=0; j<watchable[i].trajectory.size() ; j++){
	    path_s.push_back( watchable[i].trajectory[j].s);
	    path_n.push_back( watchable[i].trajectory[j].n);
	}
	watchable[i].collision_checker_ = path_planning::AABBTree::Build( path_s, path_n, watchable[i].length, watchable[i].width);
	
	int check = watchable[i].collision_checker_.CollisionIndex( trj_s, trj_n, length, width );
	if( check != -1 ){
	    std::cout << "collision" << std::endl;
	    is_collision = true;
	    break;
	}
    }
    return is_collision;
}

