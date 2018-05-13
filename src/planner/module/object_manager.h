#pragma once
#include <vector>
#include <map>
#include "../utils/spline.h"
#include "../utils/aabb.h"

namespace planning_object{
    class obj_state_t{
	public:
	    double x;
	    double y;
	    double yaw;
	    double spd;
	    double s;
	    double n;
    };

    class obj_stateT_t : public obj_state_t{
	public:
	    double time;
    };

    class obj_shape_t{
	public:
	    double length = 5.0;
	    double width = 2.2;
    };

    typedef std::vector<obj_stateT_t> obj_predict_t;

    class planning_object_t : public obj_state_t, public obj_shape_t{
	public:
	    int id;
	    int lane_id;
	    obj_predict_t trajectory;

	    double prediction_dt_ = -1.0;
	    path_planning::AABBTree collision_checker_;
    };

    typedef std::vector<planning_object_t> planning_object_list_t;

    class object_manager{
	public:
	    object_manager( Map map, std::vector<std::vector<double>> object_list );
	    ~object_manager() {}

	private:
	    planning_object_list_t object_list_;

	private:
	    Map map_;

	private:
	    std::multimap<int, planning_object_t*> lane_index_;

	// local function
	private:
	    void		    AddObjects( std::vector<std::vector<double>> object );
	    int			    AssociateLane( double n ); 
	    obj_predict_t	    PredictMotion( obj_state_t state, double dt, double T );
	    obj_predict_t	    PredictMotion( obj_state_t state, double dt, double start_time, double end_time);
	    planning_object_list_t  GetWatchable( double s, double n );
	    planning_object_list_t  ObjectsInRange( double s_min, double s_max, double n_min, double n_max );
	// =============================
	// External interface
	// =============================
				    // [x, y, vx, vy, s, n]

	public:
	    bool		    GetPrecedingVehicleLane(double s, int lane_idx, planning_object_t& preceding);
	    bool		    GetPrecedingVehicleXY(double x, double y, double yaw, 
							planning_object_t& preceding);
	    bool		    GetPrecedingVehicleSN( double s, double n ,
							planning_object_t& preceding);
	    bool		    GetObject(int object_id,
							planning_object_t& object);
	    planning_object_list_t  GetAllObjects();
	    planning_object_list_t  GetObjectsInLane( int lane_idx );
	    bool		    IsCollision(double start_time,
						double dt, 
						std::vector<double> trj_s, 
						std::vector<double> trj_n, 
						double length, double width );
	     

    };

}

