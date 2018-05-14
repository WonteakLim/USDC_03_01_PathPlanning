#pragma once
#include "../data_type/planning_data_type.h"
#include "../utils/spline.h"
#include <vector>

// ==================================
// class: discrete_trajectory
class discrete_trajectory{
    public:
	discrete_trajectory( double dt, std::vector<cartesian_state> path, double in_range = 2.0 );
	~discrete_trajectory() {};

	// cfg
    private:
	double threshold_valid_offset_ = 2.0;

    private:	
	bool valid_trajectory_ = false;
	double time_interval_ = -1.0;

	std::vector<cartesian_state> path_;

	int num_node_ = -1;

    public:
	bool IsValid(void);
	bool IsOnTrajectory( double pose_x, double pose_y );

	std::vector<double> FindClosestNode( double pose_x, double pose_y );
	int		    FindClosestNodeIdx(	double pose_x, double pose_y );

	cartesian_state	    GetNode( int idx );	

	inline int	    GetNumNode(){return num_node_; }
	inline double	    GetTimeInterval() { return time_interval_; }

    private:	
	double GetDistance( double x1, double y1, double x2, double y2 );
};

// ==================================
// class: StartSelector
class start_selector{
    public:
	start_selector() {}
	~start_selector() {}

    public:
    	sn_state SelectStartNode( 
		Map* map,
		double prev_path_dt,
		std::vector<cartesian_state> path,
		std::vector<double> ego_pose,	// x, y, yaw, spd_x, acc_x
		double lookahead );

    private:
	// =======================
	// Main algorithm
	sn_state GetSNState( Map* map, discrete_trajectory* trj, int idx );
	sn_state FindLookaheadNode( Map* map,
		discrete_trajectory* trj,
		double ego_x, double ego_y, double ego_yaw, double lookahead );	
	
	sn_state ConvXY2SN( Map* map, xy_state );


	// ========================
	// Configuration
    private:
	double in_range_bnd_ = 2.0;
    public:
	inline void SetCfg_InRangeDist( double dist ) { in_range_bnd_ = dist; }
};


