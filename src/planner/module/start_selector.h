#pragma once
#include "../data_type/planning_data_type.h"
#include "../utils/spline.h"
#include <vector>

struct xy_state{
    state x;
    state y;
};

struct sn_state{
    state s;
    state n;
};

// ==================================
// class: discrete_trajectory
class discrete_trajectory{
    public:
        discrete_trajectory( double dt, std::vector<double> path_x, std::vector<double> path_y );
	~discrete_trajectory() {};

    private:
	const double THRESHOLD_VALID_OFFSET = 2.0;
    private:	
	bool valid_trajectory_ = false;
	double time_interval_ = -1.0;

	std::vector<double> path_x_;
	std::vector<double> path_y_;

	int num_node_ = -1;

    public:
	bool IsValid(void);
	bool IsOnTrajectory( double pose_x, double pose_y );

	std::vector<double> FindClosestNode( double pose_x, double pose_y );
	int		    FindClosestNodeIdx(	double pose_x, double pose_y );

	xy_state	    GetNode( int idx );	
	inline int	    GetNumNode(){return num_node_; }
	inline double	    GetTimeInterval() { return time_interval_; }

    private:	
	std::vector<double> GetSpd( double dt,
		std::vector<double> s1, 
		std::vector<double> s2);
	std::vector<double> GetAcc( double dt,
		std::vector<double> s1, 
		std::vector<double> s2, 
		std::vector<double> s3);

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
		std::vector<double> prev_path_x, // x positions of nodes in a previous path
		std::vector<double> prev_path_y, // y positions of nodes in a previous path
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
};


