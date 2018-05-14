#include "../data_type/planning_data_type.h"
#include "../utils/spline.h"
#include "object_manager.h"

typedef std::vector<trajectory> trajectory_set;

class optimal_selector{
    public:
	optimal_selector();
	~optimal_selector();

    public:
	bool Optimization(	 Map* map,
				 double lookahead_time,
				 candidate_p_set* s_candidate,
				 candidate_p_set* n_candidate,
				 double s_weight, double n_weight,
				 planning_object::object_manager* objects,
				 trajectory& opt_trajectory);

    private:
	trajectory_set SN2Trajectory(   candidate_p_set* s_candidate,
					candidate_p_set* n_candidate,
					double s_weight, double n_weight);

	bool IsValidCurvature( Map* map, trajectory* p_trajectory, double t_resol, double T );

	// Configuration
    private:
	double collision_check_resol_ = 0.1;
	double collision_check_time_ = 3.0;
	double vehicle_length_ = 6.0;
	double vehicle_width_ = 3.0;
	double curvature_check_t_resol_ = 0.2;
	double curvature_check_T_ = 2.0;
	double curvature_limit_ = 0.167;

    public:
	inline void	SetCfgCol_TimeResol( double resol )	{ collision_check_resol_ = resol; }
	inline void	SetCfgCol_TimeRange( double T )		{ collision_check_time_ = T; }
	inline void	SetCfgCol_VehWidth( double width )	{ vehicle_width_ = width; }
	inline void	SetCfgCol_VehLength( double length )	{ vehicle_length_ = length; }
	inline void	SetCfgK_TimeResol( double resol )	{ curvature_check_t_resol_ = resol; }
        inline void	SetCfgK_TimeRange( double T )		{ curvature_check_T_ = T; }
	inline void	SetCfgK_Limit( double k )		{ curvature_limit_ = k; }

    private:
	void print( trajectory* trj);

};


