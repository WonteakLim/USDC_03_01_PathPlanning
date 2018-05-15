#include "../data_type/planning_data_type.h"
#include "../utils/spline.h"
#include "../utils/ini_parser/ini_parser.h"
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
				 double s_weight, double n_weight, double lane_weight,
				 planning_object::object_manager* objects,
				 int desired_lane,
				 trajectory& opt_trajectory);

    private:
	trajectory_set SN2Trajectory(   candidate_p_set* s_candidate,
					candidate_p_set* n_candidate,
					int desired_lane,
					double s_weight, double n_weight, double lane_weight);

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

    private:
	void print( trajectory* trj);

    private:
	CINI_Parser config_parser_;
	void ProcessINI();
};


