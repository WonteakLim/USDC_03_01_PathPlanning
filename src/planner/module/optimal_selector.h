#include "../data_type/planning_data_type.h"
#include "object_manager.h"

typedef std::vector<trajectory> trajectory_set;

class optimal_selector{
    public:
	optimal_selector();
	~optimal_selector();

    public:
	bool Optimization( double lookahead_time,
				 candidate_p_set* s_candidate,
				 candidate_p_set* n_candidate,
				 double s_weight, double n_weight,
				 planning_object::object_manager* objects,
				 trajectory& opt_trajectory);

    private:
	trajectory_set SN2Trajectory(   candidate_p_set* s_candidate,
					candidate_p_set* n_candidate,
					double s_weight, double n_weight);

	// Collision cehck
    private:
	double collision_check_resol_ = 0.1;
	double collision_check_time_ = 3.0;
	double vehicle_length_ = 6.0;
	double vehicle_width_ = 3.0;

    private:
	void print( trajectory* trj);

};


