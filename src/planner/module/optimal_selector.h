#include "../data_type/planning_data_type.h"
#include "object_manager.h"

typedef std::vector<trajectory> trajectory_set;

class optimal_selector{
    public:
	optimal_selector();
	~optimal_selector();

    public:
	trajectory Optimization( candidate_p_set* s_candidate,
				 candidate_p_set* n_candidate,
				 double desired_spd,
				 planning_object::object_manager* objects,
				 trajectory_weight weight );

    private:
	trajectory_set SN2Trajectory(   candidate_p_set* s_candidate,
					candidate_p_set* n_candidate,
					double desired_spd,
					trajectory_weight weight );

};


