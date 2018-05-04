#pragma once
#include <vector>

#include "planning_data_type.h"
#include "poly.h"

class poly_candidate : public candidate{
    public:
	poly_candidate(int lane_idx, state start, state end, state desired, double T);
	~poly_candidate();

	// virtual function
    public:
	double GetMaxSpd();
	double GetMaxAcc();
	double GetMinAcc();
	double GetMaxJerk();
	double GetIntegSpd();
	double GetIntegJerk();
	double GetT();
	state	GetState( double t );

    private:
	state start_state_;
	state end_state_;
	double continuation_time_;

    private:
	polynomial poly_;
};

typedef std::vector<poly_candidate> poly_candidate_set;


