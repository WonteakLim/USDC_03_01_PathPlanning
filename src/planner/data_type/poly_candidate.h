#pragma once
#include <vector>

#include "planning_data_type.h"
#include "poly.h"

class poly_candidate : public candidate{
    public:
	poly_candidate(state start, state end, double T);
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



