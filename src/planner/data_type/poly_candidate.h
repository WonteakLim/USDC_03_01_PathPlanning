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
	state desired_state_;
	double continuation_time_;
	double violate_dist_ = 0.0;

    private:
	polynomial poly_;

	// cost
    private:
	double weight_jerk_ = 0.0;
	double weight_con_time_ = 0.0;
	double weight_terminal_ = 0.0;
	double weight_violate_ = 0.0;

    public:
	inline void SetViolateDist(double dist) { violate_dist_ = dist; }
	void CalTotalCost( double w_jerk, double w_time, double w_terminal, double w_violate = 0.0 );
};

typedef std::vector<poly_candidate> poly_candidate_set;


