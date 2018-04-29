#pragma once
#include <vector>

#define DEFAULT_ACC -1000.0

typedef std::vector<double> state;

struct xy_state{
    state x;
    state y;
};

struct sn_state{
    state s;
    state n;
};

enum feasible_status{
    UNKNOWN = 0,
    FEASIBLE,
    INFEASIBLE
};

class candidate{
    public:
	candidate() {}
	~candidate() {}

	// ==============================
	// External interface
	// =============================

	// virtual functions
    public:
	virtual double GetMaxSpd() {return 0.0;}
	virtual double GetMaxAcc() {return 0.0;}
	virtual double GetMinAcc() {return 0.0;}
	virtual double GetMaxJerk() {return 0.0;}
	virtual double GetIntegJerk() {return 0.0;}
	virtual double GetT() {return 0.0;}
	virtual state GetState(double t) { return {0.0, 0.0, 0.0};}

	// predefined functions
    public:
	int GetLaneIndex() { return lane_index_; }

    protected:
	void SetLaneIndex(int idx) { lane_index_ = idx; }

    protected:
	double max_spd_ = -1.0;
	double max_acc_ = DEFAULT_ACC;	
	double min_acc_ = DEFAULT_ACC;
	double max_jerk_ = -1.0;
	double integ_jerk_ = -1.0;
	int lane_index_ = -1;
	feasible_status is_feasible = UNKNOWN;
};

struct trajectory_weight{

};

class trajectory{
    public:
	trajectory( candidate s_candidate, candidate n_candidate,
	       trajectory_weight weight	);
	~trajectory() {}

    private:
	candidate s_trajectory_;
	candidate n_trajectory_;

	double time_horizon_=-1.0;
	double cost_ = 0.0;

    private:
	double	    CalTimeHorizon( candidate s, candidate n );
	double	    CalCost( trajectory_weight weight );

	// =========================
	// External interface
    public:
	inline double		    GetCost() { return cost_; }
	inline double		    GetTimeHorizon() { return time_horizon_; }
	std::vector<state>	    GetNode(double t);
};




