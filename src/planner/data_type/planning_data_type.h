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
	double GetDesiredSpd() { return desired_spd_; }
	double GetDesiredDist() { return desired_dist_; }

    protected:
	void SetLaneIndex(int idx) { lane_index_ = idx; }
	void SetDesiredSpeed(double spd) {desired_spd_ = spd;}
	void SetDesiredDist(double dist) {desired_dist_ = dist;}

    protected:
	double max_spd_ = -1.0;
	double max_acc_ = DEFAULT_ACC;	
	double min_acc_ = DEFAULT_ACC;
	double max_jerk_ = -1.0;
	double integ_jerk_ = -1.0;
	int lane_index_ = -1;
	feasible_status is_feasible = UNKNOWN;

	double desired_spd_ = 0.0;
	double desired_dist_ = 0.0;

    private:
	double pre_weight_dist_ = 1.0;
	double pre_weight_spd_ = 1.0;
    public:
	inline double GetPreWeightDist() { return pre_weight_dist_;} 
	inline double GetPreWeightSpd() { return pre_weight_spd_; }
	inline void SetPreWeightDist() { pre_weight_dist_ = 1.0;}
	inline void SetPreWeightSpd() { pre_weight_spd_ = 1.0;}
	inline void ClearPreWeightDist() { pre_weight_dist_ = 0.0; }
	inline void ClearPreWeightSpd() { pre_weight_spd_ = 0.0; }

};

typedef std::vector<candidate> candidate_set;
typedef std::vector<candidate*> candidate_p_set;

struct trajectory_weight{
    double t;
    double s_comfort;
    double n_comfort;
    double s_desired_dist;
    double s_desired_spd;
};

class trajectory{
    public:
	trajectory() {}
	trajectory( int idx, 
		candidate* s_candidate, candidate* n_candidate,
		double desired_spd,
	       trajectory_weight weight	);
	~trajectory() {}

    private:
	candidate* s_trajectory_;
	candidate* n_trajectory_;

	int idx_=-1;
	double time_horizon_=-1.0;
	double cost_ = 0.0;
	double desired_spd_=0.0;

    private:
	double	    CalTimeHorizon( );
	double	    CalCost( trajectory_weight weight );

	// =========================
	// External interface
    public:
	inline int		    GetIdx() { return idx_; }
	inline double		    GetCost() { return cost_; }
	inline double		    GetTimeHorizon() { return time_horizon_; }
	inline double		    GetS_T() { return s_trajectory_->GetT(); }
	inline double		    GetS_DesiredSpd() { return s_trajectory_->GetDesiredSpd(); }
	inline double		    GetS_DesiredDist() { return s_trajectory_->GetDesiredDist(); }
	std::vector<state>	    GetNode(double t);
	std::vector<double>	    GetMaxSpd();
	int			    GetDiscretePathSN( double dt, double T,
					    std::vector<double>& path_s, std::vector<double>& path_n );

    public:
	bool operator < (const trajectory& trj ) const{
	    double cost_1 = this->cost_;
	    double cost_2 = trj.cost_;
	    return cost_1 < cost_2;
	}
};




