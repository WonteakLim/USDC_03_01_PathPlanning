#pragma once
#include <vector>

#define DEFAULT_ACC -1000.0

typedef std::vector<double> state;

struct cartesian_state{
    double x;
    double y;
    double yaw;
    double k;
    double spd;
    double acc;
};

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

enum maneuver_type{
    STOP = 0,
    FOLLOW,
    KEEPING
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

    protected:
	double cost_ = 0.0;
	maneuver_type maneuver_;

    public:
	inline double	GetCost() { return cost_; }
	inline maneuver_type GetManeuver() { return maneuver_; }
};

typedef std::vector<candidate> candidate_set;
typedef std::vector<candidate*> candidate_p_set;

class trajectory{
    public:
	trajectory() {}
	trajectory( int idx, 
		candidate* s_candidate, candidate* n_candidate,
		int desired_lane,
	        double s_weight, double n_weight, double lane_weight	);
	~trajectory() {}

    private:
	candidate* s_trajectory_;
	candidate* n_trajectory_;

	int idx_=-1;
	double time_horizon_=-1.0;
	double cost_ = 0.0;

	int lane_idx_ = -1;
	int desired_lane_idx_ = -1;

	double speed_limit = 21.0;

    private:
	double	    CalTimeHorizon( );
	double	    CalCost( double s_weight, double n_weight, double lane_weight );

	// =========================
	// External interface
    public:
	inline int		    GetIdx() { return idx_; }
	inline double		    GetCost() { return cost_; }
	inline double		    GetLaneIdx() { return lane_idx_; }
	inline double		    GetTimeHorizon() { return time_horizon_; }
	//inline double		    GetS_T() { return s_trajectory_->GetT(); }
	sn_state		    GetNode(double t);
	std::vector<double>	    GetMaxSpd();
	int			    GetDiscretePathSN( double dt, double T,
					    std::vector<double>& path_s, std::vector<double>& path_n );

	candidate*		    GetpTrajectoryS() { return s_trajectory_; }
	candidate*		    GetpTrajectoryN() { return n_trajectory_; }

    public:
	bool operator < (const trajectory& trj ) const{
	    double cost_1 = this->cost_;
	    double cost_2 = trj.cost_;
	    return cost_1 < cost_2;
	}
};




