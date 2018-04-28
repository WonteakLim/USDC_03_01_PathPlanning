#pragma once

#include <algorithm>
#include <cassert>
#include <vector>
#include <utility>

#include "../../Eigen-3.3/Eigen/Core"
#include "../../Eigen-3.3/Eigen/Dense"

using namespace std;

typedef Eigen::VectorXd polynomial;
Eigen::VectorXd		solve_poly( vector<double> state_0,
				    vector<double> state_1,
				    double T);
Eigen::VectorXd		keep_state_poly(vector<double> state);

double			poly_eval(const Eigen::VectorXd &coeffs, double x);
vector<double>		poly_eval_v(const Eigen::VectorXd &coeffs, double x, int up_to_diff = 2);
double			poly_max(const Eigen::VectorXd &coeffs, vector<double> points);
double			poly_max_sign(const Eigen::VectorXd &coeffs, vector<double> points);
double			poly_min_sign(const Eigen::VectorXd &coeffs, vector<double> points);

double			poly_max_acc_signed(const Eigen::VectorXd &coeffs, double t0, double t1);
double			poly_min_acc_signed( const Eigen::VectorXd &coeffs, double t0, double t1);
double			poly_max_jerk(const Eigen::VectorXd &coeffs, double t0, double t1);
double			poly_jerk_integral(const Eigen::VectorXd &coeffs, double t);
double			poly_jerk_integral(const Eigen::VectorXd &coeffs, double t0, double t1);

inline bool between( double test, double x0, double x1 ){
    return (test>x0 && test < x1 );
}

inline bool not_zero( double test ){
    return fabs(test) > 0.00001;
}
