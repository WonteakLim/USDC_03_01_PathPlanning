#include "poly.h"

Eigen::VectorXd solve_poly(vector<double> state_0,
							vector<double> state_1,
							double T)
{
	double coeff0 = state_0[0];
	double coeff1 = state_0[1];
	double coeff2 = state_0[2] * 0.5;

	double t = T;
	double t2 = t*t;
	double t3 = t*t2;

	Eigen::VectorXd result(6);

	
	// Quintic polynomial
	if (state_1.size() == 3)
	{
		double t4 = t*t3;
		double t5 = t*t4;

		Eigen::Matrix3d A;
		A << t3, t4, t5,
			3 * t2, 4 * t3, 5 * t4,
			6 * t, 12 * t2, 20 * t3;

		Eigen::Vector3d b;
		b << state_1[0] - (coeff0 + coeff1*t + coeff2*t2),
			state_1[1] - (coeff1 + state_0[2] * t),
			state_1[2] - state_0[2];

		auto Ai = A.inverse();
		auto coeffs = Ai*b;

		result <<
			coeff0,
			coeff1,
			coeff2,
			coeffs[0],
			coeffs[1],
			coeffs[2];
	}

	return result;
}

Eigen::VectorXd keep_state_poly(vector<double> state){
	Eigen::VectorXd result(6);
	result <<	state[0],
				state[1],
				state[2] * 0.5,
				0,
				0,
				0;

	return result;
}

double			poly_eval(const Eigen::VectorXd &coeffs, double x)
{
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

vector<double>	poly_eval_v(const Eigen::VectorXd &coeffs, double x, int up_to_diff)
{
	double f = 0.0;
	double df = 0.0;
	double ddf = 0.0;
	double dddf = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		f += coeffs[i] * pow(x, i);
		if ((up_to_diff > 0) && (i > 0)) df += i*coeffs[i] * pow(x, i - 1);
		if ((up_to_diff > 1) && (i > 1)) ddf += (i - 1)*i*coeffs[i] * pow(x, i - 2);
		if ((up_to_diff > 2) && (i > 2)) dddf += (i - 2)*(i - 1)*i*coeffs[i] * pow(x, i - 3);
	}
	vector<double> result;
	result.push_back(f);
	if (up_to_diff > 0) result.push_back(df);
	if (up_to_diff > 1) result.push_back(ddf);
	if (up_to_diff > 2) result.push_back(dddf);
	return result;
}

double			poly_max(const Eigen::VectorXd &coeffs, vector<double> points)
{
	double max_v = 0;
	for (double t : points){
		double v = fabs(poly_eval(coeffs, t));
		if (v > max_v) max_v = v;
	}
	return max_v;
}

double			poly_max_sign(const Eigen::VectorXd &coeffs, vector<double> points)
{
	double max_v = 0;
	for (double t : points){
		double v = poly_eval(coeffs, t);
		if (v > max_v) max_v = v;
	}
	return max_v;
}

double			poly_min_sign(const Eigen::VectorXd &coeffs, vector<double> points)
{
	double min_v = 0;
	for (double t : points){
		double v = poly_eval(coeffs, t);
		if (v < min_v) min_v = v;
	}
	return min_v;
}

double			poly_max_acceleration(const Eigen::VectorXd &coeffs, double t0, double t1)
{
	vector<double> time_points = { t0, t1 };
	if (not_zero(coeffs[5])){
		double a = 60 * coeffs[5];
		double b = 24 * coeffs[4];
		double c = 6 * coeffs[3];

		double d = b*b - 4 * a*c;
		if (d >= 0.0){
			b = -0.5*b / a;
			d = 0.5*sqrt(d) / a;

			double jerk_zero1 = b - d;
			if (between(jerk_zero1, t0, t1)){
				time_points.push_back(jerk_zero1);
			}

			double jerk_zero2 = b + d;
			if (between(jerk_zero2, t0, t1)){
				time_points.push_back(jerk_zero2);
			}
		}
	}
	else if (not_zero(coeffs[4])){
		double jerk_zero = -coeffs[3] / (4 * coeffs[4]);
		if (between(jerk_zero, t0, t1)){
			time_points.push_back(jerk_zero);
		}
	}

	Eigen::Vector4d a_coeffs;
	a_coeffs <<
		2 * coeffs[2]
		, 6 * coeffs[3]
		, 12 * coeffs[4]
		, 20 * coeffs[5];

	return poly_max(a_coeffs, time_points);
}

double poly_max_acc_signed( const Eigen::VectorXd &coeffs, double t0, double t1){
	vector<double> time_points = { t0, t1 };
	if (not_zero(coeffs[5])){
		double a = 60 * coeffs[5];
		double b = 24 * coeffs[4];
		double c = 6 * coeffs[3];

		double d = b*b - 4 * a*c;
		if (d >= 0.0){
			b = -0.5*b / a;
			d = 0.5*sqrt(d) / a;

			double jerk_zero1 = b - d;
			if (between(jerk_zero1, t0, t1)){
				time_points.push_back(jerk_zero1);
			}

			double jerk_zero2 = b + d;
			if (between(jerk_zero2, t0, t1)){
				time_points.push_back(jerk_zero2);
			}
		}
	}
	else if (not_zero(coeffs[4])){
		double jerk_zero = -coeffs[3] / (4 * coeffs[4]);
		if (between(jerk_zero, t0, t1)){
			time_points.push_back(jerk_zero);
		}
	}

	Eigen::Vector4d a_coeffs;
	a_coeffs <<
		2 * coeffs[2]
		, 6 * coeffs[3]
		, 12 * coeffs[4]
		, 20 * coeffs[5];

	return poly_max_sign(a_coeffs, time_points);
}

double poly_min_acc_signed( const Eigen::VectorXd &coeffs, double t0, double t1){
	vector<double> time_points = { t0, t1 };
	if (not_zero(coeffs[5])){
		double a = 60 * coeffs[5];
		double b = 24 * coeffs[4];
		double c = 6 * coeffs[3];

		double d = b*b - 4 * a*c;
		if (d >= 0.0){
			b = -0.5*b / a;
			d = 0.5*sqrt(d) / a;

			double jerk_zero1 = b - d;
			if (between(jerk_zero1, t0, t1)){
				time_points.push_back(jerk_zero1);
			}

			double jerk_zero2 = b + d;
			if (between(jerk_zero2, t0, t1)){
				time_points.push_back(jerk_zero2);
			}
		}
	}
	else if (not_zero(coeffs[4])){
		double jerk_zero = -coeffs[3] / (4 * coeffs[4]);
		if (between(jerk_zero, t0, t1)){
			time_points.push_back(jerk_zero);
		}
	}

	Eigen::Vector4d a_coeffs;
	a_coeffs <<
		2 * coeffs[2]
		, 6 * coeffs[3]
		, 12 * coeffs[4]
		, 20 * coeffs[5];

	return poly_min_sign(a_coeffs, time_points);
}

double			poly_max_jerk(const Eigen::VectorXd &coeffs, double t0, double t1)
{
	vector<double> time_points = { t0, t1 };
	if (not_zero(coeffs[5])){
		double snap_zero_time = -coeffs[4] / (5 * coeffs[5]);
		if (between(snap_zero_time, t0, t1)){
			time_points.push_back(snap_zero_time);
		}
	}
	Eigen::Vector3d jerk_coeffs;
	jerk_coeffs <<
		6 * coeffs[3]
		, 24 * coeffs[4]
		, 60 * coeffs[5];

	return poly_max(jerk_coeffs, time_points);
}
double			poly_jerk_integral(const Eigen::VectorXd &coeffs, double t)
{
	double coeff3 = coeffs[3];
	double coeff4 = coeffs[4];
	double coeff5 = coeffs[5];
	double t2 = t  * t;
	double t3 = t2 * t;
	double t4 = t2 * t2;
	double t5 = t2 * t3;

	double result =
		36 * coeff3 * coeff3 * t
		+ 144 * coeff3 * coeff4 * t2
		+ 192 * coeff4 * coeff4 * t3
		+ 240 * coeff3 * coeff5 * t3
		+ 720 * coeff4 * coeff5 * t4
		+ 720 * coeff5 * coeff5 * t5;

	return result;
}

double			poly_jerk_integral(const Eigen::VectorXd &coeffs, double t0, double t1)
{
	return poly_jerk_integral(coeffs, t1) - poly_jerk_integral(coeffs, t0);
}
