#ifndef ONLINE_TRAJ_PLANNER_H
#define ONLINE_TRAJ_PLANNER_H
#define R2D (180 / 3.1415926)
#define D2R (3.1415926 / 180)
#include <utility.h>

class OnlineTrajPlanner
{
public:
	OnlineTrajPlanner(int n = 1);
	void init(int T, double vmax[], double amax[], int n, int K0 = 20);
	int generate(double s[], double e[], double speed = 0.5, double acc = 0.1, double v[] = nullptr, int method = 1);
	int generate(double e[], double speed = 0.5, double acc = 0.1);
	int generate(double speed = 0.5, double acc = 0.1);
	int step(double p[], double v[], double ddp[]);
	int step(double p[], double v[], double ddp[], double tt);
protected:
	double vmax[20];
	double amax[20];
	int T; // ms
	int K;
	int K0;
	double t;

	double a0[20];
	double a1[20];
	double a2[20];
	double a3[20];
	double a4[20];
	double a5[20];

	double t1[20];
	double t2[20];
	double a[20];
	double b[20];
	double tb[20];
	double pv[20];
	double offset[20];
	double from[20];
	double to[20];
	int flags[20];

	double curP[20];
	double curV[20];
	double tol[20];
	
	int dim;
	int cnt;
	int method_flag;
};


class CartesianTrajPlanner : public OnlineTrajPlanner
{
public:
	CartesianTrajPlanner(double v, double a, double w, double alpha);
	int generateMotion(const double* Ts, const double* Te, double speed = 0.1, double acc = 0.1, int method = 1);
	int step(double* T, double* V, double *dV);
	int step(double* T, double* V, double *dV, double tt);
protected:
	Eigen::Vector3d ps, pe, re, pse, Rsre;
	Eigen::Matrix3d Rs, Re;
	double max_v, max_w, max_a, max_alpha;
};

#endif

