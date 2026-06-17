#include "OnlineTrajPlanner.h"
#include <math.h>
#include<iostream>
#include <cstring>
#include <utility.h>
using namespace std;
OnlineTrajPlanner::OnlineTrajPlanner(int n)
{
	this->dim = n;
	this->cnt = 0;
	this->K0 = 20;
	this->t = 0;
	this->T = 1; // ms
	this->method_flag = 0;
	for (int i = 0; i < n; i++)
	{
		vmax[i] = 3;
		amax[i] = 10;
		curP[i] = curV[i] = 0;
		tol[i] = 1e-6;
	}
	
}


void OnlineTrajPlanner::init(int T, double vmax[], double amax[], int n, int K0)
{
	this->dim = n;
	this->T = T;
	this->K0 = K0;
	memcpy(this->vmax, vmax, n * sizeof(double));
	memcpy(this->amax, amax, n * sizeof(double));
	for (int i = 0; i < n; i++)
		curP[i] = curV[i] = 0, tol[i] = 1e-6;
}





int OnlineTrajPlanner::generate(double s[], double e[], double speed, double acc, double v[], int method)
{
	cnt = 0;
	K = -1;
	method_flag = method;
	double a, b, t1, t2, d, tb, pv;
	for (int i = 0; i < dim; i++)
	{
		curV[i] = v ? v[i] : 0;
		curP[i] = from[i] = s[i];
		to[i] = e[i];
		int k = (int)ceil((fabs(e[i] - s[i]) / (T * vmax[i] * speed) * 1000));
		if (k > K)
			K = k;
	}
	if (K < K0)
		K = K0;
	t = K * T / 1000.0;
	//cout << K << endl;
	int violation = 0;
	for (int i = 0; i < dim; i++)
	{
		d = e[i] - s[i];
		
		if (fabs(d) > tol[i])
		{
			b = v ? v[i] / d : 0;// s �ĳ�ʼ�ٶ� ds0
			if (method == 1)
			{
				double delta = (2 * b * t - 4) * (2 * b * t - 4) + 4 * t * t * b * b;
				double amin = ((4 - 2 * b * t) + sqrt(delta)) / (2 * t * t);// feasible minimum a
				if (amin > fabs(amax[i] / d))
				{
					flags[i] = 2; // ���������ٶȣ����Բ�ֵ
					violation = 1;
					continue;
				}
				a = fabs(amax[i] * acc / d);// s �ĺ㶨���ٶ�
				if (amin > a)
					a = amin;

				double A = a, B = -(a * t + b), C = 1 + b * b / (2 * a);
				delta = B * B - 4 * A * C;
				tb = (-B - sqrt(delta)) / (2 * A);// ȡС�ĸ�
				pv = a * tb;
				t1 = (pv - b) / a;
				if (t1 < 0)
				{
					tb = (1 - b * b / (2 * a)) / (a * t - b);
					pv = a * tb;
					t1 = (pv - b) / -a;
					flags[i] = -1;
				}
				else
					flags[i] = 1;

				this->offset[i] = 0.5 * flags[i] * a * t1 * t1 + b * t1;
				t2 = t - tb;
				this->a[i] = a;
				this->b[i] = b;
				this->t1[i] = t1;
				this->t2[i] = t2;
				this->pv[i] = pv;
				this->tb[i] = tb;
			}
			else if (method == 2)// five order
			{
				/*a0[i] = 0;
				a1[i] = b;
				a2[i] = 3 / (t * t) - 2 / t * b;
				a3[i] = -2 / (t * t * t) + b / (t * t);*/
				a0[i] = 0;
				a1[i] = b;
				a2[i] = 0;
				a3[i] = (20 - (12 * b) * t) / (2 * t * t * t);
				a4[i] = (-30 + 16 * b * t) / (2 * t * t * t * t);
				a5[i] = (12 - 6 * b * t) / (2 * t * t * t * t *t);
				flags[i] = 1;
			}
			
		}
		else 
			flags[i] = 0; // ԭ�ز���
	}
	cnt = 0;
	return violation;
}

int OnlineTrajPlanner::generate(double e[], double speed, double acc)
{
	return generate(curP, e, speed, acc, curV);
}

int OnlineTrajPlanner::generate(double speed, double acc)
{
	double s[] = { 0 }, e[] = { 1 };
	return generate(s, e, speed, acc, nullptr);
}

int OnlineTrajPlanner::step(double p[], double v[], double ddp[], double tt)
{
	int flag = 1;
	if (tt > t)
	{
		flag = 0;
	}
	if (method_flag == 1)
	{
		for (int i = 0; i < dim; i++)
		{
			if (flags[i] == 0 || flag == 0)
				p[i] = to[i], v[i] = 0, ddp[i] = 0;
			else if (flags[i] == 2)
			{
				p[i] = tt / t * (to[i] - from[i]) + from[i];
				v[i] = (to[i] - from[i]) / t;
				if (tt < T / 1000)
					ddp[i] = v[i] / T * 1000;
				else if (fabs(tt - t) <= T/1000)
					ddp[i] = -v[i] / T * 1000;
				else
					ddp[i] = 0;

			}
			else
			{
				if (tt <= t1[i])
				{
					p[i] = 0.5 * flags[i] * a[i] * tt * tt + b[i] * tt;
					v[i] = flags[i] * a[i] * tt + b[i];
					ddp[i] = flags[i] * a[i];
				}
				else if (tt <= t2[i])
				{
					p[i] = offset[i] + (tt - t1[i]) * pv[i];
					v[i] = pv[i];
					ddp[i] = 0;
				}
				else
				{
					p[i] = 1 - 0.5 * a[i] * (t - tt) * (t - tt);
					v[i] = a[i] * (t - tt);
					ddp[i] = -a[i];
				}
				p[i] = (to[i] - from[i]) * p[i] + from[i];
				v[i] *= to[i] - from[i];
				ddp[i] *= to[i] - from[i];
			}
			curP[i] = p[i];
			curV[i] = v[i];
		}
	}
	else
	{
		for (int i = 0; i < dim; i++)
		{
			if (flags[i] == 0 || flag == 0)
				p[i] = to[i], v[i] = 0;
			else
			{
				p[i] = a0[i] + a1[i] * tt + a2[i] * tt * tt + a3[i] * tt * tt * tt + a4[i] * tt * tt * tt * tt + a5[i] * tt * tt * tt * tt *tt;
				v[i] = a1[i] + 2 * a2[i] * tt + 3 * a3[i] * tt * tt + 4 * a4[i] * tt * tt * tt + 5 * a5[i] * tt * tt * tt * tt;
				ddp[i] = 2 * a2[i] + 6 * a3[i] * tt + 12 * a4[i] * tt * tt + 20 * a5[i] * tt * tt * tt;
				p[i] = (to[i] - from[i]) * p[i] + from[i];
				v[i] *= to[i] - from[i];
				ddp[i] *= to[i] - from[i];
			}
			
			curP[i] = p[i];
			curV[i] = v[i];
		}

	}
	return flag;
}

int OnlineTrajPlanner::step(double p[], double v[], double ddp[])
{
	int flag = 1;
	if (cnt > K)
		flag = 0;
	double tt = cnt * T / 1000.0;
	if (method_flag == 1)
	{
		for (int i = 0; i < dim; i++)
		{
			if (flags[i] == 0 || flag == 0)
				p[i] = to[i], v[i] = 0, ddp[i] = 0;
			else if (flags[i] == 2)
			{
				p[i] = (double)cnt / K * (to[i] - from[i]) + from[i];
				v[i] = (to[i] - from[i]) / t;
				if (cnt == 0)
					ddp[i] = v[i] / T * 1000;
				else if (cnt == K)
					ddp[i] = -v[i] / T * 1000;
				else
					ddp[i] = 0;
			}
			else
			{
				if (tt <= t1[i])
				{
					p[i] = 0.5 * flags[i] * a[i] * tt * tt + b[i] * tt;
					v[i] = flags[i] * a[i] * tt + b[i];
					ddp[i] = flags[i] * a[i];
				}
				else if (tt <= t2[i])
				{
					p[i] = offset[i] + (tt - t1[i]) * pv[i];
					v[i] = pv[i];
					ddp[i] = 0;
				}
				else
				{
					p[i] = 1 - 0.5 * a[i] * (t - tt) * (t - tt);
					v[i] = a[i] * (t - tt);
					ddp[i] = -a[i];
				}
				p[i] = (to[i] - from[i]) * p[i] + from[i];
				v[i] *= to[i] - from[i];
				ddp[i] *= to[i] - from[i];
			}
			curP[i] = p[i];
			curV[i] = v[i];
		}
	}
	else
	{
		for (int i = 0; i < dim; i++)
		{
			if (flags[i] == 0 || flag == 0)
				p[i] = to[i], v[i] = 0;
			else
			{
				p[i] = a0[i] + a1[i] * tt + a2[i] * tt * tt + a3[i] * tt * tt * tt + a4[i] * tt * tt * tt * tt + a5[i] * tt * tt * tt * tt *tt;
				v[i] = a1[i] + 2 * a2[i] * tt + 3 * a3[i] * tt * tt + 4 * a4[i] * tt * tt * tt + 5 * a5[i] * tt * tt * tt * tt;
				ddp[i] = 2 * a2[i] + 6 * a3[i] * tt + 12 * a4[i] * tt * tt + 20 * a5[i] * tt * tt * tt;
				p[i] = (to[i] - from[i]) * p[i] + from[i];
				v[i] *= to[i] - from[i];
				ddp[i] *= to[i] - from[i];
			}
			
			curP[i] = p[i];
			curV[i] = v[i];
		}

	}
	
	cnt++;
	return flag;
}

CartesianTrajPlanner::CartesianTrajPlanner(double v, double a, double w, double alpha) : OnlineTrajPlanner(1)
{
	max_a = a;
	max_v = v;
	max_w = w;
	max_alpha = alpha;
}

int CartesianTrajPlanner::generateMotion(const double* Ts, const double* Te, double speed, double acc, int method)
{
	Eigen::Map<const Eigen::Matrix4d> T0(Ts);
	Eigen::Map<const Eigen::Matrix4d> T1(Te);

	Rs = T0.block<3, 3>(0, 0);
	Re = T1.block<3, 3>(0, 0);
	ps = T0.block<3, 1>(0, 3);
	pe = T1.block<3, 1>(0, 3);
	re = logR(Rs.transpose() * Re);
	Rsre = Rs * re;
	pse = pe - ps;
	double vmax[] = { std::min(max_v / pse.norm(), max_w / Rsre.norm()) };
	double amax[] = { std::min(max_a / pse.norm(), max_alpha / Rsre.norm()) };
	init(1, vmax, amax, 1);
	double s[] = { 0 }, e[] = { 1 };
	return generate(s, e, speed, acc, nullptr, method);
}

int CartesianTrajPlanner::step(double* T, double* V, double *dV)
{
	double p[1], v[1], a[1];
	Eigen::Map<Eigen::Matrix4d> Tt(T);
	Eigen::Map<Eigen::Vector6d> Vt(V);
	Eigen::Map<Eigen::Vector6d> dVt(dV);
	if (OnlineTrajPlanner::step(p, v, a))
	{
		Tt.block<3, 3>(0, 0) = Rs * exp_r(re * p[0]);
		Tt.block<3, 1>(0, 3) = ps + pse * p[0];
		Tt.block<1, 4>(3, 0) << 0, 0, 0, 1;
		Vt.head(3) = Rsre * v[0];
		Vt.tail(3) = pse * v[0];

		dVt.head(3) = Rsre * a[0];
		dVt.tail(3) = pse * a[0];

		return 1;
	}
	else
		return 0;
}

int CartesianTrajPlanner::step(double* T, double* V, double* dV, double tt)
{
	double p[1], v[1], a[1];
	Eigen::Map<Eigen::Matrix4d> Tt(T);
	Eigen::Map<Eigen::Vector6d> Vt(V);
	Eigen::Map<Eigen::Vector6d> dVt(dV);
	if (OnlineTrajPlanner::step(p, v, a, tt))
	{
		Tt.block<3, 3>(0, 0) = Rs * exp_r(re * p[0]);
		Tt.block<3, 1>(0, 3) = ps + pse * p[0];
		Tt.block<1, 4>(3, 0) << 0, 0, 0, 1;
		Vt.head(3) = Rsre * v[0];
		Vt.tail(3) = pse * v[0];
		dVt.head(3) = Rsre * a[0];
		dVt.tail(3) = pse * a[0];
		return 1;
	}
	else
		return 0;
}