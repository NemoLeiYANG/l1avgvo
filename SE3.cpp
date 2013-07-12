/** Copyright 2013. All rights reserved.
* Author: German Ros (gros@cvc.uab.es)
*         Advanced Driver Assistance Systems (ADAS)
*         Computer Vision Center (CVC)
*	  Universitat Autònoma de Barcelona (UAB)
*
* This is free software; you can redistribute it and/or modify it under the
* terms of the GNU General Public License as published by the Free Software
* Foundation; either version 3 of the License, or any later version.
*
* This software is distributed in the hope that it will be useful, but WITHOUT ANY
* WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
* PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with
* this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
* Street, Fifth Floor, Boston, MA 02110-1301, USA 
*
*/

#include "SE3.hpp"

Matrix3d so3Exp(Vector3d V)
{
	double theta = V.norm();
	if(theta == 0.0)
		return Matrix3d::Identity();

	Matrix3d Omega;
	Omega << 0, -V(2)/theta, V(1)/theta, V(2)/theta, 0, -V(0)/theta, -V(1)/theta, V(0)/theta, 0;

	return Matrix3d::Identity() + sin(theta) * Omega + (1.0 - cos(theta)) * (Omega * Omega);
}

Matrix4d se3Exp(Vector6d V)
{
	Matrix4d S, SSq, SCub;
	S << 0, -V(2), V(1), V(3), V(2), 0, -V(0), V(4), -V(1), V(0), 0, V(5), 0, 0, 0, 0;

	double theta = V.head<3>().norm();
	double thetaSq = theta * theta;
	double thetaCub = thetaSq * theta;

	if(theta == 0)
		return Matrix4d::Identity() + S;
	else
	{
		SSq.noalias() = S * S;
		SCub.noalias() = SSq * S;

		return Matrix4d::Identity() + S + (((1.0 - cos(theta)) / thetaSq) * SSq) + (((theta - sin(theta)) / thetaCub) * SCub);
	}
}

void se3Exp(const Vector6d& V, Matrix4d& M)
{
	Matrix4d S, SSq, SCub;
	S << 0, -V(2), V(1), V(3), V(2), 0, -V(0), V(4), -V(1), V(0), 0, V(5), 0, 0, 0, 0;

	double theta = V.head<3>().norm();
	double thetaSq = theta * theta;
	double thetaCub = thetaSq * theta;

	if(theta == 0)
		M = Matrix4d::Identity() + S;
	else
	{
		SSq.noalias() = S * S;
		SCub.noalias() = SSq * S;

		M = Matrix4d::Identity() + S + (((1.0 - cos(theta)) / thetaSq) * SSq) + (((theta - sin(theta)) / thetaCub) * SCub);
	}
}

Vector3d so3Log(Matrix3d R)
{
	double sig = 0.5*(R.trace()-1.0);
	sig = (sig > 1.0)? 1.0 : sig;

	double theta = acos(sig);
	if(theta == 0.0)
		return Vector3d::Zero();

	double b = theta*(1.0/(2.0*sin(theta)));

	Vector3d V;
	V << b*( R(2, 1) - R(1, 2) ), b*( R(0, 2) - R(2, 0) ), b*( R(1, 0) - R(0, 1) );

	return V;
}

Vector6d se3Log(Matrix4d M)
{
	double epsilon = 1e-9;
	Matrix3d R = M.block<3, 3>(0, 0);
	Vector3d T = M.block<3, 1>(0, 3);
	
	Vector3d r = so3Log(R);
	double nn = r.norm();

	Matrix3d Omega;
	Omega << 0, -r(2), r(1), r(2), 0, -r(0), -r(1), r(0), 0;

	Vector3d t;
	if(nn < epsilon)
		t = T;
	else
	{
		Matrix3d Vinv = Matrix3d::Identity() - 0.5*Omega +  ((2.0*sin(nn) - nn - nn*cos(nn))/(2.0*nn*nn*sin(nn)))*Omega*Omega;
		t = (Vinv*T);
	}

	Vector6d V;
	V.head<3>() = r;
	V.tail<3>() = t;
	
	return V;
}



/**
* It returns an array of 6 4x4 matrices (a tensor in matrix form)
*/
Matrix4d* JExpSE3(Vector6d V, int n)
{
	Matrix4d* derivative = new Matrix4d[6];

	double da1[] = {	0, 0, 0, 0, 
				0, 0, 1, 0,  
				0, -1, 0, 0, 
				0, 0, 0, 0
			};

	double da2[] = {	0, 0, -1, 0,
				0, 0, 0, 0,
				1, 0, 0, 0,
				0, 0, 0, 0
			};

	double da3[] = {	0, 1, 0, 0,
				-1, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0
			};

	double da4[] = {	0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0,
				1, 0, 0, 0
			};

	double da5[] = {	0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 1, 0, 0
			};

	double da6[] = {	0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 1, 0
			};

	//basis for the derivative
	Matrix4d DA1(da1);
	Matrix4d DA2(da2);
	Matrix4d DA3(da3);
	Matrix4d DA4(da4);
	Matrix4d DA5(da5);
	Matrix4d DA6(da6);

	Matrix4d A, Apower;
	Matrix4d Je1, Je2, Je3, Je4, Je5, Je6;
	Matrix4d DAcc1, DAcc2, DAcc3, DAcc4, DAcc5, DAcc6;
	double fact = 1;

	// A = hat(V)
	A << 0, -V(2), V(1), V(3), V(2), 0, -V(0), V(4), -V(1), V(0), 0, V(5), 0, 0, 0, 0;

	DAcc1 = DA1;
	DAcc2 = DA2;
	DAcc3 = DA3;
	DAcc4 = DA4;
	DAcc5 = DA5;
	DAcc6 = DA6;
	Apower = A;

	Je1 = DA1;
	Je2 = DA2;
	Je3 = DA3;
	Je4 = DA4;
	Je5 = DA5;
	Je6 = DA6;	

	for(int i=2; i<=n; i++)
	{
		//i-th power (wrt a,b,c)
		DAcc1 = DA1*Apower + A*DAcc1;
		DAcc2 = DA2*Apower + A*DAcc2;
		DAcc3 = DA3*Apower + A*DAcc3;
		DAcc4 = A*DAcc4;
		DAcc5 = A*DAcc5;
		DAcc6 = A*DAcc6;

		fact *= i;

		Je1 = Je1 + (DAcc1 / fact);
		Je2 = Je2 + (DAcc2 / fact);
		Je3 = Je3 + (DAcc3 / fact);
		Je4 = Je4 + (DAcc4 / fact);
		Je5 = Je5 + (DAcc5 / fact);
		Je6 = Je6 + (DAcc6 / fact);

		if(i < n)
			Apower = Apower * A;
	}

	derivative[0] = Je1;
	derivative[1] = Je2;
	derivative[2] = Je3;
	derivative[3] = Je4;
	derivative[4] = Je5;
	derivative[5] = Je6;

	return derivative;
}

Matrix4d L2averaging(const vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> > models, double epsilon, int maxiters)
{
	int N = models.size();
	bool done = false;

	Matrix4d Tr = models[0];
	Matrix4d invTr = Matrix4d::Identity();
	Matrix3d R;
	Matrix3d Rt;
	Vector3d T;
	
	int iters = 0;
	Vector6d r, vi;
	while( !done && (iters < maxiters) )
	{
		R = Tr.block<3, 3>(0, 0);
		T = Tr.block<3, 1>(0, 3);
		r.setZero();
		for(int i=0; i<N; i++)
		{
			//vi = se3Log(Tr.inverse()*models[i]);
			InverseSE3(R, Rt, T, invTr);
			vi = se3Log(invTr*models[i]);
			r.noalias() += vi;
		}

		r = r / double(N);

		if(r.norm() < epsilon)
			done = true;
		
		Tr = Tr * se3Exp(r);

		iters++;
	}

	return Tr;
}

void InverseSE3(const Matrix3d& R, Matrix3d& Rt, const Vector3d& T, Matrix4d& invTr)
{
	Rt = R.transpose();
	invTr.block<3, 3>(0, 0) = Rt;
	invTr.block<3, 1>(0, 3) = -Rt*T;
}

Matrix4d L1averaging(const vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> > models, double epsilon, int maxiters)
{
	Matrix4d Tr = L2averaging(models, epsilon, maxiters);
	//Matrix4d Tr = models[0];
	Matrix4d invTr = Matrix4d::Identity();
	Matrix3d R;
	Matrix3d Rt;
	Vector3d T;

	int N = models.size();
	bool done = false;

	int iters = 0;
	Vector6d r, vi, delta;
	double u;

	long unsigned int debug = 0;
	while( !done && (iters < maxiters) )
	{
		R = Tr.block<3, 3>(0, 0);
		T = Tr.block<3, 1>(0, 3);
		r.setZero();
		u = 0.0;
		for(int i=0; i<N; i++)
		{
			//vi = se3Log(Mi*Tr.inverse());
			InverseSE3(R, Rt, T, invTr);
			vi = se3Log(models[i]*invTr);
			
			double norm_vi = vi.norm();
			if(norm_vi != 0.0)
			{
				u += (1.0 / norm_vi);
				r.noalias() += (vi / norm_vi);

			}
		}

		delta = r / u;

		if(delta.norm() < epsilon)
			done = true;
			
		Tr = se3Exp(delta)*Tr;
		
		iters++;
	}

	return Tr;
}

