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

#include "optimizer.hpp"

//Timerx timer;

/**
* The specific functor I'm interesting in.
* It defines the cost function and its Jacobian
* 
*/

struct l2costFunctor : public FunctorOpti
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

	// Data matrices
	const MatrixXd* X_left_p;
	const MatrixXd* LC;
	const MatrixXd* RC;
	Matrix4d dcardan[6];

	Matrix4d Tr;
	Matrix3d R;
	Vector3d T;

	MatrixXd X_left_c; 
	MatrixXd X_right_c;
	Matrix3d G;
	Matrix3d dR;
	
	//Calibration parameters
	double f, cu, cv, B;
	RowVector3d Baseline;

	int N;

	l2costFunctor() : FunctorOpti()
	{
		X_left_p = NULL;
		LC = NULL;
		RC = NULL;
	}

	int operator()(const VectorXd& x, VectorXd& fvec)
	{
		//Bring the point to the manifold via the retraction
		cardanExp(x, R, T);

		//transform the points from the previous left view
		//to the current left view and the current right view
		//X_left_c = zeros(size(X_left_p));
		//X_right_c = zeros(size(X_left_p));
		for(int i=0; i<N; i++)
		{
			X_left_c.row(i) = R*X_left_p->row(i).transpose() + T;
			X_right_c.row(i) = X_left_c.row(i) - Baseline;
		}

		//now let's compute the error
		for(int i=0; i<N; i++)
		{
			double expected_lc_u = f * X_left_c(i, 0)/X_left_c(i, 2) + cu;
			double expected_lc_v = f * X_left_c(i, 1)/X_left_c(i, 2) + cv;
			double expected_rc_u = f * X_right_c(i, 0)/X_left_c(i, 2) + cu;
			double expected_rc_v = f * X_left_c(i, 1)/X_left_c(i, 2) + cv;
			
			double weight = 1.0/(fabs(LC->operator()(i, 0) - cu)/fabs(cu) + 0.05);

			fvec((i*4))   = weight*(expected_lc_u - LC->operator()(i, 0));
			fvec((i*4)+1) = weight*(expected_lc_v - LC->operator()(i, 1));
			fvec((i*4)+2) = weight*(expected_rc_u - RC->operator()(i, 0));
			fvec((i*4)+3) = weight*(expected_rc_v - RC->operator()(i, 1));
		}

		return 0;
	}
 
	int df(const VectorXd& x, MatrixXd& Jac)
	{
		//derivative of the exponential map at x
		// x can be something different from 0! :D
		// at 0 the derivative is rather simple
		cardanExp(x, R, T);
		JCardan(x, dcardan);

		for(int k=0; k<3; k++)
		{
			dR = dcardan[k].block(0, 0, 3, 3);
			for(int i=0; i<N; i++)
			{
				if(k == 0)
				{
					X_left_c.row(i) = R*X_left_p->row(i).transpose() + T;
					X_right_c.row(i) = X_left_c.row(i) - Baseline;
				}

				Vector3d A = dR * X_left_p->row(i).transpose();
				double weight = 1.0/(fabs(LC->operator()(i, 0) - cu)/fabs(cu) + 0.05);
				double cc = weight * f / (X_left_c(i, 2)*X_left_c(i, 2));

				Jac((i*4), k) 	= cc * (A(0) * X_left_c(i, 2) - X_left_c(i, 0)  * A(2)); 
     				Jac((i*4)+1, k) = cc * (A(1) * X_left_c(i, 2) - X_left_c(i, 1)  * A(2));
     				Jac((i*4)+2, k) = cc * (A(0) * X_left_c(i, 2) - X_right_c(i, 0) * A(2));
     				Jac((i*4)+3, k) = cc * (A(1) * X_left_c(i, 2) - X_left_c(i, 1)  * A(2));
		
				Jac((i*4), k+3)   = cc * (G(k, 0) * X_left_c(i, 2) - X_left_c(i, 0)  * G(k, 2)); 
     				Jac((i*4)+1, k+3) = cc * (G(k, 1) * X_left_c(i, 2) - X_left_c(i, 1)  * G(k, 2));
     				Jac((i*4)+2, k+3) = cc * (G(k, 0) * X_left_c(i, 2) - X_right_c(i, 0) * G(k, 2));
     				Jac((i*4)+3, k+3) = cc * (G(k, 1) * X_left_c(i, 2) - X_left_c(i, 1)  * G(k, 2));
			}
		}
		
		return 0;
	}
 
	/**
	* This method assumes the following format
	  X_left_p : Nx3
	  LC : Nx2
	  RC : Nx2
	*/
	int setData(const MatrixXd& X_left_p, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib)
	{
		this->X_left_p = &X_left_p;
		this->LC = &LC;
		this->RC = &RC;
		this->f = calib.f;
		this->cu = calib.cu;
		this->cv = calib.cv;
		this->B = calib.B;
		this->Baseline = Vector3d(B, 0, 0);
		this->N = X_left_p.rows();
		
		X_left_c = MatrixXd(N, 3); 
		X_right_c = MatrixXd(N, 3);
		G = Matrix3d::Identity();
		return 0;
	}

	// size of the state
	int inputs() const { return 6; }

	// number of constraints
	int values() const { return 4*N; }
};

VectorXd run_opt(const MatrixXd& X_left_p, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib, VectorXd& seed, int maxits, double epsx, double epsg, double epsf)
{
	l2costFunctor functor;
	functor.setData(X_left_p, LC, RC, calib);
 
	LevenbergMarquardt<l2costFunctor, double> lm(functor);
	lm.parameters.ftol = epsf;
	lm.parameters.xtol = epsx;
	lm.parameters.gtol = epsg;
	lm.minimize(seed);

	return VectorXd(seed);
}
