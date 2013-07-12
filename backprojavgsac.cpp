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

#include "backprojavgsac.hpp"

void triangulate(const MatrixXd& LP, const MatrixXd& RP, const Calibration& calib, MatrixXd& PTS)
{
	double f = calib.f;
	double cu = calib.cu;
	double cv = calib.cv;
	double B = calib.B;

	int N = LP.rows();
	for(int i=0; i<N; i++)
	{
		double d = max(LP(i, 0) - RP(i, 0), 1.0);
		double Bd = B / d;
		PTS(i, 0) = ( LP(i, 0) - cu ) * Bd;
		PTS(i, 1) = ( LP(i, 1) - cv ) * Bd;
		PTS(i, 2) = f * Bd;
	}
}

void getSamples(int N, int M, int current_it, vector<int>& indices)
{
	//const int maxSize = min(2.0 * log(current_it+1) / log(2.0), double(N));
	const int maxSize = min(4.0 * log(4.0*double(current_it+1)) / log(2.0), double(N));
	int P = max(M, maxSize-1);

	// 33.3% of the time
	if( (rand() % 3) == 0 )
		P = N;

	//I'm going to take advantage of the fact that
	// M <<< N to check that the M selected samples are different
	for (int i=0; i<M; i++)
	{
		int j = rand() % P;
		
		//check if j is already in indices
		bool repeat = false;
		for(int k=0; k<i; k++)
			if(indices[k] == j)
				repeat = true;
		if(repeat)
			continue;

		indices[i] = j;
  	}
}

void getSamples(int N, int M, vector<int>& indices)
{
	//I'm going to take advantage of the fact that
	// M <<< N to check that the M selected samples are different
	for (int i=0; i<M; i++)
	{
		int j = rand() % N;
		
		//check if j is already in indices
		bool repeat = false;
		for(int k=0; k<i; k++)
			if(indices[k] == j)
				repeat = true;
		if(repeat)
			continue;

		indices[i] = j;
  	}
}

bool comparator(pair<double, int> l, pair<double, int> r) { return l.first < r.first; }

vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> > selectData(vector< pair<double, int> >& residuals, const vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> >& models, int N)
{
	vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> > best_models;

	sort(residuals.begin(), residuals.end(), comparator);
	for(int i=0; i<N; i++)
	{
		best_models.push_back(models[residuals[i].second]);
		//cout << ">>> res = " << residuals[i].first << endl;
	}

	return best_models;
}

void selectData(const vector<int>& indices, const MatrixXd& In_X_left_p, const MatrixXd& In_LC, const MatrixXd& In_RC, MatrixXd& Out_X_left_p, MatrixXd& Out_LC, MatrixXd& Out_RC)
{
	for(int i=0; i<indices.size(); i++)
	{
		Out_X_left_p.row(i) = In_X_left_p.row(indices[i]);
		Out_LC.row(i) = In_LC.row(indices[i]);
		Out_RC.row(i) = In_RC.row(indices[i]);
	}	
}


inline double evalSolution(const Matrix4d& Tr, const MatrixXd& RMM_l, const MatrixXd& RMM_r)
{
	Vector13d X;
	X.segment<3>(0) = Tr.block<1, 3>(0, 0);
	X.segment<3>(3) = Tr.block<1, 3>(1, 0);
	X.segment<3>(6) = Tr.block<1, 3>(2, 0);
	X.segment<3>(9) = Tr.block<3, 1>(0, 3);
	X(12) = 1.0;

	return fabs(X.transpose()*RMM_l*X) + fabs(X.transpose()*RMM_r*X);
}

vector<int> evalSolution(const Matrix4d& Tr, const MatrixXd& X_left_p, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib, double threshold)
{
		Matrix3d R = Tr.block<3, 3>(0, 0);
		Vector3d T = Tr.block<3, 1>(0, 3);

		double f = calib.f;
		double cu = calib.cu;
		double cv = calib.cv;
		double B = calib.B;

		RowVector3d Baseline(B, 0, 0);

		int N = X_left_p.rows();

		MatrixXd X_left_c(N, 3);
		MatrixXd X_right_c(N, 3);

		//transform the points from the previous left view
		//to the current left view and the current right view
		for(int i=0; i<N; i++)
		{
			X_left_c.row(i) = R*X_left_p.row(i).transpose() + T;
			X_right_c.row(i) = X_left_c.row(i) - Baseline;
		}

		//now let's compute the error
		vector<int> inliers;
		for(int i=0; i<N; i++)
		{
			double expected_lc_u = f * X_left_c(i, 0)/X_left_c(i, 2) + cu;
			double expected_lc_v = f * X_left_c(i, 1)/X_left_c(i, 2) + cv;
			double expected_rc_u = f * X_right_c(i, 0)/X_left_c(i, 2) + cu;
			double expected_rc_v = f * X_left_c(i, 1)/X_left_c(i, 2) + cv;

			double e1 = (expected_lc_u - LC(i, 0));
			double e2 = (expected_lc_v - LC(i, 1));
			double e3 = (expected_rc_u - RC(i, 0));
			double e4 = (expected_rc_v - RC(i, 1));

			double error = sqrt(e1*e1 + e2*e2 + e3*e3 + e4*e4);
			if(error < threshold)
				inliers.push_back(i);
		}

		return inliers;	
}


void createRMMatrix(const MatrixXd& X_left_p, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib, MatrixXd& RMM_l, MatrixXd& RMM_r, const Options& localopts)
{
	const int N = X_left_p.rows();
	MatrixXd W_left(13, 3*N);
	MatrixXd W_right(13, 3*N);

	double f = calib.f;
	double cu = calib.cu;
	double cv = calib.cv;
	double B = calib.B;

	for(int i=0; i<N; i++)
	{
		double X = X_left_p(i, 0);
		double Y = X_left_p(i, 1);
		double Z = X_left_p(i, 2);

		double u1 = LC(i, 0);
		double v1 = LC(i, 1);

		double u2 = RC(i, 0);
		double v2 = RC(i, 1);

		W_left(0, 3*i) = 0;
		W_left(1,  3*i) = 0;
		W_left(2,  3*i) = 0;
		W_left(3,  3*i) = f*X;
		W_left(4,  3*i) = f*Y;
		W_left(5,  3*i) = f*Z;
		W_left(6,  3*i) =  (cv-v1)*X;
		W_left(7,  3*i) =  (cv-v1)*Y;
		W_left(8,  3*i) =  (cv-v1)*Z;
		W_left(9,  3*i) = 0;
		W_left(10,  3*i) = f;
		W_left(11,  3*i) = cu-v1;
		W_left(12,  3*i) = 0;

		W_left(0,3*i+1) = -f*X;
		W_left(1,3*i+1) = -f*Y;
		W_left(2,3*i+1) = -f*Z;
		W_left(3,3*i+1) = 0;
		W_left(4,3*i+1) = 0;
		W_left(5,3*i+1) = 0;
		W_left(6,3*i+1) =  (u1-cu)*X;
		W_left(7,3*i+1) =  (u1-cu)*Y;
		W_left(8,3*i+1) =  (u1-cu)*Z;
		W_left(9,3*i+1) = -f;
		W_left(10,3*i+1) = 0;
		W_left(11,3*i+1) = u1-cu;
		W_left(12,3*i+1) = 0;

		W_left(0,3*i+2) = f*v1*X;
		W_left(1,3*i+2) = f*v1*Y;
		W_left(2,3*i+2) = f*v1*Z;
		W_left(3,3*i+2) = -f*u1*X;
		W_left(4,3*i+2) = -f*u1*Y;
		W_left(5,3*i+2) = -f*u1*Z;
		W_left(6,3*i+2) =  (cu*v1-cv*u1)*X;
		W_left(7,3*i+2) =  (cu*v1-cv*u1)*Y;
		W_left(8,3*i+2) =  (cu*v1-cv*u1)*Z;
		W_left(9,3*i+2) = f*v1;
		W_left(10,3*i+2) = -f*u1;
		W_left(11,3*i+2) = -cv*u1+cu*v1;
		W_left(12,3*i+2) = 0;

		W_right(0, 3*i) = 0;
		W_right(1,  3*i) = 0;
		W_right(2,  3*i) = 0;
		W_right(3,  3*i) = f*X;
		W_right(4,  3*i) = f*Y;
		W_right(5,  3*i) = f*Z;
		W_right(6,  3*i) =  (cv-v2)*X;
		W_right(7,  3*i) =  (cv-v2)*Y;
		W_right(8,  3*i) =  (cv-v2)*Z;
		W_right(9,  3*i) = 0;
		W_right(10,  3*i) = f;
		W_right(11,  3*i) = cu-v2;
		W_right(12,  3*i) = 0;

		W_right(0,3*i+1) = -f*X;
		W_right(1,3*i+1) = -f*Y;
		W_right(2,3*i+1) = -f*Z;
		W_right(3,3*i+1) = 0;
		W_right(4,3*i+1) = 0;
		W_right(5,3*i+1) = 0;
		W_right(6,3*i+1) =  (u2-cu)*X;
		W_right(7,3*i+1) =  (u2-cu)*Y;
		W_right(8,3*i+1) =  (u2-cu)*Z;
		W_right(9,3*i+1) = -f;
		W_right(10,3*i+1) = 0;
		W_right(11,3*i+1) = u2-cu;
		W_right(12,3*i+1) = B*f;

		W_right(0,3*i+2) = f*v2*X;
		W_right(1,3*i+2) = f*v2*Y;
		W_right(2,3*i+2) = f*v2*Z;
		W_right(3,3*i+2) = -f*u2*X;
		W_right(4,3*i+2) = -f*u2*Y;
		W_right(5,3*i+2) = -f*u2*Z;
		W_right(6,3*i+2) =  (cu*v2-cv*u2)*X;
		W_right(7,3*i+2) =  (cu*v2-cv*u2)*Y;
		W_right(8,3*i+2) =  (cu*v2-cv*u2)*Z;
		W_right(9,3*i+2) = f*v2;
		W_right(10,3*i+2) = -f*u2;
		W_right(11,3*i+2) = -cv*u2+cu*v2;
		W_right(12,3*i+2) = -B*f*v2;	
	}

	if(localopts.typeRMM == 1)
	{
		HouseholderQR<MatrixXd> qr_left(W_left.transpose());
		RMM_l = qr_left.matrixQR().block(0, 0, 13, 13).triangularView<Upper>();
		HouseholderQR<MatrixXd> qr_right(W_right.transpose());
		RMM_r = qr_right.matrixQR().block(0, 0, 13, 13).triangularView<Upper>();
	}

	else if(localopts.typeRMM == 2)
	{
		JacobiSVD<MatrixXd> svd_left(W_left.transpose());
		MatrixXd D_l = svd_left.singularValues().asDiagonal();
		MatrixXd V_l = svd_left.matrixV();
		RMM_l.noalias() = D_l * V_l.transpose();

		JacobiSVD<MatrixXd> svd_right(W_right.transpose());
		MatrixXd D_r = svd_right.singularValues().asDiagonal();
		MatrixXd V_r = svd_right.matrixV();
		RMM_r.noalias() = D_r * V_r.transpose();
	}

	else if(localopts.typeRMM == 3)
	{
		//RMM_l.setZero();
		RMM_l.selfadjointView<Upper>().rankUpdate(W_left);
		RMM_l.triangularView<StrictlyLower>() = RMM_l.transpose();

		//RMM_r.setZero();
		RMM_r.selfadjointView<Upper>().rankUpdate(W_right);
		RMM_r.triangularView<StrictlyLower>() = RMM_r.transpose();
	}
}

MatrixXd backprojAVGSACOpt(const MatrixXd& LP, const MatrixXd& RP, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib, const Options& localopts)
{	
	MatrixXd X_left_p(LP.rows(), 3);
	triangulate(LP, RP, calib, X_left_p);

	//El tema de la creacion de la RMM hay que mirarlo bien...
	MatrixXd RMM_left(13, 13);
	MatrixXd RMM_right(13, 13);
	createRMMatrix(X_left_p, LC, RC, calib, RMM_left, RMM_right, localopts);

	int N = X_left_p.rows();
	vector< Matrix4d, Eigen::aligned_allocator<Matrix4d> > vec_models;
	vector< pair<double, int> > vec_res;

	MatrixXd X_left_p_t(3, 3);
	MatrixXd LC_t(3, 2);
	MatrixXd RC_t(3, 2);

	vector<int> indices(3);
	Matrix4d curr_sol;

	// Generate and evaluate all the models with the RMMs
	for(int it = 0; it < localopts.maxiters; it++)
	{
		getSamples(N, 3, it, indices);
		selectData(indices, X_left_p, LC, RC, X_left_p_t, LC_t, RC_t);
		
		VectorXd seed = VectorXd::Zero(6);
		seed(5) = 0.8;
		VectorXd sol_t = run_opt(X_left_p_t, LC_t, RC_t, calib, seed, 10, 1e-4, 1e-4, 1e-3);
		cardanExp(sol_t, curr_sol);

		double curr_res = evalSolution(curr_sol, RMM_left, RMM_right);
		vec_models.push_back(curr_sol);
		vec_res.push_back(make_pair<double, int>(curr_res, it));

	}

	// Select just the "NM" models with smallest residuals
	int NM = min(localopts.avgWindow, localopts.maxiters);  
	vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> > best_models = selectData(vec_res, vec_models, NM);
   
	// perform the L1-averaging
    	Matrix4d bestSol = L1averaging(best_models, localopts.epsilonAvg, 300);

	if(localopts.recalculateModel)
	{
		vector<int> best_indices = evalSolution(bestSol, X_left_p, LC, RC, calib, localopts.threshold);
		int bestInliers = best_indices.size();

		MatrixXd X_left_p_f(bestInliers, 3);
		MatrixXd LC_f(bestInliers, 2);
		MatrixXd RC_f(bestInliers, 2);

		selectData(best_indices, X_left_p, LC, RC, X_left_p_f, LC_f, RC_f);

		//final optimization!
		VectorXd seed = VectorXd::Zero(6);
		seed(5) = 0.8;
		VectorXd sol_t = run_opt(X_left_p_f, LC_f, RC_f, calib, seed, 10, 1e-4, 1e-4, 1e-3);		
		bestSol = cardanExp(sol_t);
	}

	return bestSol;
}


