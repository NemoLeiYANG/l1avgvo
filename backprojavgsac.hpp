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

#ifndef _BACKPROJAVGSAC_H_
	#define _BACKPROJAVGSAC_H_

	#include <iostream>
	#include <vector>
	#include <algorithm>
	#include <Eigen/Dense>
	#include <Eigen/QR>
	#include "optimizer.hpp"
	#include "mytime.hpp"

	using namespace std;
	using namespace Eigen;

	typedef Matrix<double, 4, 8> Matrix48d;
	typedef Matrix<double, 8, 8> Matrix8d;
	typedef Matrix<double, 6, 1> Vector6d;
	typedef Matrix<double, 13, 1> Vector13d;
	typedef Matrix<double, 1, 13> RowVector13d;

	/*
	* This function performs or call most of the functionality of the program
	*/
	MatrixXd backprojAVGSACOpt(const MatrixXd& LP, const MatrixXd& RP, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib, const Options& localopts);
	
	/*
	* It creates two Reduced Measurement Matrices from the data
	*/
	void createRMMatrix(const MatrixXd& X_left_p, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib, MatrixXd& RMM_l, MatrixXd& RMM_r, const Options& localopts);

	/*
	* Point triangulation from data correspondences and calibration parameters
	*/
	void triangulate(const MatrixXd& LP, const MatrixXd& RP, const Calibration& calib, MatrixXd& PTS);

	/*
	* Quick hypotheses evaluation via coarse functions (based on RMMs)
	*/
	vector<int> evalSolution(const Matrix4d& Tr, const MatrixXd& X_left_p, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib, double threshold);

	inline double evalSolution(const Matrix4d& Tr, const MatrixXd& RMM_l, const MatrixXd& RMM_r);

	/*
	* This function selects a subset of the whole data set as specified by the indices vector.
	*/
	void selectData(const vector<int>& indices, const MatrixXd& In_X_left_p, const MatrixXd& In_LC, const MatrixXd& In_RC, MatrixXd& Out_X_left_p, MatrixXd& Out_LC, MatrixXd& Out_RC);

	vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> > selectData(const vector< pair<double, int> >& residuals, const vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> >& models, int N);

	/*
	* Function used to select minimum correspondences to generate a SE(3) model-
	* N stands for the total number of correspondences
	* M is the number of indices to be selected
	* indices is a vector that will contain the selected indices
	* current_it is just the current iteration. This is needed for the
	* Progressive selection scheme.
	*/
	void getSamples(int N, int M, int current_it, vector<int>& indices);

	void getSamples(int N, int M, vector<int>& indices);

#endif
