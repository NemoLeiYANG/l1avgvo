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

#ifndef _OPTIMIZER_H_
	#define _OPTIMIZER_H

	#include <iostream>
	#include <Eigen/Dense>
	#include "cardan.hpp"
	#include "SE3.hpp"
	#include "mytime.hpp"
	#include "tools.hpp"
	#include <unsupported/Eigen/NonLinearOptimization>

	using namespace std;
	using namespace Eigen;

	/*
	* This function runs the optimization process based on LM optimizer
	*/
	VectorXd run_opt(const MatrixXd& X_left_p, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib, VectorXd& seed, int maxits, double epsx, double epsg, double epsf);

	/**
	* General structure for functors
	*/
	struct FunctorOpti
	{
		virtual int operator()(const VectorXd& x, VectorXd& fvec) = 0;
		virtual int df(const VectorXd& x, MatrixXd& fjac) = 0;
		virtual int setData(const MatrixXd& X_left_p, const MatrixXd& LC, const MatrixXd& RC, const Calibration& calib) = 0;
		virtual int inputs() const = 0;
		virtual int values() const = 0;	
	};

#endif
