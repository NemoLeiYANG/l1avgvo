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

#ifndef _SE3_HPP_
#define _SE3_HPP_

	#include <iostream>
	#include <Eigen/Dense>
	#include <vector>
	#include <cmath>
	#include "mytime.hpp"

	using namespace Eigen;
	using namespace std;

	typedef Matrix<double, 6, 1> Vector6d;

	/*
	* Fast inverse of a SE(3) element
	* Instead of computing a regular inverse we
	* transpose the rotation
	*/
	void InverseSE3(const Matrix3d& R, Matrix3d& Rt, const Vector3d& T, Matrix4d& invTr);

	/*
	* Logarithm map of SO(3)
	*/
	Vector3d so3Log(Matrix3d R);

	/*
	* Exponential map of SO(3)
	*/
	Matrix3d so3Exp(Vector3d V);

	/*
	* Logarithm map of SE(3)
	*/
	Vector6d se3Log(Matrix4d M);

	/*
	* Exponential map of SE(3)
	*/
	Matrix4d se3Exp(Vector6d V);

	/*
	* Derivatives of the exponential map for SE(3).
	* Again, the structure is a tensor given as a set
	* of matrices (6 matrices).
	*/
	Matrix4d* JExpSE3(Vector6d V, int n);

	/*
	* This is a method of L2 geodesic averaging
	* as described by Prof. Govindu in "Lie-algebraic
	* averaging for globally consistent motion estimation"
	*/
	Matrix4d L2averaging(const vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> > models, double epsilon, int maxiters);

	/*
	* This is the Weiszfeld algorithm for L1 geodesic averaging
	* as described by Prof. Hartley in "L1 rotation averaging using
	* the Weiszfeld algorithm"
	*/
	Matrix4d L1averaging(const vector<Matrix4d, Eigen::aligned_allocator<Matrix4d> > models, double epsilon, int maxiters);

#endif

