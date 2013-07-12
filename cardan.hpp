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

#ifndef _CARDAN_HPP_
#define _CARDAN_HPP_

	#include <iostream>
	#include <Eigen/Dense>

	using namespace Eigen;
	using namespace std;

	typedef Matrix<double, 6, 1> Vector6d;

	/*
	* These functions compute Cardan retractions
	* producing different outputs (for efficiency purposes) 
	*/

	void cardanExp2(const Vector6d& V, Matrix4d& RT);

	Matrix4d cardanExp(const Vector6d& V);

	void cardanExp(const Vector6d& V, Matrix4d& RT);

	void cardanExp(const Vector6d& V, Matrix3d& R, Vector3d& T);

	/*
	* This function computes the Jacobian of the Cardan map
	* with respect to the matrix parameters. Thus, the Jacobian
	* is a tensor, but we split it in 6 matrices.
	*/
	void JCardan(const Vector6d& V, Matrix4d derivative[6]);

#endif

