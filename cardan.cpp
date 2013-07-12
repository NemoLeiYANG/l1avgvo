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

#include "cardan.hpp"

void cardanExp(const Vector6d& V, Matrix4d& RT)
{
	double sv0 = sin(V(0));
	double cv0 = cos(V(0));
	double sv1 = sin(V(1));
	double cv1 = cos(V(1));
	double sv2 = sin(V(2));
	double cv2 = cos(V(2));

	RT(0, 0) = cv1*cv2;
	RT(0, 1) = -cv1*sv2;
	RT(0, 2) = -sv1;

	RT(1, 0) = -cv2*sv0*sv1 + cv0*sv2;
	RT(1, 1) = cv0*cv2 + sv0*sv1*sv2;
	RT(1, 2) = -cv1*sv0;

	RT(2, 0) = cv0*cv2*sv1 + sv0*sv2;
	RT(2, 1) = cv2*sv0 - cv0*sv1*sv2;
	RT(2, 2) = cv0*cv1;

	RT(0, 3) = V(3);
	RT(1, 3) = V(4);
	RT(2, 3) = V(5);
	RT(3, 0) = 0;
	RT(3, 1) = 0;
	RT(3, 2) = 0;
	RT(3, 3) = 1;
	
}

void cardanExp2(const Vector6d& V, Matrix4d& RT)
{
	Matrix3d R1, R2, R3;

	R1 << 	1,		0,		0,		
		0,		cos(V(0)),	-sin(V(0)),
		0,		sin(V(0)),	cos(V(0));

	R2 <<	cos(V(1)),	0,		-sin(V(1)),
		0,		1,		0,
		sin(V(1)),	0,		cos(V(1));

	R3 <<	cos(V(2)),	-sin(V(2)),	0,
		sin(V(2)),	cos(V(2)),	0,
		0,		0,		1;

	RT.block<3, 3>(0, 0) = R1*R2*R3;
	RT(0, 3) = V(3);
	RT(1, 3) = V(4);
	RT(2, 3) = V(5);
	RT(3, 0) = 0;
	RT(3, 1) = 0;
	RT(3, 2) = 0;
	RT(3, 3) = 1;
}

Matrix4d cardanExp(const Vector6d& V)
{
	Matrix3d R1, R2, R3;
	Matrix4d RT;

	R1 << 	1,		0,		0,		
		0,		cos(V(0)),	-sin(V(0)),
		0,		sin(V(0)),	cos(V(0));

	R2 <<	cos(V(1)),	0,		-sin(V(1)),
		0,		1,		0,
		sin(V(1)),	0,		cos(V(1));

	R3 <<	cos(V(2)),	-sin(V(2)),	0,
		sin(V(2)),	cos(V(2)),	0,
		0,		0,		1;

	Matrix3d R = R1*R2*R3;

	RT.block<3, 3>(0, 0) = R;
	RT(0, 3) = V(3);
	RT(1, 3) = V(4);
	RT(2, 3) = V(5);
	RT(3, 0) = 0;
	RT(3, 1) = 0;
	RT(3, 2) = 0;
	RT(3, 3) = 1;

	return RT;
}

void cardanExp(const Vector6d& V, Matrix3d& R, Vector3d& T)
{
	Matrix3d R1, R2, R3;

	R1 << 	1,		0,		0,		
		0,		cos(V(0)),	-sin(V(0)),
		0,		sin(V(0)),	cos(V(0));

	R2 <<	cos(V(1)),	0,		-sin(V(1)),
		0,		1,		0,
		sin(V(1)),	0,		cos(V(1));

	R3 <<	cos(V(2)),	-sin(V(2)),	0,
		sin(V(2)),	cos(V(2)),	0,
		0,		0,		1;

	R = R1*R2*R3;
	T = V.tail<3>();
}

/**
* It returns an array of 6 4x4 matrices (a tensor in matrix form)
*/
void JCardan(const Vector6d& V, Matrix4d derivative[6])
{

	double ca1 = cos(V(0));
	double sa1 = sin(V(0));
	double ca2 = cos(V(1));
	double sa2 = sin(V(1));
	double ca3 = cos(V(2));
	double sa3 = sin(V(2));	

	Matrix4d J1, J2, J3, J4, J5, J6;

	J1 <<	0,			0,				0,		0,
		-ca1*ca3*sa2 - sa1*sa3,	-ca3*sa1 + ca1*sa2*sa3,		-ca1*ca2,	0,
		-ca3*sa1*sa2 + ca1*sa3,	ca1*ca3 + sa1*sa2*sa3,		-ca2*sa1,	0,
		0,			0,				0,		0;

	J2 <<	-ca3*sa2,		sa2*sa3,			-ca2,		0,
		-ca2*ca3*sa1,		ca2*sa1*sa3,			sa1*sa2,	0,
		ca1*ca2*ca3,		-ca1*ca2*sa3,			-ca1*sa2,	0,
		0,			0,				0,		0;

	J3 <<	-ca2*sa3,		-ca2*ca3,			0,		0,
		ca1*ca3 + sa1*sa2*sa3,	ca3*sa1*sa2 - ca1*sa3,		0,		0,
		ca3*sa1 - ca1*sa2*sa3,	-ca1*ca3*sa2 - sa1*sa3,		0,		0,
		0,			0,				0,		0;

	J4 = Matrix4d::Zero(4, 4);
	J4(0, 3) = 1;

	J5 = Matrix4d::Zero(4, 4);
	J5(1, 3) = 1;

	J6 = Matrix4d::Zero(4, 4);
	J6(2, 3) = 1;

	derivative[0] = J1;
	derivative[1] = J2;
	derivative[2] = J3;
	derivative[3] = J4;
	derivative[4] = J5;
	derivative[5] = J6;
}
