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

#ifndef _RENDERVO_HPP_
	#define _RENDERVO_HPP_

	#include "render.hpp"

	/**
	* This class offers rendering functionalities for Visual Odometry approaches.
	* It is able of drawing a trajectory, a 3D map, and other related things.
	*/
	class RenderVO : public Render
	{
		private:
			//Internal vector of poses
			vector<Point3d> vec_poses;
			Arrays arr_poses;

			//landmarks and their colours go here
			vector<Arrays> arr_landmarks;		

			//figure representing the camera/car
			vector<Point3d> vec_head;			
			Arrays arr_head;
			double w_head, h_head;

			//variables for the ORTHO view
			float minX, maxX, minY, maxY;
			bool firstTime;
			float min_xp, max_xp, min_yp, max_yp;
			float h_length, v_length, hv_length;

		public:
			RenderVO(string name, int width, int height) : Render(name, width, height)
			{
				minX = -100.0;
				maxX = 100.0;
				minY = -100.0;
				maxY = 100.0;

				w_head = 0.5;
				h_head = 1.0;
				firstTime = true;

				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				gluOrtho2D(minX, maxX, minY, maxY);	
			}

			void addPose(MatrixXd P, int ID)
			{
				Point3d lastPoint = Point3d(P(0, 3), P(2, 3), 1.0);
				Point3d vec;
				vec_poses.push_back(lastPoint);

			
				if(firstTime)
				{
					min_xp = lastPoint.x;
					max_xp = lastPoint.x;
					min_yp = lastPoint.y;
					max_yp = lastPoint.y;
					firstTime = false;
				}

				else
				{
					min_xp = (lastPoint.x < min_xp)? lastPoint.x : min_xp;
					max_xp = (lastPoint.x > max_xp)? lastPoint.x : max_xp;
					min_yp = (lastPoint.y < min_yp)? lastPoint.y : min_yp;
					max_yp = (lastPoint.y > max_yp)? lastPoint.y : max_yp;
				}

				h_length = 1.3*(max_xp - min_xp);
				v_length = 1.3*(max_yp - min_yp);
				hv_length = max(h_length, v_length);

				arr_poses.setVertexArray(vec_poses);	

				//head location
				vec_head.clear();

				//it's required to rescale the head to mantain a fix size in screen
				double w_headScaled = w_head * 0.05 * hv_length;
				double h_headScaled = h_head * 0.05 * hv_length;

				Point3d point0 = Point3d(lastPoint.x, lastPoint.y + 0.5*h_headScaled, 1.0);
				Point3d point1 = Point3d(lastPoint.x - 0.5*w_headScaled, lastPoint.y - 0.5*h_headScaled, 1.0);
				Point3d point2 = Point3d(lastPoint.x + 0.5*w_headScaled, lastPoint.y - 0.5*h_headScaled, 1.0);

				Point3d point0n, point1n, point2n;

				if(vec_poses.size() > 1)
					vec = lastPoint - vec_poses.at(vec_poses.size()-2);
				else
					vec = lastPoint;

				//this is for giving the proper orientation to the triangle aka head
				double alpha = acos(vec.y / sqrt(vec.x*vec.x + vec.y*vec.y));
				alpha = (vec.x < 0)? -alpha : alpha;

				point0n.x = cos(alpha)*point0.x + sin(alpha)*point0.y - lastPoint.y * sin(alpha) + lastPoint.x * (-cos(alpha) + 1.0);
				point0n.y = -sin(alpha)*point0.x + cos(alpha)*point0.y + lastPoint.x * sin(alpha)  + lastPoint.y * (1 - cos(alpha));
				point0n.z = 1.0;

				point1n.x = cos(alpha)*point1.x + sin(alpha)*point1.y - lastPoint.y * sin(alpha) + lastPoint.x * (-cos(alpha) + 1.0);
				point1n.y = -sin(alpha)*point1.x + cos(alpha)*point1.y + lastPoint.x * sin(alpha)  + lastPoint.y * (1 - cos(alpha));
				point1n.z = 1.0;

				point2n.x = cos(alpha)*point2.x + sin(alpha)*point2.y - lastPoint.y * sin(alpha) + lastPoint.x * (-cos(alpha) + 1.0);
				point2n.y = -sin(alpha)*point2.x + cos(alpha)*point2.y + lastPoint.x * sin(alpha)  + lastPoint.y * (1 - cos(alpha));
				point2n.z = 1.0;

				vec_head.push_back(point0n);
				vec_head.push_back(point1n);
				vec_head.push_back(point2n);
				arr_head.setVertexArray(vec_head);
			
			}

			/*
			* Method that serves to include new landmarks to the current map.
			* A landmark is just a 3 components vector
			*/
			void addLandmarks(const vector<Point3d>& X)
			{
				vector<Point3d> vec_landmarks;
				for(int i=0; i< X.size(); i++)
				{
					Point3d pt = X[i];
					pt.y = pt.z;
					pt.z = 1.0;
					vec_landmarks.push_back(pt);
				}

				Arrays a;
				a.setVertexArray(vec_landmarks);
			
				arr_landmarks.push_back(a);
				
			}

			/*
			* Method that serves to include new landmarks to the current map.
			* Here, colour values of each landmark are also considered (C vector).
			*/
			void addLandmarks(const vector<Point3d>& X, const vector<Vec3b>& C)
			{
				vector<Point3d> vec_landmarks;
				vector<Vec3b> vec_landmarksColours;
				for(int i=0; i< X.size(); i++)
				{
					Point3d pt = X[i];
					pt.y = pt.z;
					pt.z = 1.0;
					vec_landmarks.push_back(pt);
		
					vec_landmarksColours.push_back(C[i]);
				}

				Arrays a;
				a.setVertexArray(vec_landmarks);
				a.setColorArray(vec_landmarksColours);
				arr_landmarks.push_back(a);
			}

			/*
			* Function that draws the scene as an ortho view
			*/
			void draw()
			{
				double cX = 0.5*(max_xp + min_xp);
				double cY = 0.5*(max_yp + min_yp);

				minX = cX - 0.5*hv_length;
				maxX = cX + 0.5*hv_length;
				minY = cY - 0.5*hv_length;
				maxY = cY + 0.5*hv_length;

				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				gluOrtho2D(minX, maxX, minY, maxY);

				for(int i=0; i<arr_landmarks.size(); i++)
					render(arr_landmarks[i], POINTS, Scalar(250, 230, 230));

				glLineWidth(5.0);

				render(arr_poses, LINE_STRIP, Scalar(255, 0, 0));
				render(arr_head, TRIANGLES, Scalar(0, 255, 0));
			}
	};

#endif


