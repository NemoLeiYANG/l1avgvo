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


#ifndef _RENDER_HPP_
	#define _RENDER_HPP_

#include <iostream>
#include <fstream>
#include <vector>
#include <GL/gl.h>
#include <GL/glu.h>
#include "opencv2/core/core.hpp"
#include "opencv2/core/opengl_interop.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ArcBall.h"

using namespace std;
using namespace cv;
using namespace ogl;

void drawer(void* param);
void mouseCallback(int event, int x, int y, int flags, void* userdata);

/**
* The Render class offers OpenGL capabilities over OpenCV highgui windows.
* This class is very generic and needs to be completed. So far it is just a skeleton
* 
*/
class Render
{
	private:
		//Window properties
		int width, height;
		string name;

		//Arcball-related properties
		ArcBallT* arcball;
		Point2fT MousePt;                                             //*NEW* Current mouse point
		bool isClicked;                                           //*NEW* Clicking the mouse?
		bool isRClicked;                                          //*NEW* Clicking the right mouse button?
		bool isMoving;

		Matrix4fT   Transform; 
		Matrix3fT   LastRot;
		Matrix3fT   ThisRot;

		void initializeTrackballVars()
		{
			isClicked = false;
			isRClicked = false;
			isMoving = false;

			Matrix4fSetIdentity(&Transform);
			Matrix3fSetIdentity(&LastRot);
			Matrix3fSetIdentity(&ThisRot);
		}

	public:

		Render(string name, int width, int height)
		{
			this->name = name;
			this->width = width;
			this->height = height;

			namedWindow(name, WINDOW_OPENGL | CV_WINDOW_AUTOSIZE);
			resizeWindow(name, width, height);

			setOpenGlContext(name);
			setOpenGlDrawCallback(name, drawer, this);

			
			setMouseCallback(name, mouseCallback, this);
			initializeTrackballVars();
			arcball = new ArcBallT(width, height);
			arcball->setBounds((GLfloat)width, (GLfloat)height);

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
	
		}

		void onMouseEvent(int event, int x, int y, int flags)
		{
			return;
		}

		void update()
		{
			setOpenGlContext(name);	
			updateWindow(name);
		}

		virtual void draw() = 0;

		void setSize(int width, int height)
		{
			this->width = width;
			this->height = height;

			resizeWindow(name, width, height);
			arcball->setBounds((GLfloat)width, (GLfloat)height);
		}

		Mat renderToMat()
		{
			setOpenGlContext(name);	

			Mat img(height, width, CV_8UC3);
    			glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3)?1:4);
  			glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize());
    			glReadPixels(0, 0, img.cols, img.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, img.data);
	
			Mat flipped(img);
			flip(img, flipped, 0);

			return flipped;
		}

		void setPosition(int x, int y)
		{
			moveWindow(name, x, y);	
		}
};

/*
* Auxiliar function that acts as a proxy for drawing
*/
void drawer(void* param)
{
	Render* renderer = (Render*)param;
	renderer->draw();
}

/*
* mouse callback
*/
void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
	Render* renderer = (Render*)userdata;
	renderer->onMouseEvent(event, x, y, flags);
}

#endif
