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

#include <iostream>
#include <fstream>
#include <dirent.h>
#include <Eigen/Dense>
#include "backprojavgsac.hpp"
#include "mytime.hpp"
#include "renderVO.hpp"
#include "tools.hpp"


int main(int argc, char **argv)
{
	Timerx timer;

	if(argc != 5)
	{
		cerr << "Syntax error. You must provide all these parameters: " << argv[0] << " matcheds_folder calibration_file options_file use_prior (0|1)" << endl;
		return -1;
	}

	char* folder = argv[1];
	bool cost = atoi(argv[4]);
	Calibration calib = readCalibration(argv[2]);
	Options localopts = readOptions(argv[3]);

	//Window creation
	int win_width = 800;
	int win_height = 800;
	RenderVO* renderer = new RenderVO("Visual Odometry", win_width, win_height);

	//matcheds files
	vector<string> files = getDirAsVector(folder);

	MatrixXd currentPose = MatrixXd::Identity(4, 4);
	bool quit = false;
	for(int i=0; i<files.size(); i++)
	{
		string current_file;
		current_file = string(folder) + "/" + files[i];

		cout << current_file << endl;
		multiMatrix matches = readData(current_file.c_str(), cost);

		//estimate the current pose
		MatrixXd sol = backprojAVGSACOpt(matches.LC, matches.RC, matches.LP, matches.RP, calib, localopts);
		currentPose = currentPose * sol;

		cout << currentPose << endl;

		//adding new pose and landmarks
		renderer->addPose(currentPose, 1);
		
		//Showing images and rendering scene
		renderer->update();

		//Detecting events
		int key = waitKey(5);
		switch(char(key))
		{
			case 'p':
				int k2;
				do{ k2 = waitKey(-1); } while( (char(k2) != 'p') && (char(k2) != 'q') );
				if(char(k2) == 'q')
					quit = true;
				break;
			case 'q':
				quit = true;
		}

		if(quit)
			break;
	}

	if(!quit)
		int key = waitKey(-1);
	delete renderer;

	return 0;	   
}
