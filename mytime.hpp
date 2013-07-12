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

#ifndef _MYTIME_H_
	#define _MYTIME_H_
	
	#include <iostream>
	#include <sys/time.h>

	using namespace std;
 
	class Timerx
	{
		private:
			timeval t1, t2;
			bool showMessage;

		public:
			Timerx(bool msg=false)
			{
				showMessage = msg;
			}			
	
			void start()
			{
				gettimeofday(&t1, NULL);
			}

			double stop(const char* msg)
			{
				gettimeofday(&t2, NULL);

				double elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    				elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
   
				if(showMessage)
					cout << msg << "; Time = [" << elapsedTime << "] ms" << endl;
				return elapsedTime;
			}
	};
#endif
