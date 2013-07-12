# Copyright 2013. All rights reserved.
# Author: German Ros (gros@cvc.uab.es)
#         Advanced Driver Assistance Systems (ADAS)
#         Computer Vision Center (CVC)
#	  Universitat Autònoma de Barcelona (UAB)
#
# This is free software; you can redistribute it and/or modify it under the
# terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or any later version.
#
# This software is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
# Street, Fifth Floor, Boston, MA 02110-1301, USA 
#
#

1. Purpose of this software
----------------------------

The software here provided performs robust Stereo Odometry 
by using a novel technique called Coarse L1-Averaging.
Here we present a demo that calculate the trajectory of a moving
vehicle along a city. 

The user needs to provide feature associations for the different views, since
our software is just intended to estimate the trajectory. However, for the sake
of simplicity we have included the matcheds of a partial sequence from the KITTI
benchmark http://www.cvlibs.net/datasets/kitti/

For any doubt, comment, suggestion or bug just send my an e-mail to gros@cvc.uab.es

2. Software compilation and execution
--------------------------------------

1. cd L1AVGVO
2. Create a directory (e.g., build): mkdir build
3. cd build
4. cmake ..
5. Check that cmake found all the required dependencies, mainly Eigen and
   OpenCV for visualization purposes (you can remove the visualization part in
   the main.cpp file. Just modify the lines that have to do with the renderer).
6. make
7. Everything ok so far? Then run run_demo.sh. To do this you might need to change
   default mode: chmod 777 run_demo.sh

   Then run the script as follows: ./run_demo.sh

3. Citing this software
------------------------

If you use this software for any of your work, please cite it as follows:

@inproceedings{ros13,
 author = {Ros, G. and Guerrero, J. and Sappa, A. D. and Ponsa, D. and L\'{o}pez, A. M.},
 title = {Fast and Robust l1-averaging-based Pose Estimation for Driving Scenarios},
 booktitle = {Proceedings of the British Machine Vision Conference},
 year = {2013},
 address = {Bristol, UK},
}

