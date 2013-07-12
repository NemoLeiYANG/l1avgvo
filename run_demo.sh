#!/bin/bash

# The parameters of this demo are:
# A directory containing files with matcheds. Each file
# will contain all the matcheds between two stereo frames (i.e., 4 views).
# Each match will be stored in a different line and its format is:
#
# x_left_previous x_right_previous, x_left_current, x_right_current
#
# where x_* is a point (x, y) in pixel coordinates.
#
# The next parameter is the calibration file, which contains the:
# 	focal length, principal point components (u, v) and the baseline
# These four parameters are separated by spaces.
#
# Then the options file contains several options required by the method. Such
# options are:
#	maxiters:		Number of models generated
#	avgWindow:		Number of models averaged
#	threshold:		Threshold for identifying outliers
#	epsilonAvg:		Stop condition for averaging
#	recalculateModel:	Shall we recalculate a final model just with inliers?
#	opt_maxits:		Max. number of iterations for optimizer
#	opt_epsx:		Min. change in x tolerated by the optimizer
#	opt_epsg:		Min. change in the gradient tolerated by the optimizer
#	opt_epsf:		Min. change in the cost function tolerated by the optimizer
#
#
# Finally we use a boolean variable (1 or 0) to specify if the provided matcheds
# also contain prior information regarding their "goodness". If the data files
# contain this information then you should set the last parameter to 1 (0 otherwise).
#
#
./build/l1avgVO data/seq00_part1 data/calib.txt data/options.txt 1
