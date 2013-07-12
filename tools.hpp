#ifndef _TOOLS_HPP_
	#define _TOOLS_HPP_

	#include <iostream>
	#include <fstream>
	#include <dirent.h>
	#include <Eigen/Dense>
	#include <opencv2/opencv.hpp>

	using namespace std;
	using namespace Eigen;
	using namespace cv;

	/**
	* This structure serves to store a matched feature in 4 views
	* u is the x-coordinate in image plane and v is the y-coordinate.
	* lp, rp, lc and rc stand for left previous (current) and 
	* right previous (current)
	*/
	struct VOData
	{
		int ulp, vlp, urp, vrp, ulc, vlc, urc, vrc, cost;

		VOData() {};

		VOData(int ulp, int vlp, int urp, int vrp, int ulc, int vlc, int urc, int vrc, int cost) : 
			ulp(ulp), vlp(vlp), urp(urp), vrp(vrp), ulc(ulc), vlc(vlc), urc(urc), vrc(vrc), cost(cost) {};
	};

	/**
	* This structure contains the extracted keypoints
	* for the 4 views, i.e., LP (left previous), RP (right previous),
	* LC (left current) and RC (right current)
	*/
	struct multiMatrix
	{
		MatrixXd LP, RP, LC, RC;
	};

	typedef Matrix<double, 4, 8> Matrix48d;
	typedef Matrix<double, 8, 8> Matrix8d;
	typedef Matrix<double, 6, 1> Vector6d;

	/**
	* Some useful parameters:
	* 	maxiters:		Number of models generated
	* 	avgWindow:		Number of models averaged
	* 	threshold:		Threshold for identifying outliers
	* 	epsilonAvg:		Stop condition for averaging
	* 	recalculateModel:	Shall we recalculate a final model just with inliers?
	* 	opt_maxits:		Max. number of iterations for optimizer
	* 	opt_epsx:		Min. change in x tolerated by the optimizer
	* 	opt_epsg:		Min. change in the gradient tolerated by the optimizer
	* 	opt_epsf:		Min. change in the cost function tolerated by the optimizer
	*/
	struct Options
	{
		int maxiters;
		int avgWindow;
		double threshold;
		double epsilonAvg;
		bool recalculateModel;

		int opt_maxits;
		double opt_epsx;
		double opt_epsg;
		double opt_epsf;

		int typeRMM;
	};

	/** These elements are:
	*	focal length
	*	principal point (component x)
	*	principal point (component y)
	*	Baseline
	*/
	struct Calibration
	{
		double f, cu, cv, B;
	};


	multiMatrix readData(const char* file, bool cost=false);

	Calibration readCalibration(const char* file);

	Options readOptions(const char* file);

	vector<string> getDirAsVector(string path);

#endif
