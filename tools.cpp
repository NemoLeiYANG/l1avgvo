#include "tools.hpp"

bool sortCost(VOData l, VOData r) { return l.cost < r.cost; }

/**
* This function loads the matcheds from a file. 
* These matcheds might or might not include prior information
* about their own goodness. 
* If this information is available then "cost" should be set to true.
* The format of the files should consist in one line per feature matched in 4 views.
* Then, each line has the following format: 
* 2D_point_left_previous_view 2D_point_right_previous_view 2D_point_left_current_view 2D_point_right_current_view 
*
* Notice this is just the association of a keypoint in 4 views. A prior of its goodness can be also provided after
* 2D_point_right_current_view.
*/
multiMatrix readData(const char* file, bool cost)
{
	ifstream fd(file);
	if(!fd.is_open())
	{
		cerr << "[Reading Data] Error reading the input file" << endl;
		throw -2;
	}

	vector<VOData> vec_data;
	VOData row; 
	
	double c;
	while(!fd.eof())
	{
		fd >> row.ulp;
		fd >> row.vlp;
		fd >> row.urp;
		fd >> row.vrp;

		fd >> row.ulc;
		fd >> row.vlc;
		fd >> row.urc;
		fd >> row.vrc;
	
		if(cost)
		{
			fd >> c;
			row.cost = c;
		}

		vec_data.push_back(row);
	}

	fd.close();

	int N = vec_data.size();

	if(cost)
		sort(vec_data.begin(), vec_data.end(), sortCost);

	MatrixXd pts_left_p(N, 2);
	MatrixXd pts_right_p(N, 2);
	MatrixXd pts_left_c(N, 2);
	MatrixXd pts_right_c(N, 2);

	for(int i=0; i<N; i++)
	{
		pts_left_p(i, 0) = vec_data[i].ulp;
		pts_left_p(i, 1) = vec_data[i].vlp;
		pts_right_p(i, 0) = vec_data[i].urp;
		pts_right_p(i, 1) = vec_data[i].vrp;

		pts_left_c(i, 0) = vec_data[i].ulc;
		pts_left_c(i, 1) = vec_data[i].vlc;
		pts_right_c(i, 0) = vec_data[i].urc;
		pts_right_c(i, 1) = vec_data[i].vrc;
	}
	
	multiMatrix result;
	result.LP = pts_left_p;
	result.RP = pts_right_p;
	result.LC = pts_left_c;
	result.RC = pts_right_c;
	
	return result;
}

/** 
* This function reads a calibratoin file.
* The format of the calibration file is just
* a vector of 4 elements separated by spaces.
* These elements are:
*	focal length
*	principal point (component x)
*	principal point (component y)
*	Baseline
*/
Calibration readCalibration(const char* file)
{
	ifstream fd(file);
	if(!fd.is_open())
	{
		cerr << "[Reading Calibration] Error reading the input file" << endl;
		throw -3;
	}

	Calibration cal;

	fd >> cal.f;
	fd >> cal.cu;
	fd >> cal.cv;
	fd >> cal.B;

	fd.close();

	return cal;
}

/**
* This function reads the options from a file.
* The format of such a file is very simple. It is
* just a vector of values separated by spaces. The order
* of the parameters are:
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
Options readOptions(const char* file)
{
	ifstream fd(file);
	if(!fd.is_open())
	{
		cerr << "[Reading Options] Error reading the input file" << endl;
		throw -4;
	}

	Options opts;

	fd >> opts.maxiters;
	fd >> opts.avgWindow;
	fd >> opts.threshold;
	fd >> opts.epsilonAvg;
	fd >> opts.recalculateModel;
	fd >> opts.opt_maxits;
	fd >> opts.opt_epsx;
	fd >> opts.opt_epsg;
	fd >> opts.opt_epsf;
	fd >> opts.typeRMM;

	fd.close();

	return opts;
}

/**
* This is just an auxiliar function to
* read the number of files of a given folder
*/
int myFilter(const struct dirent *d)
{
	if (d->d_type != DT_REG)
		return 0;
	return 1;
}

vector<string> getDirAsVector(string path)
{
	vector<string> results;
	struct dirent **namelist;
	int n;

	n = scandir(path.c_str(), &namelist, myFilter, versionsort);
	for(int i=0; i<n; i++)
		results.push_back(namelist[i]->d_name);

	for(int i=0; i<n; i++)
		free(namelist[i]);
        free(namelist);

	return results;
}

