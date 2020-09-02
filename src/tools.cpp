#include <iostream>
#include <cv.hpp>
#include "tools.h"

bool interCameraParamGet(Mat &k, Mat &d, int dlen, string file)
{
	ifstream in;
	in.open(file, ios::in);
	int i = 0;
	k = Mat::eye(Size(3, 3), CV_32FC1);
	d = Mat::zeros(Size(1, dlen), CV_32FC1);

	while (i < 9)
	{
		if (!(in >> k.at<float>(i / 3, i % 3)))
		{
			cout << i << endl;
			in.close();
			return false;
		}
		i++;
	}

	i = 0;
	while (i < dlen)
	{
		if (!(in >> d.at<float>(i, 0)))
		{
			cout << i << endl;
			in.close();
			return false;
		}
		i++;
	}

	in.close();
	// cout << k << endl << endl;
	// cout << d << endl << endl;
	return true;
}

bool interCameraParamGet(Mat &k, Mat &d, string file)
{
	return true;
}