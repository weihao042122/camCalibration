#ifndef _TOOLS_H_
#define _TOOLS_H_
#include "opencv2/opencv.hpp"
#include <fstream>
#include <string>
#include <iostream>
#include "calibration.h"

using namespace std;
using namespace cv;


bool interCameraParamGet(Mat &k, Mat &d, int dlen, string file);
bool interCameraParamGet(Mat &k, Mat &d, string file);

#endif /* _TOOLS_H_ */
