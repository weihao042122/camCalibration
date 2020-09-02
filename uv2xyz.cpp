#include <iostream>
#include <signal.h>
#include <vector>
 
#include <opencv2/opencv.hpp>
#include "src/tools.h"

Mat K0, K1, R ,dicoeff0, dicoeff1, T;
int getCameraParameter(string path) {
    
    interCameraParamGet(K0, dicoeff0, 5, path+"/calibResult0.txt");
    interCameraParamGet(K1, dicoeff1, 5, path+"/calibResult1.txt");
    interCameraParamGet(R, T, 3, path+"/extrinsics.txt");
    return 0;
}

int uv2xyz(Point3f uv0, Point3f uv1, Mat K0, Mat K1, Mat R, Mat T, Point3f& xyz)
{
	Mat m0 = Mat(uv0);
	Mat m1 = Mat(uv1);

	// cout << "m0:" << endl << m0 << endl;
	// cout << "m1:" << endl << m1 << endl;

	
	Mat A = R*K0.inv()*m0;
	Mat B = K1.inv()*m1;

	// cout << "A:" << endl << A << endl;
	// cout << "B:" << endl << B << endl;

	float Ty = T.at<float>(1, 0);
	float Tx = T.at<float>(0, 0);
	float Ay = A.at<float>(1, 0);
	float Ax = A.at<float>(0, 0);
	float By = B.at<float>(1, 0);
	float Bx = B.at<float>(0, 0);


	// cout <<"Tz="<< Tz << " Tx=" << Tx << endl;
	// cout <<"Az="<< Az << " Ax=" << Ax << endl;
	// cout <<"Bz="<< Bz << " Bx=" << Bx << endl;
	float lamda2 = (Ty-Tx*Ay/Ax)/(By-Bx*Ay/Ax);
	float lamda1 = (lamda2*By-Ty)/Ay;
	
	Mat t = K0.inv() * m0 * lamda1;
	// cout << t << endl;
    xyz.x = t.at<float>(0, 0);
    xyz.y = t.at<float>(1, 0);
    xyz.z = t.at<float>(2, 0);
	return 0;
}

int main(int argc, char **argv) {
    if (argc != 3){
        cout << argv[0] << " paramPath \"[x1, y1], [x2, y2]\"" << endl;
		exit(-1);
    }
	string path = argv[1];
    float x1, y1, x2, y2;
    sscanf(argv[2], "[%f, %f], [%f, %f]", &x1, &y1, &x2, &y2);
	getCameraParameter(path);
	cout << "K0:" << endl << K0 << endl;
	cout << "d0:" << endl << dicoeff0 << endl;
	cout << "K1:" << endl << K1 << endl;
	cout << "d1:" << endl << dicoeff1 << endl;
	cout << "R:" << endl << R << endl;
	cout << "T:" << endl << T << endl;
	// return 0;
	// [406.08,380.16]|[309.6,377.28]
	// [456,383]|[360,381]
	//
	// Point3f uv00 = Point3f(406.08, 380.16, 1);
	// Point3f uv01 = Point3f(309.6, 377.28, 1);
	// Point3f uv10 = Point3f(456, 383, 1);
	// Point3f uv11 = Point3f(360, 381, 1);
	// Point3f xyz = Point3f(0, 0, 0);

	// [449,247]|[342,243]
	// [449,278]|[342,274]
	// [515.52,273.6]|[408.96,269.28]
	Point3f uv00 = Point3f(x1, y1, 1);
	Point3f uv01 = Point3f(x2, y2, 1);
	Point3f xyz = Point3f(0, 0, 0);

	uv2xyz(uv00, uv01, K0, K1, R, T, xyz);
    cout << xyz << endl;
	return 0;
}