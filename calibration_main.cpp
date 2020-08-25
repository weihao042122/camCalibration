#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "src/calibration.h"
#include "src/undistort.h"

using namespace std;
using namespace cv;

void help_print(char* a){
    cout <<a<<" [path]"<<endl;
}

int main(int argc, char **argv)
{
	string path;
    cout << argc << endl;
    if (argc != 2) {
        help_print(argv[0]);
        exit(-1);
    }
    path = argv[1];
    string patternImgPath = path+"/";
    string calibResultPath = path + "/";
#if !IMG_JPEG
    string srcImgPath = path+"/"+"3.bmp";
#else
    string srcImgPath = path+"/"+"3.jpeg";
#endif
    cout << patternImgPath << endl;
    cout << calibResultPath << endl;
    cout << srcImgPath << endl;

//     Size boardSize=Size(9, 6);  //my chessboard
    Size boardSize=Size(10, 7);
    CCalibration calibration(patternImgPath, calibResultPath, boardSize);
    calibration.run(path);
    CUndistort undistort(srcImgPath, calibResultPath);
    undistort.run(NULL);
}
