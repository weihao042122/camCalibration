#include "undistort.h"

bool CUndistort::readParams()
{
    ifstream in;
    if (calibResultFile != "")
        in.open(calibResultFile, ios::in);
    else
        in.open(calibResultPath+"/calibResult.txt", ios::in);
    
#if 0
    in>>K.at<float>(0, 0);
    in>>K.at<float>(1, 1);
    in>>K.at<float>(0, 2);
    in>>K.at<float>(1, 2);
#ifdef CV
	in >> discoeff.at<float>(0, 0);
	in >> discoeff.at<float>(1, 0);
	in >> discoeff.at<float>(2, 0);
	in >> discoeff.at<float>(3, 0);
	in >> discoeff.at<float>(4, 0);
#elif defined FISHEYE
	in >> discoeff.at<float>(0, 0);
	in >> discoeff.at<float>(1, 0);
	in >> discoeff.at<float>(2, 0);
	in >> discoeff.at<float>(3, 0);
#endif
#else
    int i = 0;
    while (i < 9) {
        if(!(in>>K.at<float>(i/3, i%3)))
        {
            cout << i << endl;
            in.close();
            return false;
        }
        i++;
    }
    
    i = 0; 
    while (i < 5) {
        if(!(in>>discoeff.at<float>(i, 0)))
        {
            cout << i << endl;
            in.close();
            return false;
        }
        i++;
    }
#endif
    in.close();
    cout << K << endl;
    cout << discoeff << endl;
    return true;
}

bool CUndistort::undistProcess(char *file)
{
    //***************»û±äÐ£Õý****************//
    R=Mat::eye(Size(3, 3),CV_32FC1);
    Mat mapx, mapy;
    Mat srcImg=imread(srcImgPath);
    Mat dstImg;
#ifdef CV
    cv::initUndistortRectifyMap(K, discoeff, R, K, srcImg.size(),CV_32FC1, mapx, mapy);
#elif defined FISHEYE
	cv::fisheye::initUndistortRectifyMap(K, discoeff,R, K, srcImg.size(), CV_32FC1, mapx, mapy);
#endif
    remap(srcImg, dstImg, mapx, mapy, CV_INTER_LINEAR);
	// cv::resize(dstImg, dstImg, cv::Size(), 0.25, 0.25, CV_INTER_LINEAR);
	cv::namedWindow("show", 1);
    imshow("show", dstImg);
    waitKey(0);
    
    if (file != NULL) {
        vector<int> compression_params;
        compression_params.push_back(IMWRITE_JPEG_QUALITY);
        compression_params.push_back(100);
        imwrite(file, dstImg, compression_params);
    }

    return true;
}

void CUndistort::run(char *file)
{
    bool readSuccess=readParams();
	if (!readSuccess)
	{
		cout << "read Params Failed!" << endl;
		getchar();
	}
    undistProcess(file);
}

