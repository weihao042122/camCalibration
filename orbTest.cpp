#include <iostream>
#include <signal.h>
#include <vector>
 
#include <opencv2/opencv.hpp>
#include "src/tools.h"
 
using namespace cv;
using namespace std;
 
int ChessBoardCorners(char *file){
    Mat img=imread(file, IMREAD_GRAYSCALE);
    
    Mat image,scaleImg;
    int scale = 1;
    Size boardSize=Size(10, 7);
    vector<Point2f> corners;//存储一幅棋盘图中的所有角点二维坐标
    vector<vector<Point2f>> cornersSeq;//存储所有棋盘图角点的二维坐标
    vector<Mat> image_Seq;//存储所有棋盘图
    
    image=img.clone();
		//降采样原图,加快角点提取速度
// 		cv::resize(image, scaleImg, cv::Size(), scale, scale, CV_INTER_LINEAR);
	    /**********************提取角点*************************/
	    bool patternfound= findChessboardCorners(image, boardSize,
            corners, CALIB_CB_ADAPTIVE_THRESH);
	    if (!patternfound)
	    {
		    cout<<"Can not find chess board corners!\n"<<endl;
                imshow("CirclePattern", image);
                cout << "press any key to see next pattern image" << endl;
                waitKey(-1);
	    }
	    else
	    {
			//上采样corner
			for (int num = 0; num < corners.size(); num++)
			{
				cv::Point2f tempPoint = corners[num];
				corners[num] = cv::Point2f(tempPoint.x / scale, tempPoint.y / scale);
			}

		    /************************亚像素精确化******************************/
// 		    cornerSubPix(image, corners, Size(11, 11), Size(-1,-1) , TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
// 			bool good = testCorners(corners, boardSize.width, boardSize.height);
            bool good = true;
			if (true == good) {
                /************************绘制检测到的角点并显示******************************/
                Mat cornerImg = image.clone();
                cvtColor(cornerImg, cornerImg, CV_GRAY2BGR);
                for (int j=0; j< corners.size(); j++)
                {
                    circle(cornerImg, corners[j], 2, Scalar(0,0,255), 2, 4, 0);
                }
                // namedWindow("CirclePattern", WINDOW_NORMAL);
    #if SHOW_IMG  || 1
                imshow("CirclePattern", cornerImg);
                cout << "press any key to see next pattern image" << endl;
                waitKey(-1);
    #endif
    // 		    cornersSeq.push_back(corners);
    // 			image_Seq.push_back(image);
            }
	    }
	    return 1;
}
 
int main(int argc, char **argv)
{
    if (argc != 3) {
        cout << argv[0] << " pic1 pic2" << endl;
        return -1;
    }
    
    cv::Mat img_1 = cv::imread( argv[1] ); 
    cv::Mat img_2 = cv::imread( argv[2] ); 
// 	Mat img_1 = imread("data/1.png");
// 	Mat img_2 = imread("data/2.png");
 
	if (!img_1.data || !img_2.data)
	{
		cout << "error reading images " << endl;
		return -1;
	}
 
	vector<Point2f> recognized;
	vector<Point2f> scene;
 
	recognized.resize(500);
	scene.resize(500);
 
	Mat d_srcL, d_srcR;
 
	Mat img_matches, des_L, des_R;
	//ORB算法的目标必须是灰度图像
	cvtColor(img_1, d_srcL, COLOR_BGR2GRAY);//CPU版的ORB算法源码中自带对输入图像灰度化，此步可省略
	cvtColor(img_2, d_srcR, COLOR_BGR2GRAY);
 
	Ptr<ORB> d_orb = ORB::create();
 
	Mat d_descriptorsL, d_descriptorsR, d_descriptorsL_32F, d_descriptorsR_32F;
 
	vector<KeyPoint> keyPoints_1, keyPoints_2;
 
	//设置关键点间的匹配方式为NORM_L2，更建议使用 FLANNBASED = 1, BRUTEFORCE = 2, BRUTEFORCE_L1 = 3, BRUTEFORCE_HAMMING = 4, BRUTEFORCE_HAMMINGLUT = 5, BRUTEFORCE_SL2 = 6 
	Ptr<DescriptorMatcher> d_matcher = DescriptorMatcher::create(NORM_L2);
 
	std::vector<DMatch> matches;//普通匹配
	std::vector<DMatch> good_matches;//通过keyPoint之间距离筛选匹配度高的匹配结果
 
	d_orb -> detectAndCompute(d_srcL, Mat(), keyPoints_1, d_descriptorsL);
 
	d_orb -> detectAndCompute(d_srcR, Mat(), keyPoints_2, d_descriptorsR);
 
	d_matcher -> match(d_descriptorsL, d_descriptorsR, matches);
 
	int sz = matches.size();
	double max_dist = 0; double min_dist = 100;
 
	for (int i = 0; i < sz; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
 
	cout << "\n-- Max dist : " << max_dist << endl;
	cout << "\n-- Min dist : " << min_dist << endl;
 
	for (int i = 0; i < sz; i++)
	{
		if (matches[i].distance < 0.3*max_dist)
		{
			good_matches.push_back(matches[i]);
		}
	}
	cout << "good match<0.3*" << max_dist << endl;
    cout << "good match cnt:" << good_matches.size() << endl;
	//提取良好匹配结果中在待测图片上的点集，确定匹配的大概位置
	for (size_t i = 0; i < good_matches.size(); ++i)
	{
		scene.push_back(keyPoints_2[ good_matches[i].trainIdx ].pt);
	}
 
// 	for(unsigned int j = 0; j < scene.size(); j++)
// 		cv::circle(img_2, scene[j], 2, cv::Scalar(0, 255, 0), 2);
	//画出普通匹配结果
	Mat ShowMatches;
	drawMatches(img_1,keyPoints_1,img_2,keyPoints_2,matches,ShowMatches);
	imshow("matches", ShowMatches);
    waitKey(0);
	imwrite("matches.png", ShowMatches);
    destroyWindow("matches");
    
    //画出良好匹配结果
	Mat ShowGoodMatches;
	drawMatches(img_1,keyPoints_1,img_2,keyPoints_2,good_matches,ShowGoodMatches);
	imshow("good_matches", ShowGoodMatches);
    waitKey(0);
	imwrite("good_matches.png", ShowGoodMatches);
    destroyWindow("good_matches");
    
    for(int i = 0; i < good_matches.size(); i++) {
        cout << keyPoints_1[ good_matches[i].queryIdx].pt << ", " <<keyPoints_2[ good_matches[i].trainIdx ].pt << endl;
        	//画出良好匹配结果中在待测图片上的点集
        Mat t2 = img_2.clone();
        Mat t1 = img_1.clone();
        cv::circle(t2, keyPoints_2[ good_matches[i].trainIdx ].pt, 1, cv::Scalar(0, 255, 0), 2);
        cv::circle(t1, keyPoints_1[ good_matches[i].queryIdx ].pt, 1, cv::Scalar(0, 255, 0), 2);
        imshow("MatchPoints_in_img_2", t2);
        imshow("MatchPoints_in_img_1", t1);
        waitKey(0);
    }
	return 0;
}