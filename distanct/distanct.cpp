//
//  distanct.cpp
//  use binocular camera to compute distanct
//  Usage: g++ `pkg-config opencv --cflags` distanct.cpp -o distanct `pkg-config opencv --libs`
//
// Created by zhaozhichao on 2019/8/5
// Copyright © 2019 zhaozhichao. All rights reserved.
//

#include <opencv2/opencv.hpp>  
#include <iostream>  

using namespace std;
using namespace cv;

Mat rectifyImageL, rectifyImageR;
Rect validROIL, validROIR;
Mat Q;
Mat xyz;                                         
Ptr<StereoBM> bm = StereoBM::create(16, 9);
int blockSize = 0, uniquenessRatio = 0, numDisparities = 0;

Point origin;                                    
Rect selection;                                 
bool selectObject = false;

void stereo_match_sgbm(int, void*)
{
	bm->setBlockSize(2 * blockSize + 5);    
	bm->setROI1(validROIL);
	bm->setROI2(validROIR);
	bm->setPreFilterCap(31);
	bm->setMinDisparity(0);  
	bm->setNumDisparities(numDisparities * 16 + 16);
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(uniquenessRatio);
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(-1);
	Mat disp, disp8;
	bm->compute(rectifyImageL, rectifyImageR, disp);
	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));
	reprojectImageTo3D(disp, xyz, Q, true); 
	xyz = xyz * 16;
	imshow("disparity", disp8);
}
 
												  
static void onMouse(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:   
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		std::cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << std::endl;
		break;
	case EVENT_LBUTTONUP:    
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			break;
	}

}


int main()
{
	// (1) read parameter from intrinsics.yml file
	Mat intrMatFirst, intrMatSec, distCoeffsFirst, distCoffesSec;
	Mat R, T, E, F, RFirst, RSec, PFirst, PSec ;
	Rect validRoi[2];

	FileStorage fs("../utils/intrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"] >> intrMatFirst;
		fs["D1"] >> distCoeffsFirst;
		fs["M2"] >> intrMatSec;
		fs["D2"] >> distCoffesSec;

		fs["R"] >> R;
		fs["T"] >> T;
		fs["Q"] >> Q;

		fs.release();
	}

    // (2) read image
	Mat viewLeft, viewRight;
	viewLeft = imread("../img/left_13.png", 1);
	viewRight = imread("../img/right_13.png", 1);																						 
	Size imageSize = viewLeft.size();

    // (3) stereoRectify
	stereoRectify(intrMatFirst, distCoeffsFirst, intrMatSec, distCoffesSec, imageSize,
	              R, T, RFirst, RSec, PFirst, PSec,
	              Q, CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);

	Mat rmapFirst[2], rmapSec[2], rviewFirst, rviewSec;
	initUndistortRectifyMap(intrMatFirst, distCoeffsFirst, RFirst, PFirst,
		imageSize, CV_16SC2, rmapFirst[0], rmapFirst[1]);
	initUndistortRectifyMap(intrMatSec, distCoffesSec, RSec, PSec,
		imageSize, CV_16SC2, rmapSec[0], rmapSec[1]);
	remap(viewLeft, rectifyImageL, rmapFirst[0], rmapFirst[1], INTER_LINEAR);
	remap(viewRight, rectifyImageR, rmapSec[0], rmapSec[1], INTER_LINEAR);
	cvtColor(rectifyImageL, rectifyImageL, CV_BGR2GRAY);
	cvtColor(rectifyImageR, rectifyImageR, CV_BGR2GRAY);
	imshow("remap_left", rectifyImageL);
	imshow("remap_right", rectifyImageR);

    // (4) computer disparity
	namedWindow("disparity", WINDOW_NORMAL);
	createTrackbar("BlockSize:\n", "disparity", &blockSize, 8, stereo_match_sgbm);
	createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match_sgbm);
	createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match_sgbm);
	setMouseCallback("disparity", onMouse, 0);
	stereo_match_sgbm(0, 0);
	waitKey(0);

	return 0;

}