//
//  3dreconstruction.cpp
//  use binocular cemera to 3D reconstruction
//
// Created by zhaozhichao on 2019/8/11
// Copyright © 2019 zhaozhichao. All rights reserved.
//

#define  _SCL_SECURE_NO_WARNINGS

#include <iostream>   
#include <opencv2/opencv.hpp>  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
  
using namespace pcl;
using namespace std;
using namespace cv;

void viewerOneOff(visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
}

Mat color;
Mat rectifyImageL, rectifyImageR;

Rect validROIL; 
Rect validROIR;
Mat Q;
Mat xyz;
Ptr<StereoBM> bm = StereoBM::create(16, 9);
int blockSize = 1, uniquenessRatio = 0, numDisparities = 5;
void stereo_match_sgbm(int, void*) 
{

	PointCloud<PointXYZRGB> cloud_a;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	color = imread("../rectified.png");
        cv::imshow("color", color);
	
	int rowNumber = color.rows;
	int colNumber = color.cols;

	cloud_a.height = rowNumber;
	cloud_a.width = colNumber;
	cloud_a.points.resize(cloud_a.width * cloud_a.height);

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
	waitKey(0);

	for (unsigned int u = 0; u < rowNumber; ++u)
	{
		for (unsigned int v = 0; v < colNumber; ++v)
		{
			/* unsigned int num = rowNumber*colNumber-(u*colNumber + v)-1; */
			unsigned int num = u*colNumber + v;

			cloud_a.points[num].b = color.at<Vec3b>(u, v)[0];
			cloud_a.points[num].g = color.at<Vec3b>(u, v)[1];
			cloud_a.points[num].r = color.at<Vec3b>(u, v)[2];

			cloud_a.points[num].x = u;  // xyz.at<Vec3f>(u, v)[0];
			cloud_a.points[num].y = v;  // xyz.at<Vec3f>(u, v)[1];
			cloud_a.points[num].z = xyz.at<Vec3f>(u, v)[2];
		}
	}
	*cloud = cloud_a;
	visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	while (!viewer.wasStopped())
	{
	}

}


Point origin;                              
Rect selection;                             
bool selectObject = false;                      										
static void onMouse(int event, int x, int y, int, void*){
    if (selectObject){
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event){
	case EVENT_LBUTTONDOWN:  
            origin = Point(x, y);
	    selection = Rect(x, y, 0, 0);
            selectObject = true;
            std::cout << origin << " in world coordinate is: " << xyz.at<Vec3f>(origin)<< endl<< " ori "<<xyz.at<Vec3f>(origin)[0] << std::endl;
	    break;
	case EVENT_LBUTTONUP:
	    selectObject = false;
	    if (selection.width > 0 && selection.height > 0) break;
    }
}


int main(int argc, const char * argv[])
{
	Mat intrMatFirst, intrMatSec, distCoeffsFirst, distCoffesSec;
	Mat R, T, E, F, RFirst, RSec, PFirst, PSec;
	Rect validRoi[2];
	Mat viewLeft, viewRight;

        // read image
	viewLeft = imread("../LeftView.png", 1);
	viewRight = imread("../RightView.png", 1);

	Size imageSize = viewLeft.size();
        Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC3);
	Mat canvasLeft = canvas(Rect(0, 0, imageSize.width, imageSize.height));
	Mat canvasRight = canvas(Rect(imageSize.width, 0, imageSize.width, imageSize.height));

	FileStorage fs("../../utils/intrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"] >> intrMatFirst;fs["D1"] >> distCoeffsFirst;
		fs["M2"] >> intrMatSec;fs["D2"] >> distCoffesSec;

		fs["R"] >> R;
		fs["T"] >> T;
		fs["Q"] >> Q;

		fs.release();
	}

        // stereo rectify
	std::cout << " ... stereo rectify ..." << std::endl;
	stereoRectify(intrMatFirst, distCoeffsFirst, intrMatSec, distCoffesSec, imageSize, R, T, 
                      RFirst, RSec, PFirst, PSec, Q, CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);

        
	Mat rmapFirst[2], rmapSec[2], rviewFirst, rviewSec;
	initUndistortRectifyMap(intrMatFirst, distCoeffsFirst, RFirst, PFirst,
		                imageSize, CV_16SC2, rmapFirst[0], rmapFirst[1]);
	initUndistortRectifyMap(intrMatSec, distCoffesSec, RSec, PSec,
		                imageSize, CV_16SC2, rmapSec[0], rmapSec[1]);

	remap(viewLeft, rectifyImageL, rmapFirst[0], rmapFirst[1], INTER_LINEAR);
	remap(viewRight, rectifyImageR, rmapSec[0], rmapSec[1], INTER_LINEAR);

	rectifyImageL.copyTo(canvasLeft);
	rectifyImageR.copyTo(canvasRight);

        cv::imwrite("../rectified.png", rectifyImageL);

	cvtColor(rectifyImageL, rectifyImageL, CV_BGR2GRAY);
	cvtColor(rectifyImageR, rectifyImageR, CV_BGR2GRAY);

	// imshow("remap_left", rectifyImageL);
	// imshow("remap_right", rectifyImageR);

	namedWindow("disparity", WINDOW_NORMAL);
	// createTrackbar("BlockSize:\n", "disparity", &blockSize, 8, stereo_match_sgbm);
	// createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match_sgbm);
	// createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match_sgbm);
	// setMouseCallback("disparity", onMouse, 0);

	stereo_match_sgbm(0, 0); 

	waitKey(0);
	while (1);

}
