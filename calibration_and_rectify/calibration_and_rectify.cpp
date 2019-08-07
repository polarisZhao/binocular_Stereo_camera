//
//  calibration_and_rectify.cpp
//  camera calibration and rectify
//  Usage: g++ `pkg-config opencv --cflags` calibration_and_rectify.cpp -o calibration_and_rectify `pkg-config opencv --libs`
//
// Created by zhaozhichao on 2019/8/5
// Copyright © 2019 zhaozhichao. All rights reserved.
//


#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

// 此处参数需要根据棋盘格个数修改
// 例如：黑白棋盘格宽(w)为10个棋盘格, 那么w = 10 -1 = 9
#define  chessboard_w  9      // 棋盘格宽的黑白交叉点个数
#define  chessboard_h  6      // 棋盘格高的黑白交叉点个数
const  float chessboardSquareSize = 12.5f;  // 每个棋盘格方块的边长, 单位为mm

using namespace cv;

static bool getImgnameList(const std::string& xml_filename, std::vector<std::string>& imgname_list)
{
	imgname_list.resize(0);
    
	FileStorage fs(xml_filename, FileStorage::READ);
	if (!fs.isOpened()) return false;
    
	FileNode node = fs.getFirstTopLevelNode();
	if (node.type() != FileNode::SEQ) return false;
    
    for (FileNodeIterator it = node.begin(); it != node.end(); ++it){
		imgname_list.push_back((std::string)*it);
    }
	return true;
}


static void calcChessboardCorners(Size boardSize, float squareSize, std::vector<Point3f>& corners)
{
	corners.resize(0);
    for (int i = 0; i < boardSize.height; i++)
    {
        for (int j = 0; j < boardSize.width; j++)
        {
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        }
    }
}


bool calibrate(Mat& intrMat, Mat& distCoeffs, 
               std::vector<std::vector<Point2f>>& imagePoints,
               std::vector<std::vector<Point3f>>& ObjectPoints,
               Size& imageSize, const int cameraId, std::vector<std::string> imageList)
{
    
	Size boardSize;
	boardSize.width = chessboard_w;
	boardSize.height = chessboard_h;
    std::vector<Point2f> pointBuf;
	float squareSize = chessboardSquareSize;
    std::vector<Mat> rvecs, tvecs; // rotate vector & translation vector

	int nImages = (int)imageList.size() / 2;
    std::cout <<"num of image:"<< nImages << std::endl;;

	int num_of_validity_img = 0;
	for (int i = 0; i< nImages; i++)
	{
		Mat view, viewGray;
		view = imread(imageList[i * 2 + cameraId], 1);
		imageSize = view.size();
		cvtColor(view, viewGray, COLOR_BGR2GRAY);

		bool found = findChessboardCorners(view, boardSize, pointBuf,
			                               CV_CALIB_CB_ADAPTIVE_THRESH |
                                           CV_CALIB_CB_FAST_CHECK |
                                           CV_CALIB_CB_NORMALIZE_IMAGE);
		if(found)
		{
			num_of_validity_img++;
			cornerSubPix(viewGray, pointBuf, Size(11, 11),
				Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(view, boardSize, Mat(pointBuf), found);
			imagePoints.push_back(pointBuf);
		}
		imshow("View", view);
		waitKey(250);
	}
    std::cout << "num of validity img:" << num_of_validity_img << std::endl;

	// calculate chessboardCorners
	calcChessboardCorners(boardSize, squareSize, ObjectPoints[0]);
	ObjectPoints.resize(imagePoints.size(), ObjectPoints[0]);
	double rms = cv::calibrateCamera(ObjectPoints, imagePoints, imageSize, intrMat, distCoeffs,
                                     rvecs, tvecs);
	if (checkRange(intrMat) && checkRange(distCoeffs)) {
		std::cout << "calibrate done with RMS error = " << rms << std::endl;
		return true;
	}
	else{
		return false;
	}
}


int main(int argc, const char * argv[])
{
	// (1) parse imgname from xml file
    std::vector <std::string> imgname_list;
    std::string xml_filename = "./img_list.xml";
	bool read_ok = getImgnameList(xml_filename, imgname_list);
	if (!read_ok || imgname_list.empty()){
		std::cout << " Can't open " << xml_filename
		          << " or the string list is empty" << std::endl;
		return -1;
	}
	if (imgname_list.size() % 2 != 0){
        std::cout << " Error: the image list contains odd (non-even) number of elements" << std::endl;
		return -1;
	}

    // (2) calibrate
    Size imageSize;
    int cameraIdFirst = 0, cameraIdSec = 1;
    Mat intrMatFirst, intrMatSec, distCoeffsFirst, distCoffesSec;
    std::vector<std::vector<Point2f>> imagePointsFirst, imagePointsSec;
    std::vector<std::vector<Point3f>> ObjectPoints(1);
	FileStorage fs("../utils/intrinsics.yml", FileStorage::WRITE);

    std::cout << "calibrate left camera ..." << std::endl;
	bool calib_left_ok = calibrate(intrMatFirst, distCoeffsFirst, imagePointsFirst,
                                   ObjectPoints, imageSize, cameraIdFirst, imgname_list);
	if (!calib_left_ok){
        std::cout << "fail to calibrate the left camera" << std::endl;
		return -1;
	}
    
    std::cout << "calibrate the right camera ..." << std::endl;
	bool calib_right_ok = calibrate(intrMatSec, distCoffesSec, imagePointsSec,
	                                ObjectPoints, imageSize, cameraIdSec, imgname_list);
	if (!calib_right_ok){
		std::cout << "fail to calibrate the right camera" << std::endl;
		return -1;
	}
    
    fs << "M1" << intrMatFirst 
       << "D1" << distCoeffsFirst 
       << "M2" << intrMatSec 
       << "D2" << distCoffesSec;
    std::cout << "intrMatFirst" << intrMatFirst << std::endl;
    std::cout << "distCoeffsFirst" << distCoeffsFirst << std::endl;
    std::cout << "intrMatSec" << intrMatSec << std::endl;
    std::cout << "distCoffesSec" << distCoffesSec << std::endl;
    
	// (3) estimate position and orientation
    Mat R, T, E, F;
	std::cout << "estimate position and orientation of the second camera" << std::endl
		      << "relative to the first camera..." << std::endl;
	double rms = cv::stereoCalibrate(ObjectPoints, imagePointsFirst, imagePointsSec,
		                      intrMatFirst, distCoeffsFirst, intrMatSec, distCoffesSec, 
		                      imageSize, R, T, E, F, CALIB_USE_INTRINSIC_GUESS, // CV_CALIB_FIX_INTRINSIC,
		                      TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));
	std::cout << " Calibrate done with RMS error = " << rms << std::endl;
    
    // (4) binocular stereo rectify
    std::cout << "stereo rectify ... " << std::endl;
	Mat  RFirst, RSec, PFirst, PSec, Q;
	Rect validRoi[2];
	stereoRectify(intrMatFirst, distCoeffsFirst, intrMatSec, distCoffesSec, imageSize, R, T, 
		          RFirst, RSec, PFirst, PSec, Q, CALIB_ZERO_DISPARITY, -1, imageSize, &validRoi[0], &validRoi[1]);

	std::cout << "R: " << R << "T:" << T <<"Q:" << Q << std::endl;
	std::cout << "P1:" << PFirst << " R1: " << RFirst << std::endl;
	std::cout << "P2:" << PSec << " R2: " << RSec << std::endl;

	if (fs.isOpened())
	{
		fs << "R" << R 
		   << "T" << T 
		   << "R1" << RFirst 
		   << "R2" << RSec 
		   << "P1" << PFirst 
		   << "P2" << PSec 
		   << "Q" << Q;
		fs.release();
	}

    // (5) binocular stereo rectify
    std::cout << "read the picture for 3d-reconstruction..." << std::endl;

	Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC3);
	Mat canvasLeft = canvas(Rect(0, 0, imageSize.width, imageSize.height));
	Mat canvasRight = canvas(Rect(imageSize.width, 0, imageSize.width, imageSize.height));

	Mat viewLeft, viewRight;
	viewLeft = imread(imgname_list[6], 1);
	viewRight = imread(imgname_list[7], 1);

	// show image before rectify
	viewLeft.copyTo(canvasLeft);
	viewRight.copyTo(canvasRight);
	for (int j = 0; j <= canvas.rows; j += 16)  // draw line  
		line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
	imshow("before rectify", canvas);

    // rectify
	Mat rmapFirst[2], rmapSec[2], rviewFirst, rviewSec;
	initUndistortRectifyMap(intrMatFirst, distCoeffsFirst, RFirst, PFirst,
		                    imageSize, CV_16SC2, rmapFirst[0], rmapFirst[1]); 
	initUndistortRectifyMap(intrMatSec, distCoffesSec, RSec, PSec, 
		                    imageSize, CV_16SC2, rmapSec[0], rmapSec[1]); 
	remap(viewLeft, rviewFirst, rmapFirst[0], rmapFirst[1], INTER_LINEAR);
	remap(viewRight, rviewSec, rmapSec[0], rmapSec[1], INTER_LINEAR);

    //　show image after rectify
	rviewFirst.copyTo(canvasLeft);
	rviewSec.copyTo(canvasRight);
	for (int j = 0; j <= canvas.rows; j += 16)  // draw line 
		line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
	imshow("after rectify", canvas); 
	waitKey(0);
    
	return 0;
}
