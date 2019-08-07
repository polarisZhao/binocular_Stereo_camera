//
//  imgCaputer.cpp
//  use binocular cemera to capute img (Binocular/Left/right)
//  Usage: g++ `pkg-config opencv --cflags` imgCaputer.cpp -o imgCaputer `pkg-config opencv --libs`
//
// Created by zhaozhichao on 2019/8/4
// Copyright © 2019 zhaozhichao. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

using namespace cv;
using namespace std;

int main(){

    cv::VideoCapture cap(0);
    if (!cap.isOpened()){
        std::cout << "Could not open the Camera !!" << std::endl;
        return -1;
    }

    static int i = 1;
    Mat Frame, BinocularView, LeftView, RightView;;
    cap >> Frame;
    system("../utils/chmodCamera.sh");

    while(true) {
        cap >> Frame; 
	if (Frame.empty())   break;

        cv::resize(Frame, BinocularView, cv::Size(640, 240), (0, 0), (0, 0), cv::INTER_AREA);
        LeftView = BinocularView(cv::Rect(0, 0, 320, 240));
        RightView = BinocularView(cv::Rect(320, 0, 320, 240));

        cv::imshow("Binocular View", BinocularView);
        cv::imshow("Left View", LeftView);
        cv::imshow("Right View", RightView);

	char c = cv::waitKey(30);
	if (c == 27){    // Press Esc to Exit
	    break;
	}

	if (c == 'S'){    // Press 'S' to save img   
        string left_name = "../img/left_" + to_string(i) + ".png"; 
	    imwrite(left_name, LeftView);
  
        string right_name = "../img/right_" + to_string(i) + ".png"; 
	    imwrite(right_name, RightView);

	    string binocular_name = "../img/bin_" + to_string(i) + ".png"; 
	    imwrite(binocular_name, BinocularView);

	    i++;
	}
    }

    cap.release();
    return 0;
}


