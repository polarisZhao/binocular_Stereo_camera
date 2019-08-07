//
//  useCamera.cpp
//  use binocular cemera to show view(Binocular/Left/right)
//  Usage: g++ `pkg-config opencv --cflags` useCamera.cpp -o useCamera `pkg-config opencv --libs`
//
// Created by zhaozhichao on 2019/8/4
// Copyright © 2019 zhaozhichao. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, const char * argv[]){

    cv::VideoCapture Camera(0);
    if (!Camera.isOpened()){
        std::cout << "Could not open the Camera !!" << std::endl;
        return -1;
    }

    cv::Mat Frame, BinocularView, LeftView, RightView;
    Camera >> Frame;
    system("../utils/chmodCamera.sh");

    while (true){
        Camera >> Frame;
        if (Frame.empty()) break;
        cv::resize(Frame, BinocularView, cv::Size(640, 240), (0, 0), (0, 0), cv::INTER_AREA);
        LeftView = BinocularView(cv::Rect(0, 0, 320, 240));
        RightView = BinocularView(cv::Rect(320, 0, 320, 240));

        cv::imshow("Binocular View", BinocularView);
        cv::imshow("Left View", LeftView);
        cv::imshow("Right View", RightView);

        char c = cv::waitKey(30);
        if (c == 27){  // Press Esc to Exit
            break;
        }
    }

    return 0;
}

