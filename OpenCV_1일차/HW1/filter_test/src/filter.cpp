#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

int main() 
{
    cv::Mat image = cv::imread("../resources/image.png");

    if(image.empty()) {
        std::cout << "이미지 로딩 실패" << std::endl;
        return -1;
    }

    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask_yellow;

    cv::inRange(hsv, cv::Scalar(20, 170, 100), cv::Scalar(30, 255, 255), mask_yellow);

    cv::Mat blurred_mask;
    cv::Mat blurred_hsv;
    
    cv::GaussianBlur(hsv, blurred_hsv, cv::Size(3,3), 5);
    cv::inRange(blurred_hsv, cv::Scalar(20, 170, 100), cv::Scalar(30, 255, 255), blurred_mask);

    cv::imshow("Original Image", image);
    cv::imshow("Gaussian Blurred", blurred_mask);
    cv::imshow("NON_filtered_mask", mask_yellow);

    cv::imwrite("blurred_image.jpg", blurred_mask);
    cv::imwrite("yellow.jpg", mask_yellow);

    cv::waitKey(0);

    return 0;
}
