#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

// 색상 범위를 설정하는 함수
cv::Mat getMask(const cv::Mat& image, cv::Scalar lower, cv::Scalar upper) {
    cv::Mat hsv, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lower, upper, mask);
    return mask;
}

// 커스텀 오프닝 연산
cv::Mat customOpen(const cv::Mat& mask, int kernelSize) {
    cv::Mat eroded, opened;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::erode(mask, eroded, kernel);          // 커스텀 침식
    cv::dilate(eroded, opened, kernel);       // 커스텀 팽창
    return opened;
}

// 커스텀 클로징 연산
cv::Mat customClose(const cv::Mat& mask, int kernelSize) {
    cv::Mat dilated, closed;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::dilate(mask, dilated, kernel);        // 커스텀 팽창
    cv::erode(dilated, closed, kernel);       // 커스텀 침식
    return closed;
}

int main()
{
    cv::Mat image = cv::imread("/home/lwj/Documents/KakaoTalk Downloads/ballon.jpg");

    if (image.empty()) 
    {
        std::cout << "이미지 로딩 실패" << std::endl;
        return -1;
    }

    // 색상 범위 설정 (HSV 범위)
    cv::Scalar lowerRed1(0, 100, 100), upperRed1(10, 255, 255);   // 빨간색
    cv::Scalar lowerRed2(160, 100, 100), upperRed2(179, 255, 255); // 빨간색
    cv::Scalar lowerGreen(35, 100, 100), upperGreen(85, 255, 255); // 초록색
    cv::Scalar lowerBlue(100, 200, 50), upperBlue(140, 255, 255);  // 파란색

    // 빨간색 마스크 생성 (두 개의 범위 결합)
    cv::Mat redMask1 = getMask(image, lowerRed1, upperRed1);
    cv::Mat redMask2 = getMask(image, lowerRed2, upperRed2);
    cv::Mat redMask = redMask1 | redMask2;

    // 초록색 및 파란색 마스크 생성
    cv::Mat greenMask = getMask(image, lowerGreen, upperGreen);
    cv::Mat blueMask = getMask(image, lowerBlue, upperBlue);

    // 커널 크기 설정
    int kernelSize = 5;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

    // **1. OpenCV 내장 함수로 Open + Close 연산 수행**
    cv::Mat redOpenCloseOpencv, greenOpenCloseOpencv, blueOpenCloseOpencv;
    cv::morphologyEx(redMask, redOpenCloseOpencv, cv::MORPH_OPEN, kernel); // Open
    cv::morphologyEx(redOpenCloseOpencv, redOpenCloseOpencv, cv::MORPH_CLOSE, kernel); // Close

    cv::morphologyEx(greenMask, greenOpenCloseOpencv, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(greenOpenCloseOpencv, greenOpenCloseOpencv, cv::MORPH_CLOSE, kernel);

    cv::morphologyEx(blueMask, blueOpenCloseOpencv, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(blueOpenCloseOpencv, blueOpenCloseOpencv, cv::MORPH_CLOSE, kernel);

    // **2. 커스텀 함수로 Open + Close 연산 수행**
    cv::Mat redOpenCloseCustom = customClose(customOpen(redMask, kernelSize), kernelSize);
    cv::Mat greenOpenCloseCustom = customClose(customOpen(greenMask, kernelSize), kernelSize);
    cv::Mat blueOpenCloseCustom = customClose(customOpen(blueMask, kernelSize), kernelSize);

    // 결과 이미지 출력
    cv::imshow("Red Open+Close OpenCV", redOpenCloseOpencv);
    cv::imshow("Red Open+Close Custom", redOpenCloseCustom);
    cv::imshow("Green Open+Close OpenCV", greenOpenCloseOpencv);
    cv::imshow("Green Open+Close Custom", greenOpenCloseCustom);
    cv::imshow("Blue Open+Close OpenCV", blueOpenCloseOpencv);
    cv::imshow("Blue Open+Close Custom", blueOpenCloseCustom);

    cv::waitKey(0);
    return 0;
}

