#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

// 색상 범위를 설정하는 함수
cv::Mat getMask(const cv::Mat& image, cv::Scalar lower, cv::Scalar upper) 
{
    cv::Mat hsv, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lower, upper, mask);
    return mask;
}

// 객체 검출 및 바운딩 박스 그리기 함수
void detectAndLabel(const cv::Mat& image, const cv::Mat& mask, const std::string& label, cv::Scalar boxColor) 
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); i++) 
    {
        if (cv::contourArea(contours[i]) > 100)  		// 최소 면적 검사하기  
        { 
            cv::Rect boundingBox = cv::boundingRect(contours[i]);
            cv::rectangle(image, boundingBox, boxColor, 2);
            cv::putText(image, label, boundingBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, boxColor, 2);
        }
    }
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
    cv::Scalar lowerBlue(100, 200, 50), upperBlue(140, 255, 255);   // 파란색 lowerblue 튜닝 필요 

    // 빨간색 마스크 생성 (두 개의 범위 결합)
    cv::Mat redMask1 = getMask(image, lowerRed1, upperRed1);
    cv::Mat redMask2 = getMask(image, lowerRed2, upperRed2);
    cv::Mat redMask = redMask1 | redMask2; // 빨간색 범위 통합

    // 초록색 및 파란색 마스크 생성
    cv::Mat greenMask = getMask(image, lowerGreen, upperGreen);
    cv::Mat blueMask = getMask(image, lowerBlue, upperBlue);

    // 객체 검출 및 레이블링
    detectAndLabel(image, redMask, "Red Balloon", cv::Scalar(0, 0, 255));    // 빨간색 풍선
    detectAndLabel(image, greenMask, "Green Balloon", cv::Scalar(0, 255, 0)); // 초록색 풍선
    detectAndLabel(image, blueMask, "Blue Balloon", cv::Scalar(255, 0, 0));   // 파란색 풍선

    // 결과 이미지 표시
    cv::imshow("Detected Balloons", image);
    cv::waitKey(0);

    return 0;
}

