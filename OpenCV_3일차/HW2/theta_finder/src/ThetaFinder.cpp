// ThetaFinder.cpp

#include "ThetaFinder.h"
#include <iostream>
#include <cmath>

// 생성자: 이미지 경로를 초기화합니다.
ThetaFinder::ThetaFinder(const std::string& imagePath) : imagePath(imagePath) {}

// 이미지 로드
bool ThetaFinder::loadImage() 
{
    image = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (image.empty()) 
    {
        std::cerr << "이미지 로딩 실패" << std::endl;
        return false;
    }
    return true;
}

// 붉은색 필터링
void ThetaFinder::filterRedColor() 
{
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
    
    cv::Mat redMask1, redMask2;
    cv::inRange(hsvImage, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), redMask1);
    cv::inRange(hsvImage, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), redMask2);
    
    cv::bitwise_or(redMask1, redMask2, redMask);
}

// Canny 엣지 검출
void ThetaFinder::detectEdges() 
{
    cv::Canny(redMask, edges, 50, 150, 3);
}

// 허프 변환을 통한 선 검출
void ThetaFinder::applyHoughTransform() 
{
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);
    colorImage = image.clone();
}

// 검출된 선을 그리기
void ThetaFinder::drawDetectedLines()
{
    for (size_t i = 0; i < lines.size(); i++) 
    {
        cv::Vec4i l = lines[i];
        cv::line(colorImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        // 기울기 계산
        double dx = l[2] - l[0];
        double dy = l[3] - l[1];
        double slope = (dx != 0) ? dy / dx : 1e10;

        if (slope < 0) 
        {
            slope *= -1.0;
        }
        slope *= 180.0 / M_PI;
        while (slope > 180.0) 
        {
            slope -= 180.0;
        }

        // 기울기 출력
        std::cout << "Line " << i + 1 << ": (" << l[0] << ", " << l[1] << ") to (" << l[2] << ", " << l[3] << ")";
        std::cout << " - Slope: " << slope << std::endl;
    }
}

// 전체 라인 검출 과정 수행
void ThetaFinder::detectRedLines() 
{
    filterRedColor();
    detectEdges();
    applyHoughTransform();
    drawDetectedLines();
}

// 결과 이미지 디스플레이
void ThetaFinder::displayResults() const 
{
    cv::imshow("Canny Edges", edges);
    cv::imshow("Detected Lines", colorImage);
    cv::waitKey(0);
}

