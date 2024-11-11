#include "coin_detector.hpp"
#include <iostream>

CoinDetector::CoinDetector(const std::string& imagePath) : money(0) 
{
    src = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (src.empty()) 
    {
        throw std::runtime_error("이미지를 로드할 수 없습니다.");
    }
}

void CoinDetector::detectCoins() 
{
    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
    cv::blur(src_gray, src_gray, cv::Size(7, 7));
    cv::HoughCircles(src_gray, circles, cv::HOUGH_GRADIENT, 1, 30, 150, 60);

    for (const auto& c : circles) 
    {
        cv::Point center(cvRound(c[0]), cvRound(c[1]));
        int radius = cvRound(c[2]);
        cv::circle(src, center, radius, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

        if (radius > 80) 
        {
            std::cout << "500원입니다. 반지름: " << radius << std::endl;
            money += 500;
        } 
        else if (radius > 60) 
        {
            std::cout << "100원입니다. 반지름: " << radius << std::endl;
            money += 100;
        } 
        else 
        {
            std::cout << "10원입니다. 반지름: " << radius << std::endl;
            money += 10;
        }
    }
}

int CoinDetector::getTotalMoney() const 
{
    return money;
}

const cv::Mat& CoinDetector::getProcessedImage() const 
{
    return src;
}
