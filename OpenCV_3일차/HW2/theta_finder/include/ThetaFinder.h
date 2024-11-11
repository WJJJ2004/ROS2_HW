#ifndef THETA_FINDER_H
#define THETA_FINDER_H

#include <opencv2/opencv.hpp>
#include <vector>

class ThetaFinder
{
public:
    // 생성자 및 소멸자
    ThetaFinder(const std::string& imagePath);
    ~ThetaFinder() = default;

    // 이미지 로드 및 처리
    bool loadImage();
    void detectRedLines();
    void displayResults() const;

private:
    // 이미지 처리 관련 함수
    void filterRedColor();
    void detectEdges();
    void applyHoughTransform();
    void drawDetectedLines();

    // 데이터 멤버
    cv::Mat image;
    cv::Mat hsvImage;
    cv::Mat redMask;
    cv::Mat edges;
    cv::Mat colorImage;
    std::vector<cv::Vec4i> lines;

    std::string imagePath;
};

#endif // THETA_FINDER_H

