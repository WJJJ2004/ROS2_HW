#ifndef COIN_DETECTOR_HPP
#define COIN_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class CoinDetector {
public:
    CoinDetector(const std::string& imagePath);
    void detectCoins();
    int getTotalMoney() const;
    const cv::Mat& getProcessedImage() const;

private:
    cv::Mat src;
    cv::Mat src_gray;
    int money;
    std::vector<cv::Vec3f> circles;
};

#endif
