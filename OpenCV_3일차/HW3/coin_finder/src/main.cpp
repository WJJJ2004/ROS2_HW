#include "coin_detector.hpp"
#include <iostream>

int main() 
{
    CoinDetector detector("../resources/image.png");
    detector.detectCoins();
    std::cout << "총 금액: " << detector.getTotalMoney() << "원" << std::endl;
    cv::imshow("Detected Coins", detector.getProcessedImage());
    cv::waitKey(0);

    return 0;
}
