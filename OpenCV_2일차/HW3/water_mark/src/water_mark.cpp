#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

int main()
{
    // 이미지 로드
    cv::Mat seaside_image = cv::imread("/home/lwj/Documents/KakaoTalk Downloads/seaside_image.png");
    cv::Mat icon_image = cv::imread("/home/lwj/Documents/KakaoTalk Downloads/opencv_icon.png");

    // 이미지 로드 실패 체크
    if (seaside_image.empty() || icon_image.empty()) 
    {
        std::cout << "해변가 이미지 로딩 실패" << std::endl;
        return -1;
    }
	
    if (icon_image.empty()) 
    {
        std::cout << "아이콘 이미지 로딩 실패" << std::endl;
        return -1;
    }
	
    // 아이콘 이미지 크기를 해변 이미지의 일부 크기로 조절
    cv::Mat resized_icon;
    cv::resize(icon_image, resized_icon, cv::Size(seaside_image.cols / 4, seaside_image.rows / 4));

    cv::Point location(seaside_image.cols - resized_icon.cols - 10, seaside_image.rows - resized_icon.rows - 10);

    cv::Mat roi = seaside_image(cv::Rect(location.x, location.y, resized_icon.cols, resized_icon.rows));

    // 이미지 합성
    cv::addWeighted(resized_icon, 0.5, roi, 1.0, 0.0, roi);

    cv::imshow("Seaside with Icon", seaside_image);
    cv::waitKey(0);

    return 0;
}

