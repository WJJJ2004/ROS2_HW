#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

int main()
{
    // 이미지 로드
    cv::Mat image = cv::imread("/home/lwj/Documents/KakaoTalk Downloads/contours_image.png", cv::IMREAD_GRAYSCALE);

    if (image.empty()) {
        std::cout << "이미지 로딩 실패" << std::endl;
        return -1;
    }

    // 이진화 처리
    cv::Mat binary_image;
    cv::threshold(image, binary_image, 127, 255, cv::THRESH_BINARY);

    // 컨투어 모드 및 이름 리스트 정의
    int modes[] = {cv::RETR_EXTERNAL, cv::RETR_LIST, cv::RETR_CCOMP, cv::RETR_TREE};
    std::string modeNames[] = {"RETR_EXTERNAL", "RETR_LIST", "RETR_CCOMP", "RETR_TREE"};

    for (int i = 0; i < 4; ++i) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        // 현재 모드로 외곽선 검출
        cv::findContours(binary_image, contours, hierarchy, modes[i], cv::CHAIN_APPROX_SIMPLE);

        // 외곽선 그리기
        cv::Mat contour_image = cv::Mat::zeros(image.size(), CV_8UC3);
        for (size_t j = 0; j < contours.size(); j++) {
            cv::drawContours(contour_image, contours, static_cast<int>(j), cv::Scalar(0, 255, 0), 2, cv::LINE_8, hierarchy);
        }

        // 현재 모드에 대한 위계 정보 출력
        std::cout << "Mode: " << modeNames[i] << std::endl;
        std::cout << "Hierarchy: " << std::endl;
        for (const auto& h : hierarchy) {
            std::cout << "[" << h[0] << ", " << h[1] << ", " << h[2] << ", " << h[3] << "]" << std::endl;
        }

        // 각 모드에 해당하는 외곽선 이미지 창을 표시
        cv::imshow("Contours - " + modeNames[i], contour_image);
    }

    cv::waitKey(0);
    return 0;
}
