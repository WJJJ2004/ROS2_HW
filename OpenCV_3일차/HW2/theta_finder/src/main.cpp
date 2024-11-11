#include "ThetaFinder.h"

int main() 
{
    std::string imagePath = "../resources/image.jpg";
    ThetaFinder detector(imagePath);

    if (!detector.loadImage()) 
    {
        return -1;
    }

    detector.detectRedLines();
    detector.displayResults();

    return 0;
}

