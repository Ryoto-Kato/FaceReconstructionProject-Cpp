#include <iostream>
#include <fstream>

#include "../include/Eigen.h"
#include "../include/SimpleMesh.h"
#include "../include/FacialLandmarkExtractor.h"

// Includes for debugging only
#include <opencv2/imgproc.hpp>

const bool DEBUG_OUT_ENABLED = true;

int main()
{
    // const std::string filename = std::string("../Data/bunny.off");

    // SimpleMesh sourceMesh;
    // if (!sourceMesh.loadMesh(filename)) {
    // 	std::cout << "Mesh file wasn't read successfully at location: " << filename << std::endl;
    // 	return -1;
    // }

    // SimpleMesh resultingMesh = sourceMesh;
    // resultingMesh.writeMesh(std::string("result.off"));
    // std::cout << "Resulting mesh written." << std::endl;

    const std::string dataset_folder = std::string("../../dataset/EURECOM_Kinect_Face_Dataset/0001/s1");
    const std::string rgb_filename = std::string("/RGB/rgb_0001_s1_Smile.bmp");

    FacialLandmarkExtractor landmark_extractor;
    landmark_extractor.process_image(dataset_folder + rgb_filename);
    std::vector<Eigen::Vector2i> landmarks = landmark_extractor.get_landmarks();

    // Display landmarks on image
    if (DEBUG_OUT_ENABLED)
    {
        cv::Mat image = cv::imread(dataset_folder + rgb_filename);

        // Draw circles on the landmarks
        for (Eigen::Vector2i landmark : landmarks)
        {
            cv::circle(image, cv::Point(landmark.x(), landmark.y()), 2, cv::Scalar(0, 255, 0), -1);
        }

        cv::imshow("Face with Landmarks", image);
        cv::waitKey(0); // Wait for a key press to close the window
        cv::destroyAllWindows();
    }

    return 0;
}
