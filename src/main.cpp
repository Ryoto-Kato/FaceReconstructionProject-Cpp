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
    const std::string depth_filename = std::string("/Depth/DepthBMP/depth_0001_s1_Smile.bmp");
    const std::string depth_txt_filename = std::string("/Depth/DepthKinect/depth_0001_s1_Smile.txt");

    FacialLandmarkExtractor landmark_extractor;
    landmark_extractor.process_image(dataset_folder + rgb_filename);
    std::vector<Eigen::Vector2i> landmarks = landmark_extractor.get_landmarks();

    // Load Depth map
    Eigen::MatrixXi depth_map(256, 256);

    std::ifstream file(dataset_folder + depth_txt_filename);
    if (!file.is_open()) {
        std::cout << "Unable to read depth file. Aborting " <<  std::endl;
        return -1;
    }

    uint x, y, depth;
    uint x_offset = 192;
    uint y_offset = 74;
    while (file >> x >> y >> depth)
    {
        depth_map(x-x_offset, y-y_offset) = depth;
    }

    // Kinect intrinsics TODO: verify that this is correct
    float dFx = 525.0;
    float dFy = 525.0;
    float dCx = 319.5f - x_offset;
    float dCy = 239.5f - y_offset;
    Matrix3f intrinsics;
    intrinsics << dFx, 0.0f,  dCx,
                 0.0f,  dFy,  dCy,
                 0.0f, 0.0f, 1.0f;

    // Backproject landmarks
    std::vector<Vector3f> camera_landmarks;
    for (Eigen::Vector2i landmark : landmarks)
    {
        Vector3f position_screen;
        position_screen << landmark.x(), landmark.y(), 1.0;

        camera_landmarks.push_back(intrinsics.inverse() * (depth_map(landmark.x(), landmark.y()) * position_screen));

        if (DEBUG_OUT_ENABLED)
        {
            std::cout << camera_landmarks.back()(0)
                    << " " << camera_landmarks.back()(1)
                    << " " << camera_landmarks.back()(2)
                    << std::endl;
        }
    }

    // Display landmarks on image
    if (DEBUG_OUT_ENABLED)
    {
        cv::Mat rbg_image = cv::imread(dataset_folder + rgb_filename);
        cv::Mat depth_image = cv::imread(dataset_folder + depth_filename);

        cv::Mat rgb_depth_blend_image;
        cv::addWeighted(rbg_image, 0.5, depth_image, 0.5, 1.0, rgb_depth_blend_image);

        // Draw circles on the landmarks
        for (Eigen::Vector2i landmark : landmarks)
        {
            cv::circle(rgb_depth_blend_image, cv::Point(landmark.x(), landmark.y()), 2, cv::Scalar(0, 255, 0), -1);
        }

        cv::imshow("Face with Landmarks", rgb_depth_blend_image);
        cv::waitKey(0); // Wait for a key press to close the window
        cv::destroyAllWindows();
    }

    return 0;
}
