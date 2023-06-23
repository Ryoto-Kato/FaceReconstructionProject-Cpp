#include <iostream>
#include <fstream>

#include "../include/Eigen.h"
// #include "../include/SimpleMesh.h"
#include "../include/FacialLandmarkExtractor.h"
#include "../include/ProcrustesAligner.h"

#include "../BFM_basic_pipeline/src/bfm_manager.cpp"

// Includes for debugging only
#include <opencv2/imgproc.hpp>

const bool DEBUG_OUT_ENABLED = true;


//TODO: Move this to a appropriate place
void writeLandmarkPly(std::string fn, std::vector<Vector3f> & landmarks) {
	std::ofstream out;
	/* Note: In Linux Cpp, we should use std::ios::out as flag, which is not necessary in Windows */
	out.open(fn, std::ios::out | std::ios::binary);
	if(!out.is_open())
	{
		LOG(ERROR) << "Creation of " << fn << " failed.";
		return;
	}

	std::cout<<"Writing ply ...."<<std::endl;

	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "comment Made from the 3D Morphable Face Model of the Univeristy of Basel, Switzerland.\n";
	out << "element vertex " << landmarks.size() << "\n";
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "end_header\n";

	// unsigned long int cnt = 0;
	for (Vector3f landmark : landmarks)
	{
		out<<landmark.x()<<" "<<landmark.y()<<" "<<landmark.z()<<"\n";
	}

	out.close();
	std::cout<<"Finish write ply"<<std::endl;
}


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
    }

    // Generate BFM Model
    std::string bfm_h5_path = "../BFM_basic_pipeline/Data/model2017-1_bfm_nomouth.h5";
    std::string landmark_id_path = "../BFM_basic_pipeline/Data/map_dlib-bfm_rk.anl";
    std::unique_ptr<BfmManager> bfm_manager (new BfmManager(bfm_h5_path, std::array<double, 4>{dFx, dFy, dCx, dCy}, landmark_id_path));

    // Get landmarks form BFM
    std::vector<Vector3f> bfm_landmarks;
    for (size_t i = 0; i < bfm_manager->getMapLandmarkIndices().size(); i++)
    {
        bfm_landmarks.push_back(Vector3f(bfm_manager->getLandmarkCurrentBlendshape()(i * 3),
                                         bfm_manager->getLandmarkCurrentBlendshape()(i * 3 + 1),
                                         bfm_manager->getLandmarkCurrentBlendshape()(i * 3 + 2)));
    }

    assert(camera_landmarks.size() == bfm_landmarks.size() && "Number of landmarks extracted form image must be equal to the ones form BFM");


    // Remove invalid points (Depth measurement available zero)
    size_t i = 0;
    while (i < camera_landmarks.size()) {
        if (camera_landmarks[i](2) == 0) {
            camera_landmarks.erase(camera_landmarks.begin() + i);
            bfm_landmarks.erase(bfm_landmarks.begin() + i);
        }
        else {
            ++i;
        }
    }

    writeLandmarkPly("dlib_landmarks.ply", camera_landmarks);
    bfm_manager->writeLandmarkPly("bfm_landmarks_before_procrustes.ply");

    ProcrustesAligner procrustes;
    Matrix4f pose_estimate = procrustes.estimatePose(bfm_landmarks, camera_landmarks);

    std::cout << "Pose Estimation: " << std::endl << pose_estimate << std::endl;

    std::vector<Vector3f> bfm_landmark_transformed;
    for (Vector3f landmark : bfm_landmarks)



    // bfm_manager->setMatR(pose_estimate.block<3,3>(0,0).cast<double>());
    // bfm_manager->setVecT(pose_estimate.block<1,3>(0,3).cast<double>());

    // bfm_manager->genExtParams();
    // bfm_manager->genFace();
    // bfm_manager->genLandmarkBlendshape();

    // //write ply
    // bfm_manager->writeLandmarkPly("bfm_landmarks_after_procrustes.ply");


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
