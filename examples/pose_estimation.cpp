/*
 * Combining the bfm_init and Pose Estimation part
*/
#include <iostream>
#include <fstream>

#include <algorithm>
#include "FPC_ICPOptimizer.h"
#include "BFM.h"
#include "Eigen.h"
#include "FacialLandmarkExtractor.h"

//Procrustes for pose estimation
#include "ProcrustesAligner.h"

//ICP for pose estimation
#include "FacePointCloud.h"

// Includes for debugging only
#include <opencv2/imgproc.hpp>

const std::string left_line = "--------";
const std::string right_line = "--------";

#define DEBUG true
#define USE_POINT_TO_PLANE false
#define ONLY_SELECTED_LANDMARKS true
#define ps_ICP true
#define ps_ICP_with_allAvailableLandmarks false

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

int main(int argc, char *argv[])
{
    // Kinect intrinsics TODO: verify that this is correct
    uint x_offset = 192;
    uint y_offset = 74;
    float dFx = 525.0;
    float dFy = 525.0;
    float dCx = 319.5f - x_offset;
    float dCy = 239.5f - y_offset;
    const unsigned int depth_width = 640;
    const unsigned int depth_height = 480;
    Matrix3f depth_intrinsics;
    depth_intrinsics << dFx, 0.0f,  dCx,
                 0.0f,  dFy,  dCy,
                 0.0f, 0.0f, 1.0f;

    //init BFM
    std::cout<<left_line<<"Start init of BFM"<<right_line<<std::endl;
    BFM bfm;

    if(!bfm.init(argc, argv, dFx, dFy, dCx, dCy)){
        std::cout<<"ERROR in the init of BFM"<<std::endl;
    }else{
        std::cout<<left_line<<"Finish init of BFM-------"<<right_line<<std::endl;
    }

    std::cout<<left_line<<"Getting landmarks from BFM..."<<right_line<<std::endl;
    //Get landmark vertex position and id mapping individually
    std::vector<std::pair<int, int>> map_Dlib2DepthmapBFM_landmark = bfm.getMapDlib2BFM_landmark();
    std::vector<Vector3f> bfm_landmarks_vertexPos = bfm.getLandmarkPos();
    //Get landmarks which containing relevant information within Landmark for each
    std::vector<Landmarks> face_landmarks = bfm.getLandmarks();

    unsigned int original_num_landmarks = bfm.getNumLandmarks();

    #ifdef DEBUG
        // check indivisually obtained landmark pair and vertex positions

        // unsigned int _counter_pair = 0;
        // std::cout<<"Num of landmarks: "<<original_num_landmarks<<std::endl;
        // std::cout<<left_line<<"Map dlib to BFM verticies"<<right_line<<std::endl;
        // int dlibId, bfmId;
        // for(auto & [dlibId, bfmId] : map_Dlib2BFM_landmark){
        //     std::cout<<_counter_pair<<": (dlibid, bfmid) = ("<<dlibId<<" , "<<bfmId<<")"<<std::endl;
        //     _counter_pair++;
        // }

        // unsigned int _counter_landmark = 0;
        // std::cout<<left_line<<"Landmark vertices coordinates"<<right_line<<std::endl;
        // for(auto & lv : bfm_landmarks_vertexPos){
        //     std::cout<<_counter_landmark<<"(x,y,z) = "<<lv.x()<<","<<lv.y()<<","<<lv.z()<<std::endl;
        //     _counter_landmark++;
        // }

        for(auto & landmark : face_landmarks){
            std::cout<<left_line<<landmark.id<<"th landmark"<<right_line<<std::endl;
            std::cout<<"(dlibId, bfmId) = "<<"("<<landmark.dlib_id<<", "<<landmark.bfm_id<<")"<<std::endl;
            std::cout<<"(x, y, z) = "<<"("<<landmark.v.position.x()<<", "<<landmark.v.position.y()<<", "<<landmark.v.position.z()<<")"<<std::endl;
            std::cout<<"(r, g, b) = "<<"("<<(int)landmark.v.color.x()<<", "<<(int)landmark.v.color.y()<<", "<<(int)landmark.v.color.z()<<")"<<std::endl;
        }
        std::cout<<left_line<<right_line<<std::endl;
    #endif

    std::cout<<left_line<<"Finish getting landmarks from BFM..."<<right_line<<std::endl;

    std::cout<<left_line<<"Pose estimation"<<right_line<<std::endl;

    // Pose estimation using the landmarks.
    // Load RGB and RGBD image which will be processed.

    std::cout<<"RGB and RGBD input now loading....";
    const std::string dataset_folder = std::string("../data/EURECOM_Kinect_Face_Dataset/0001/s1");
    const std::string rgb_filename = std::string("/RGB/rgb_0001_s1_Smile.bmp");
    const std::string depth_filename = std::string("/Depth/DepthBMP/depth_0001_s1_Smile.bmp");
    const std::string depth_txt_filename = std::string("/Depth/DepthKinect/depth_0001_s1_Smile.txt");
    std::cout<<"Done"<<std::endl;

    const std::string out_folder = std::string("../output/");

    std::cout<<"Launch the facial landmark detection on "<<dataset_folder+rgb_filename<<std::endl;
    FacialLandmarkExtractor landmarks_extractor;
    landmarks_extractor.process_image(dataset_folder + rgb_filename);

    //Get RGB
    cv::Mat rgb_cvMat = landmarks_extractor.getRGB_cvMat();

    //Get matrix each element containing vector3uc
    MatrixRGB mat_rgb= landmarks_extractor.getRGB_EigenMat();
    //Get matrix each element containing vector4uc
    MatrixRGBA mat_rgba= landmarks_extractor.getRGBA_EigenMat();

    //containing the pixel coordinate of the facial landmark
    std::vector<Vector2i> dlib_landmarks = landmarks_extractor.get_landmarks();

    //Load depth map
    MatrixXf depth_map(256, 256);

    std::ifstream file(dataset_folder + depth_txt_filename);
    if (!file.is_open()) {
        std::cout << "Unable to read depth file. Aborting " <<  std::endl;
        return -1;
    }

    //Since EURECOM croped kinect RGB image and RGBD image
    //size = [256, 256]
    //top left original coordinate = (192, 74)
    //bottom right original coordinate = (447, 329)
    uint x, y, depth;
    while (file >> x >> y >> depth)
    {
        //subtract offset (left original coordinate) and map pixel coords (x, y) to [0, 256]
        //assign depth value to the corresponding element
        depth_map(x-x_offset, y-y_offset) = depth;
    }

    #ifdef DEBUG
        std::cout<<left_line<<"RGB image content with depth (r,g,b, depth[mm])"<<right_line<<std::endl;

        /*
        * check the raw data by reading cv::Mat rgb_cvMat
        */
        int rows = rgb_cvMat.rows;
        int cols = rgb_cvMat.cols;

        // for(unsigned int r = 0; r < rows; r++){
        //     for(unsigned int c = 0; c<cols; c++){
        //         cv::Vec3b intensity = rgb_cvMat.at<cv::Vec3b>(r, c);
        //         unsigned char blue = intensity.val[0];
        //         unsigned char green = intensity.val[1];
        //         unsigned char red = intensity.val[2];
        //         std::cout<<"("<<(int)blue<<","<<(int)green<<","<<(int)red<<")"<<", ";
        //     }
        //     std::cout<<std::endl;
        // }

        /*
        * check the retrieved rgb channel reading Eigen::MatrixRGB or Eigen::MatrixRGBA
        */

        for(unsigned int r = 0; r <rows; r++){
            for(unsigned int c = 0; c <cols; c++){
                unsigned char blue = mat_rgb(r, c).x();
                unsigned char green = mat_rgb(r, c).y();
                unsigned char red = mat_rgb(r, c).z();
                std::cout<<"("<<(int)blue<<","<<(int)green<<","<<(int)red<<","<<depth_map(r, c)<<")"<<",\t";
            }
            std::cout<<std::endl;
        }
    #endif

    Matrix4f depth_extrinsics = Matrix4f::Identity(4, 4);
    // Backproject landmarks

    // Save camera coordinate of the detected landmark on image plane in the camera frame which is treated as world here
    std::vector<Vector3f> dlib_landmarks_vertexPos;
    for (Eigen::Vector2i pix_landmark : dlib_landmarks)
    {
        Vector3f position_screen;
        position_screen << pix_landmark.x(), pix_landmark.y(), 1.0;

        //backprojection
        dlib_landmarks_vertexPos.push_back(depth_intrinsics.inverse() * (depth_map(pix_landmark.x(), pix_landmark.y()) * position_screen));
    }

    std::cout<<"Backprojected dlib facial landmarks detected on "<<dataset_folder + rgb_filename<<std::endl;

    //Check if the number of detected ladnmark and BFM registred landmarks are identical
    assert(dlib_landmarks_vertexPos.size() == bfm_landmarks_vertexPos.size() && "Number of landmarks extracted form image must be equal to the ones form BFM");

    // Generate BFM Model
    // std::string bfm_h5_path = "../BFM_basic_pipeline/Data/model2017-1_bfm_nomouth.h5";
    // std::string landmark_id_path = "../BFM_basic_pipeline/Data/map_dlib-bfm_rk.anl";
    // std::unique_ptr<BfmManager> bfm_manager (new BfmManager(bfm_h5_path, std::array<double, 4>{dFx, dFy, dCx, dCy}, landmark_id_path));
    BfmManager * bfm_manager = bfm.getBfmmanager();

    // Get landmarks form BFM
    std::vector<Vector3f> bfm_landmarks;
    for (size_t i = 0; i < bfm_manager->getMapLandmarkIndices().size(); i++)
    {
        bfm_landmarks.push_back(Vector3f(bfm_manager->getLandmarkCurrentBlendshape()(i * 3),
                                         bfm_manager->getLandmarkCurrentBlendshape()(i * 3 + 1),
                                         bfm_manager->getLandmarkCurrentBlendshape()(i * 3 + 2)));
    }

    assert(dlib_landmarks_vertexPos.size() == bfm_landmarks.size() && "Number of landmarks extracted form image must be equal to the ones form BFM");

    // Remove invalid points (Depth measurement available zero)
    size_t i = 0;
    while (i < dlib_landmarks_vertexPos.size()) {
        if (dlib_landmarks_vertexPos[i](2) == 0) {
            dlib_landmarks_vertexPos.erase(dlib_landmarks_vertexPos.begin() + i);
            bfm_landmarks.erase(bfm_landmarks.begin() + i);
        }
        else {
            ++i;
        }
    }

    #ifdef DEBUG
        std::cout<<"Dlib_landmarks: "<<dlib_landmarks_vertexPos.size()<<std::endl;
        for(unsigned int i = 0; i<dlib_landmarks_vertexPos.size(); i++){
            std::cout<<i<<": "<<dlib_landmarks_vertexPos[i]<<std::endl;
        }

        std::cout<<"BFM_landmarks: "<<bfm_landmarks.size()<<std::endl;
        for(unsigned int i = 0; i<bfm_landmarks.size(); i++){
            std::cout<<i<<": "<<bfm_landmarks[i]<<std::endl;
        }

    #endif

    writeLandmarkPly((out_folder + "dlib_landmarks.ply"), dlib_landmarks_vertexPos);
    bfm_manager->writeLandmarkPly(out_folder + "bfm_landmarks_before_procrustes.ply");

    // Perform procrustes
    ProcrustesAligner procrustes;
    Matrix4f procrustes_pose_estimate = procrustes.estimatePose(bfm_landmarks, dlib_landmarks_vertexPos);

    std::cout << "Pose Estimation: " << std::endl << procrustes_pose_estimate << std::endl;

    // Update BFM with new extrinsic parameters
    bfm_manager->setMatR(procrustes_pose_estimate.block<3,3>(0,0).cast<double>());
    bfm_manager->setVecT(procrustes_pose_estimate.block<3,1>(0,3).cast<double>());
    bfm_manager->genExtParams();
    bfm_manager->genFace();
    bfm_manager->genLandmarkBlendshape();

    // Get Transformed Landmarks
    std::vector<Vector3f> bfm_landmark_after_procrustes;
    for (size_t i = 0; i < bfm_manager->getMapLandmarkIndices().size(); i++)
    {
        bfm_landmark_after_procrustes.push_back(Vector3f(bfm_manager->getLandmarkCurrentBlendshapeTransformed()(i * 3),
                                                    bfm_manager->getLandmarkCurrentBlendshapeTransformed()(i * 3 + 1),
                                                    bfm_manager->getLandmarkCurrentBlendshapeTransformed()(i * 3 + 2)));
    }
    writeLandmarkPly((out_folder + "bfm_landmarks_after_procrustes.ply"), bfm_landmark_after_procrustes);

    // TODO: If we also want to use point to plane we need this
    std::vector<Vector3i> ps_dlib_landmark_triangleIdLists;
    std::vector<Vector3i> ps_bfm_landmark_triangleIdLists;

    FacePointCloud FPC_dlib_landmark{dlib_landmarks_vertexPos, ps_dlib_landmark_triangleIdLists};
    FacePointCloud FPC_bfm_landmark{bfm_landmarks, ps_bfm_landmark_triangleIdLists};

	// Setup the optimizer.
	FPC_CeresICPOptimizer* landmark_optimizer = nullptr;

    // ceres optimizer
	landmark_optimizer = new FPC_CeresICPOptimizer();

	landmark_optimizer->setMatchingMaxDistance(1e3);

	if(USE_POINT_TO_PLANE){
		landmark_optimizer->usePointToPlaneConstraints(true);
		landmark_optimizer->setNbOfIterations(1000); //10
	}
	else{
		landmark_optimizer->usePointToPlaneConstraints(false);
		landmark_optimizer->setNbOfIterations(1000);
	}

    // Estimate Pose by the ceres optimizer
	Matrix4f ICP_estimatedPose = landmark_optimizer->estimatePose(FPC_bfm_landmark, FPC_dlib_landmark, procrustes_pose_estimate);

	// check the estimated pose.
    #ifdef DEBUG
        std::cout<<"Estimate Pose"<<std::endl;
        std::cout<<ICP_estimatedPose<<std::endl;
    #endif

    // Update BFM with new extrinsic parameters
    bfm_manager->setMatR(ICP_estimatedPose.block<3,3>(0,0).cast<double>());
    bfm_manager->setVecT(ICP_estimatedPose.block<3,1>(0,3).cast<double>());
    bfm_manager->genExtParams();
    bfm_manager->genFace();
    bfm_manager->genLandmarkBlendshape();

    // Get Transformed Landmarks
    std::vector<Vector3f> bfm_landmark_after_icp;
    for (size_t i = 0; i < bfm_manager->getMapLandmarkIndices().size(); i++)
    {
        bfm_landmark_after_icp.push_back(Vector3f(bfm_manager->getLandmarkCurrentBlendshapeTransformed()(i * 3),
                                                    bfm_manager->getLandmarkCurrentBlendshapeTransformed()(i * 3 + 1),
                                                    bfm_manager->getLandmarkCurrentBlendshapeTransformed()(i * 3 + 2)));
    }
    writeLandmarkPly((out_folder + "bfm_landmarks_after_icp.ply"), bfm_landmark_after_icp);

	delete landmark_optimizer;

	return 0;
}