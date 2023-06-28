/*
 * Combining the bfm_init and Pose Estimation part
*/
#include <algorithm>
#include "FPC_ICPOptimizer.h"
#include "BFM.h"
#include <iostream>
#include <fstream>
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
#define ps_ICP false
#define ps_ICP_with_allAvailableLandmarks false


int main(int argc, char *argv[])
{
    //init BFM
    std::cout<<left_line<<"Start init of BFM"<<right_line<<std::endl;
    BFM bfm;

    if(!bfm.init(argc, argv)){
        std::cout<<"ERROR in the init of BFM"<<std::endl;
    }else{
        std::cout<<left_line<<"Finish init of BFM-------"<<right_line<<std::endl;
    }

    std::cout<<left_line<<"Getting landmarks from BFM..."<<right_line<<std::endl;
    //Get landmark vertex position and id mapping individually
    std::vector<std::pair<int, int>> map_Dlib2BFM_landmark = bfm.getMapDlib2BFM_landmark();
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
    //top right original coordinate = (447, 329)
    uint x, y, depth;
    uint x_offset = 192;
    uint y_offset = 74;
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

    // Kinect intrinsics TODO: verify that this is correct
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

    Matrix4f depth_extrinsics = Matrix4f::Identity(4, 4);
    // Backproject landmarks

    // Save camera coordinate of the detected landmark on image plane in the camera_landmarks
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

    // Remove invalid points (Depth measurement available zero)
    unsigned int counter = 0;
    unsigned int num_available_landmarks = 0;
    std::vector<bool> available_landmark_mask(original_num_landmarks);
    unsigned int num_truncated_landmarks = 0;
    std::cout<<"Depth of detected landmarks"<<std::endl;

    //Map 68 landmark id to id in available landmark set
    std::map<int, int> map_LMid2ALMid;
    int id_available_landmark = 0;

    while (counter < dlib_landmarks_vertexPos.size()) {
        if (dlib_landmarks_vertexPos[counter].z() <= 0.0) {
            // if the depth of the dlib landmark is 0, we eliminate the point (remove outlier)
            std::cout<<counter<<"th landmark"<<std::endl;
            std::cout<<dlib_landmarks_vertexPos[counter]<<std::endl;
            // dlib_landmarks_vertexPos.erase(dlib_landmarks_vertexPos.begin() + i);
            // bfm_landmarks_vertexPos.erase(bfm_landmarks_vertexPos.begin() + i);
            // face_landmarks.erase(face_landmarks.begin() + i);
            // dlib_landmarks.erase(dlib_landmarks.begin()+i);
            available_landmark_mask[counter] = false;
            map_LMid2ALMid[counter] = -1;
            num_truncated_landmarks++;
        }else{
            available_landmark_mask[counter] = true;
            map_LMid2ALMid[counter] = id_available_landmark;
            id_available_landmark++;
        }
        ++counter;
    }

    #ifdef DEBUG
    std::cout<<"Map landmark id to available landmark id"<<std::endl;
    for(auto & pair : map_LMid2ALMid){
        std::cout<<pair.first<<": "<<pair.second<<std::endl;
    }
    #endif

    std::vector<Vector3f> available_bfm_landmarks_vertexPos;
    std::vector<Landmarks> available_face_landmarks;
    std::vector<Vector3f> available_dlib_landmarks_vertexPos;
    std::vector<Vector2i> available_dlib_landmarks;


    for(unsigned int j = 0; j < original_num_landmarks; j++){
        if(available_landmark_mask[j]){
            available_bfm_landmarks_vertexPos.push_back(bfm_landmarks_vertexPos[j]);
            available_dlib_landmarks_vertexPos.push_back(dlib_landmarks_vertexPos[j]);
            available_face_landmarks.push_back(face_landmarks[j]);
            available_dlib_landmarks.push_back(dlib_landmarks[j]);
        }
    }

    num_available_landmarks = original_num_landmarks - num_truncated_landmarks;
    
    //get landmark image Mat
    cv::Mat landmark_cvMat = landmarks_extractor.get_landmarkImage(dlib_landmarks, DEBUG);
    //write landmark image
    cv::imwrite("detected_landmark.png", landmark_cvMat);
    //Get triangle list by delaunay triangulation
    std::vector<Vector3i> dlib_landmark_triangleIdLists = landmarks_extractor.get_triList_landmarks(landmark_cvMat, dlib_landmarks, DEBUG);
    std::vector<Vector3i> bfm_landmark_triangleIdLists = dlib_landmark_triangleIdLists;

    bfm.write_from_PointCloud_TriangleList("../output/withMesh_truncated_landmark_bfm.ply", dlib_landmarks_vertexPos, dlib_landmark_triangleIdLists);

    // vertex position list and 2d coordinate of landmark list for pose estimation
    std::vector<Vector3f> ps_bfm_landmarks_vertexPos;
    std::vector<Landmarks> ps_face_landmarks;
    std::vector<Vector3f> ps_dlib_landmarks_vertexPos;
    std::vector<Vector2i> ps_dlib_landmarks;

    // Test procrustes with less correspondences
    /*
    * Try with less correspondence
    * only consider some of detected/available (valid, non-zero depth) landmark
    */
    
    if(ONLY_SELECTED_LANDMARKS){
        /*
        * select from range 0-67 (from 0 to 67)
        * e.g.,  std::vector<int> considered_landmarks_Ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 25, 26, 27, 28, 29, 30, 34, 35, 38, 41, 44, 47, 49, 51, 53, 55, 57};
        * 
        */ 
        std::vector<int> considered_landmarks_Ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 19, 24};

        #ifdef DEBUG
        std::cout<<"ONLY consider "<<considered_landmarks_Ids.size()<<" landmarks"<<std::endl;
        #endif

        std::vector<bool> landmark_mask(num_available_landmarks);
        std::fill(landmark_mask.begin(), landmark_mask.end(), false);

        for(auto & id : considered_landmarks_Ids){
            int _landmark_id = map_LMid2ALMid[id];
            if(_landmark_id!=-1){
                landmark_mask[map_LMid2ALMid[id]] = true;
            }
        }

        for(unsigned int i = 0; i < num_available_landmarks; i++){
            if(landmark_mask[i]){
                ps_dlib_landmarks_vertexPos.push_back(available_dlib_landmarks_vertexPos[i]);
                ps_bfm_landmarks_vertexPos.push_back(available_bfm_landmarks_vertexPos[i]);
                ps_face_landmarks.push_back(available_face_landmarks[i]);
                ps_dlib_landmarks.push_back(available_dlib_landmarks[i]);
            }
        }
    }else{
        ps_bfm_landmarks_vertexPos = available_bfm_landmarks_vertexPos;
        ps_dlib_landmarks_vertexPos = available_dlib_landmarks_vertexPos;
        ps_face_landmarks = available_face_landmarks;
        ps_dlib_landmarks = available_dlib_landmarks;
    }

    //Check if the number of detected ladnmark and BFM registred landmarks are identical
    assert(ps_dlib_landmarks_vertexPos.size() == ps_bfm_landmarks_vertexPos.size() == ps_bfm_landmarks.size() == ps_dlib_landmarks.size() && "Number of landmarks extracted form image must be equal to the ones form BFM");

    unsigned int num_ps_landmark = 0;
    num_ps_landmark = ps_dlib_landmarks_vertexPos.size();
    std::cout<<num_ps_landmark<<std::endl;
    //Write bfm_landmark point cloud
    bfm.writeLandmarkPly("../output/landmark_bfm.ply", available_bfm_landmarks_vertexPos);

    //Write dlib_landmark point cloud
    std::cout<<"Write .ply given backprojected dlib landmarks"<<std::endl;
    bfm.writeLandmarkPly("../output/landmark_dlib.ply", available_dlib_landmarks_vertexPos);

    //get landmark image Mat
    cv::Mat ps_landmark_cvMat = landmarks_extractor.get_landmarkImage(ps_dlib_landmarks, DEBUG);
    //write landmark image
    cv::imwrite("PS_considered_landmarks.png", ps_landmark_cvMat);
    //Get triangle list by delaunay triangulation
    std::vector<Vector3i> ps_dlib_landmark_triangleIdLists = landmarks_extractor.get_triList_landmarks(ps_landmark_cvMat, ps_dlib_landmarks, DEBUG);
    std::vector<Vector3i> ps_bfm_landmark_triangleIdLists = ps_dlib_landmark_triangleIdLists;

    /* TODO: Check if the procrustes is implemented correctly
    Pose Estimation (Procrustes).... 
    * input
    *   Source point
    *       std::vector<Vector3f> dlib_landmarks_vertexPos;
    *   Target point
    *       std::vector<Vector3f> bfm_landmarks_vertexPos
    */

    // Create new instance for ProcrustesAligner
    ProcrustesAligner aligner;
    // Get SE(3)
    Matrix4f Procrustes_estimatedPose = aligner.estimatePose(ps_dlib_landmarks_vertexPos, ps_bfm_landmarks_vertexPos);

    #ifdef DEBUG
        std::cout<<"with Procrustes Estimate Pose"<<std::endl;
        std::cout<<Procrustes_estimatedPose<<std::endl;
    #endif

    // Get transformed BFM landmarks given estimatePose 
    std::vector<Vector3f> transformed_onlyProcrustes_available_bfm_landmarks_vertexPos;
    std::vector<Vector3f> transformed_ps_bfm_landmarks_vertexPos;
    bfm.apply_SE3_to_BFMLandmarks(Procrustes_estimatedPose, available_bfm_landmarks_vertexPos, transformed_onlyProcrustes_available_bfm_landmarks_vertexPos, DEBUG);
    bfm.apply_SE3_to_BFMLandmarks(Procrustes_estimatedPose, ps_bfm_landmarks_vertexPos, transformed_ps_bfm_landmarks_vertexPos, DEBUG);

    //Write BFM landmarks point cloud which is transformed only by Procrustes
    std::cout<<"Write .ply given only Procrustes transformed BFM landmarks"<<std::endl;
    bfm.writeLandmarkPly("../output/transformed_onlyProcrustes_landmark_bfm.ply", transformed_onlyProcrustes_available_bfm_landmarks_vertexPos);
    
    /* ICP for pose estimation
    *   Input
    *       BFM landmarks
    *             FacePointCloud bfm_FPC;
    *                   std::vector<Vertex> BFM_vertices; from std::vector<Vector3f> transformed_onlyProcrustes_bfm_landmarks_vertexPos;
    *                   [common] std::vector<Triangle> landmark_triangleIdLists; from std::vector<Vector3i> landmark_triangleIdLists 
    *       Dlib landmarks
    *             FacePointCloud dlib_FPC;
    *                   std::vector<Vertex> Dlib_vertices; from std::vector<Vector3f> dlib_landmarks_vertexPos;
    *                   [common] std::vector<Triangle> landmark_triangleIdLists; from std::vector<Vector3i> landmark_triangleIdLists 
    */

    // Create FacePointCloud instance for Sparse ICP between dlib_landmark and bfm_landmark which is transformed by Procrustes
    /* Select which ICP will you use
    *  bool ICP_with_allAvailableLandmarks = true;
    *  if ICP_with_allAvailableLandmarks = false, we use selected landmarks
    */

    #if ps_ICP == true

    #if ps_ICP_with_allAvailableLandmarks == false
        FacePointCloud FPC_dlib_landmark{ps_dlib_landmarks_vertexPos, ps_dlib_landmark_triangleIdLists};
        FacePointCloud FPC_bfm_landmark{transformed_ps_bfm_landmarks_vertexPos, ps_bfm_landmark_triangleIdLists};
    #else
        FacePointCloud FPC_dlib_landmark{available_dlib_landmarks_vertexPos, dlib_landmark_triangleIdLists};
        FacePointCloud FPC_bfm_landmark{transformed_onlyProcrustes_available_bfm_landmarks_vertexPos, bfm_landmark_triangleIdLists};
    #endif

    //check if the points are registered as members of the instance
    #ifdef DEBUG
        std::cout<<"FPC_dlib_landmark"<<std::endl;
        FPC_dlib_landmark.print_points_and_normals();
        std::cout<<"FPC_bfm_landmark"<<std::endl;
        FPC_bfm_landmark.print_points_and_normals();
    #endif


    bool _withExpression = false;

    // Create FacePointCloud instance for Dense ICP between Depthmap
    unsigned int downsampleFactor = 1;
    float maxDistance = 0.1f;
    FacePointCloud FPC_depthMap{depth_map, depth_intrinsics, depth_extrinsics, depth_width, depth_height, downsampleFactor, maxDistance};

    //check if the points are registered as members of the instance
    #ifdef DEBUG
        std::cout<<"FPC_depthMap"<<std::endl;
        FPC_depthMap.print_points_and_normals();
    #endif

    /*TODO: ICP for Pose estimation 
    * Point to point and point to plane
    */

	// Setup the optimizer.
	FPC_ICPOptimizer* landmark_optimizer = nullptr;

    // ceres optimizer
	landmark_optimizer = new FPC_CeresICPOptimizer();
	
	landmark_optimizer->setMatchingMaxDistance(1e6);

	if(USE_POINT_TO_PLANE){
		landmark_optimizer->usePointToPlaneConstraints(true);
		landmark_optimizer->setNbOfIterations(1000); //10
	}
	else{
		landmark_optimizer->usePointToPlaneConstraints(false);
		landmark_optimizer->setNbOfIterations(1000);
	}

    // Estimate Pose by the ceres optimizer 
	Matrix4f ICP_estimatedPose = landmark_optimizer->estimatePose(FPC_dlib_landmark, FPC_bfm_landmark);
	
	// check the estimated pose.
    #ifdef DEBUG
        std::cout<<"Estimate Pose"<<std::endl;
        std::cout<<ICP_estimatedPose<<std::endl;
    #endif

    // Get transformed BFM landmarks given estimatePose 
    std::vector<Vector3f> transformed_ProcrustesAndICP_bfm_landmarks_vertexPos;
    bfm.apply_SE3_to_BFMLandmarks(ICP_estimatedPose, transformed_onlyProcrustes_available_bfm_landmarks_vertexPos, transformed_ProcrustesAndICP_bfm_landmarks_vertexPos, DEBUG);

    //Write BFM landmarks point cloud which is transformed only by Procrustes
    std::cout<<"Write .ply given only Procrustes transformed BFM landmarks"<<std::endl;
    bfm.writeLandmarkPly("../output/transformed_ProcrustesAndICP_bfm_landmarks.ply", transformed_ProcrustesAndICP_bfm_landmarks_vertexPos);
    
    #endif
    /* TODO: Parameter Estimation
    
    
    */

    //Set parameters
    // Coefficients for shape (199dim)
    std::vector<double> _coef_shape = {1.35982, 4.58154, -0.144497, -5.60332, 0.0526005, 3.72205, -4.73335, 2.58471, 0.199888, 1.13355, -1.71451, 2.48621, -2.35147, -2.73202, -3.08397, -0.461363, -0.648689, 2.35612, 1.89773, -5.25898, 2.43594, -0.858787, -1.251, 1.14865, 4.48174, -0.543897, 2.25787, -3.51808, 3.35158, 0.757921, -3.79997, -0.192129, -1.22736, 1.57217, -1.46639, -5.86647, 1.74459, -3.33895, -0.554571, -0.914844, -0.213381, 5.30319, -0.455774, -2.91975, 6.91567, -3.33744, -1.96792, -0.618775, 1.08693, 4.48546, 3.61086, 2.27854, -2.39703, -1.70969, -0.559681, -0.332884, -1.94077, -0.131853, 2.4651, -0.795658, -1.92196, 0.227778, 7.95127, 1.32237, 7.73176, 1.94919, -0.972522, -2.42813, 3.07547, 1.78916, 1.73506, -3.17724, 3.47924, 0.504941, 5.09847, 0.965697, 0.156934, 3.10021, -5.67475, 0.186958, 2.0241, 2.60911, -0.00164837, 3.56536, 2.95706, -0.949758, 1.34018, -0.917555, -2.89864, -4.10212, -0.0647675, -0.0760473, -1.09079, 4.78679, -1.00966, -0.951466, -1.12815, -1.81088, 1.14092, -0.634901, 4.19393, -0.154778, -0.24172, -2.68672, -0.261226, 1.37849, 2.96354, -0.954373, -2.06333, -1.49177, -2.86876, -3.49933, 2.10861, -1.83566, -0.651453, -1.04734, 5.02986, 2.2641, -3.05282, 1.78832, 4.33105, 2.99618, -5.23611, 3.34163, 0.528679, -0.570485, 0.569117, 1.5007, 5.01634, -0.0640699, 1.54914, 3.06658, -0.928706, 0.749098, -1.33337, 0.680119, -3.28307, 5.44081, -3.97985, 1.51229, -1.74412, 0.584966, -5.18306, 0.670043, -0.0892343, 1.27419, 0.364499, -3.59402, -2.84663, -1.0695, -0.672312, 0.780548, -2.09034, -1.32798, 1.78423, -0.903917, -4.78654, -5.35615, 1.20542, 1.62399, 2.41616, -3.0187, 1.12814, -0.75492, 0.81099, -2.48641, -1.11013, 6.17187, -2.24184, -2.07369, 2.678, -1.39893, -4.37047, 1.63268, -1.67336, -3.05079, -6.13043, 0.60356, -0.57444, 1.38333, -5.78597, -1.77023, 3.77638, -0.127266, -0.0265435, -6.00795, 1.44405, 0.656326, -2.53438, -2.87057, 2.14332, -1.57009, 4.84567, 1.66648, -1.80637, 1.53991, 3.13979, -7.00785, 1.84581};
    // Coefficient for texture (199dim)
    std::vector<double> _coef_tex = {-0.939461, 0.807407, 0.0418675, -0.218482, 0.0777038, 0.0646725, 0.132211, 0.39221, -0.275034, 0.366957, 0.103293, -0.240571, 0.516328, -0.376104, 0.485397, -0.231068, -0.487349, 0.470209, -0.789866, 0.0262927, -0.424657, 0.693524, 0.0495422, -1.24146, -0.067015, -0.817428, 0.131106, 0.536999, 0.400566, -1.11455, -0.738186, 0.609291, -0.14634, -0.00507975, -0.0104612, -0.577561, -0.284545, -0.00201202, -0.338924, -0.532313, -1.00337, 0.808154, 0.0622376, 0.601659, 0.395609, -1.00979, -0.569426, -0.168853, 0.00367227, -0.147915, 0.939562, -0.13469, 0.0402932, 0.723185, 0.784846, -0.348009, -0.226573, -0.263135, -0.222904, 0.190179, 0.220754, 0.402016, -0.648363, 0.760669, 0.592709, -0.293168, -0.0339384, -0.0523662, -0.0152403, -0.685043, 0.386356, -0.64661, -0.223204, -0.720214, 0.106663, -0.226485, 0.943473, -0.399918, -1.05839, -0.280164, -0.548844, 0.500835, -0.46445, -0.221159, -0.272483, 0.541268, -0.180583, -0.335298, -0.753914, -0.945244, -0.51114, -0.755218, -0.578613, -0.00774594, -0.303722, -0.0712457, -0.0139913, 0.209496, 1.16992, 0.397189, 0.716339, 0.325273, -1.1157, -0.82799, -0.517678, -1.24194, 0.832834, 0.741728, -0.227121, -0.769922, 0.591518, -0.240251, 0.104513, 0.681891, 0.884008, -0.118074, -0.460336, 0.309614, 0.198575, -0.0223547, 0.621338, 0.334244, -0.00417056, 0.570264, -0.747836, 0.286313, -0.872641, 0.517807, 1.20792, 0.224479, -0.123916, -0.79099, -0.160249, 0.847176, -0.0720962, -0.586694, -0.0337559, -0.656616, -0.0854957, -1.25917, 0.676657, -0.0300606, -0.243738, 0.373281, 1.09505, -0.352706, 1.29842, -0.28218, 0.116789, -0.917646, -0.224453, -0.85551, -0.147545, 0.221046, 0.498491, -0.450215, 0.00495127, -0.912007, -0.194967, -0.248033, -0.74021, -0.275011, 0.386151, 0.0129647, -0.760389, -0.632965, 0.395228, -0.0788702, 0.777731, 0.0283813, 1.14026, -0.228086, 1.43991, -0.162239, 0.163396, 0.289925, 0.730762, -0.266334, -0.567568, -0.0414544, -0.165757, -1.34601, 0.138388, -0.480812, -0.995689, 0.929929, 0.0384844, 0.997379, 0.18316, -0.296618, -0.820188, -0.0731485, -0.384338, 0.931458, 0.297943, -0.744139, -0.257288, -0.0732985, 0.0552113};
    // Coefficient for expression (100dim)
    std::vector<double> _coef_exp = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //set coefficients
    bfm.setCoefs(_coef_shape, _coef_tex, _coef_exp);

    //Obtain parameter set
    Parameter_set SHAPE = bfm.getParameter_set_SHAPE();
    Parameter_set TEX = bfm.getParameter_set_TEX();
    Parameter_set EXP = bfm.getParameter_set_EXP();

    /* TODO: Write resulting face mesh
    
    
    */

    #ifdef DEBUG
        std::cout<<left_line<<"BFM Parameters and Components"<<right_line<<std::endl;
        // Printing all components is too expensive to print out, hence comment out now.

        std::cout<<left_line<<SHAPE.name<<right_line<<std::endl;
        std::cout<<SHAPE.name<<" MEAN, vector shape ="<<SHAPE.mean.rows()<<std::endl;
        // std::cout<<SHAPE.mean<<std::endl;
        std::cout<<SHAPE.name<<" VARIANCE, vector shape ="<<SHAPE.variance.rows()<<std::endl;
        // std::cout<<SHAPE.variance<<std::endl;
        std::cout<<SHAPE.name<<" Principal Components, matrix shape = ["<<SHAPE.pc.rows()<<", "<<SHAPE.pc.cols()<<"]"<<std::endl;
        std::cout<<SHAPE.name<<" Principal Components"<<std::endl;

        // std::cout<<SHAPE.pc<<std::endl;
        // std::cout<<left_line<<"Shape parameter"<<right_line<<std::endl;

        std::cout<<SHAPE.name<<" parameter, shape = "<<SHAPE.parameters.size()<<std::endl;
        // for(auto & params : SHAPE.parameters){
        //     std::cout<<params<<", ";
        // }
        // std::cout<<std::endl;

        std::cout<<left_line<<TEX.name<<right_line<<std::endl;
        std::cout<<TEX.name<<" MEAN, vector shape ="<<TEX.mean.rows()<<std::endl;
        // std::cout<<TEX.mean<<std::endl;
        std::cout<<TEX.name<<" VARIANCE, vector shape ="<<TEX.variance.rows()<<std::endl;
        // std::cout<<TEX.variance<<std::endl;
        std::cout<<TEX.name<<" Principal Components, matrix shape = ["<<TEX.pc.rows()<<", "<<TEX.pc.cols()<<"]"<<std::endl;
        std::cout<<TEX.name<<" Principal Components"<<std::endl;

        // PC matrix is too expensive to print out, hence comment out now.
        // std::cout<<SHAPE.pc<<std::endl;
        // std::cout<<left_line<<"Shape parameter"<<right_line<<std::endl;

        std::cout<<TEX.name<<" parameter, shape = "<<TEX.parameters.size()<<std::endl;
        // for(auto & params : TEX.parameters){
        //     std::cout<<params<<", ";
        // }
        // std::cout<<std::endl;

        std::cout<<left_line<<EXP.name<<right_line<<std::endl;
        std::cout<<EXP.name<<" MEAN, vector shape ="<<EXP.mean.rows()<<std::endl;
        // std::cout<<EXP.mean<<std::endl;
        std::cout<<EXP.name<<" VARIANCE, vector shape ="<<EXP.variance.rows()<<std::endl;
        // std::cout<<EXP.variance<<std::endl;
        std::cout<<EXP.name<<" Principal Components, matrix shape = ["<<EXP.pc.rows()<<", "<<EXP.pc.cols()<<"]"<<std::endl;
        std::cout<<EXP.name<<" Principal Components"<<std::endl;

        // std::cout<<SHAPE.pc<<std::endl;
        // std::cout<<left_line<<"Shape parameter"<<right_line<<std::endl;

        std::cout<<EXP.name<<" parameter, shape = "<<EXP.parameters.size()<<std::endl;

        // for(auto & params : EXP.parameters){
        //     std::cout<<params<<", ";
        // }
        // std::cout<<std::endl;
        std::cout<<left_line<<right_line<<std::endl;
    #endif

    //Get average face geometry
    //_withExpression = True; you will get average mesh with random expression, otherwise, with a eutral expression
    bfm.writeAveBFMmesh("../output/average.ply", _withExpression);

    //Get face mesh given parameters
    //_withExpression = True; you will get average mesh with random expression, otherwise, with a eutral expression
    bfm.writeBFMmesh("../output/gen_mesh_test1.ply", _coef_shape, _coef_tex, _coef_exp ,_withExpression);

	delete landmark_optimizer;

	return 0;
}
