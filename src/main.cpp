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
#include <tuple>
#include "Optimizer-ALL.h"

// Procrustes for pose estimation
#include "ProcrustesAligner.h"

// ICP for pose estimation
#include "FacePointCloud.h"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

// Includes for debugging only
// #include <opencv2/imgproc.hpp>

const std::string left_line = "--------";
const std::string right_line = "--------";

#define ONLY_SELECTED_LANDMARKS true
#define ps_ICP true
#define USE_POINT_TO_PLANE true
#define BRUTEFORCE_MATCH_GEO false
#define BRUTEFORCE_MATCH_TEX false
#define FACEREANACTMENT true

namespace fs = boost::filesystem;
namespace po = boost::program_options;

const std::string LOG_PATH = R"(./log)";

// We need to be careful matrix entry index
// Every time you should check if indices are correct, there are different way especially Opencv
/* u => column id, v => row id
    - OpenCV
        - cv::Mat rgb_cvMat (v, u)
        - cv::Mat landmark_cvMat (v, u)
    - Original
        - MatrixRGB mat_rgb(u, v)
        - MatrixRGBA mat_rgba(u, v)
        - MatrixXf depth_map(u, v)
        - MatrixNormalMap normalmap(u, v)
*/

int main(int argc, char *argv[])
{
    // Declare the supported options.
    po::options_description desc("Allowed options (all optional)");
    desc.add_options()
        ("help,h", "produce this help message")
        ("person_id,p", po::value<int>()->default_value(42), "person identifier")
        ("session,s", po::value<int>()->default_value(1), "session number (1 or 2)")
        ("expression,", po::value<std::string>()->default_value("Smile"), "face status (Neutral, Smile, OpenMouth, LeftProfile, RightProfile)")
    ;

    po::variables_map opts;
    po::store(po::parse_command_line(argc, argv, desc), opts);
    po::notify(opts);

    if (opts.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = false;
    if (fs::exists(LOG_PATH))
        fs::remove_all(LOG_PATH);
    fs::create_directory(LOG_PATH);
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = LOG_PATH;
    FLAGS_log_prefix = true;
    FLAGS_colorlogtostderr = true;

    // init BFM
    LOG(INFO) << left_line << "Init BFM" << right_line;
    BFM bfm;

    if (!bfm.init(argc, argv))
    {
        std::cout << "ERROR in the init BFM" << std::endl;
    }
    else
    {
        std::cout << left_line << "Finish init of BFM-------" << right_line << std::endl;
    }

    std::cout << left_line << "Getting landmarks from BFM..." << right_line << std::endl;
    // Get landmark vertex position and id mapping individually
    std::vector<std::pair<int, int>> map_Dlib2DepthmapBFM_landmark = bfm.getMapDlib2BFM_landmark();
    std::vector<Vector3f> bfm_landmarks_vertexPos = bfm.getLandmarkPos();
    // Get landmarks which containing relevant information within Landmark for each
    std::vector<Landmarks> face_landmarks = bfm.getLandmarks();
    unsigned int original_num_landmarks = bfm.getNumLandmarks();

    // Obtain parameter set
    Parameter_set SHAPE = bfm.getParameter_set_SHAPE();
    Parameter_set TEX = bfm.getParameter_set_TEX();
    Parameter_set EXP = bfm.getParameter_set_EXP();

    bool _withExpression = false;

    // Get average face geometry (.ply) and its components
    //_withExpression = True; you will get average mesh with random expression, otherwise, with a neutral expression

    std::vector<Eigen::Vector3f> averageBFM_vertex_pos;
    std::vector<Eigen::Vector3f> averageBFM_vertex_rgb;
    std::vector<Vector3i> averageBFM_triangle_list;

    std::tie(averageBFM_vertex_pos, averageBFM_vertex_rgb, averageBFM_triangle_list) = bfm.writeAveBFMmesh("../output/average.ply", _withExpression);

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

    for (auto &landmark : face_landmarks)
    {
        std::cout << left_line << landmark.id << "th landmark" << right_line << std::endl;
        std::cout << "(dlibId, bfmId) = "
                  << "(" << landmark.dlib_id << ", " << landmark.bfm_id << ")" << std::endl;
        std::cout << "(x, y, z) = "
                  << "(" << landmark.v.position.x() << ", " << landmark.v.position.y() << ", " << landmark.v.position.z() << ")" << std::endl;
        std::cout << "(r, g, b) = "
                  << "(" << (int)landmark.v.color.x() << ", " << (int)landmark.v.color.y() << ", " << (int)landmark.v.color.z() << ")" << std::endl;
    }
    std::cout << left_line << right_line << std::endl;
#endif

    std::cout << left_line << "Finish getting landmarks from BFM..." << right_line << std::endl;

    std::cout << left_line << "Pose estimation" << right_line << std::endl;

    // Pose estimation using the landmarks.
    // Load RGB and RGBD image which will be processed.
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << opts["person_id"].as<int>();
    const std::string person_id = ss.str();
    const std::string dataset_folder = "../data/EURECOM_Kinect_Face_Dataset/" + person_id + "/s" + std::to_string(opts["session"].as<int>()) ;
    const std::string rgb_filename = "/RGB/rgb_" + person_id + "_s" + std::to_string(opts["session"].as<int>()) + "_" + opts["expression"].as<std::string>() + ".bmp";
    const std::string depth_filename = "/Depth/DepthBMP/depth_" + person_id + "_s" + std::to_string(opts["session"].as<int>()) + "_" + opts["expression"].as<std::string>() + ".bmp";
    const std::string depth_txt_filename = "/Depth/DepthKinect/depth_" + person_id + "_s" + std::to_string(opts["session"].as<int>()) + "_" + opts["expression"].as<std::string>() + ".txt";

    std::cout << "Launch the facial landmark detection on " << dataset_folder + rgb_filename << std::endl;
    FacialLandmarkExtractor landmarks_extractor;
    landmarks_extractor.process_image(dataset_folder + rgb_filename);

    // Get RGB
    cv::Mat rgb_cvMat = landmarks_extractor.getRGB_cvMat();

    // Get matrix each element containing vector3uc
    MatrixRGB mat_rgb = landmarks_extractor.getRGB_EigenMat();
    // Get matrix each element containing vector4uc
    MatrixRGBA mat_rgba = landmarks_extractor.getRGBA_EigenMat();

    // containing the pixel coordinate of the facial landmark
    std::vector<Vector2i> dlib_landmarks = landmarks_extractor.get_landmarks();

    // Load depth map
    MatrixXf depth_map(256, 256);

    std::ifstream file(dataset_folder + depth_txt_filename);
    if (!file.is_open())
    {
        std::cout << "Unable to read depth file. Aborting " << std::endl;
        return -1;
    }

    // Since EURECOM croped kinect RGB image and RGBD image
    // size = [256, 256]
    // top left original coordinate = (192, 74)
    // bottom right original coordinate = (447, 329)
    double scaler_depth = 0.001;
    double depth_threshold = 1.0;
    uint x, y, depth;
    uint x_offset = 192;
    uint y_offset = 74;
    while (file >> x >> y >> depth)
    {
        // subtract offset (left original coordinate) and map pixel coords (x, y) to [0, 256]
        // assign depth value to the corresponding element
        depth_map(x - x_offset, y - y_offset) = depth;
    }

#ifdef DEBUG
    std::cout << left_line << "RGB image content with depth (r,g,b, depth[mm])" << right_line << std::endl;

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

    for (unsigned int r = 0; r < rows; r++)
    {
        for (unsigned int c = 0; c < cols; c++)
        {
            unsigned char blue = mat_rgb(c, r).x();
            unsigned char green = mat_rgb(c, r).y();
            unsigned char red = mat_rgb(c, r).z();
            std::cout << "(" << (int)blue << "," << (int)green << "," << (int)red << "," << depth_map(c, r) << ")"
                      << ",\t";
        }
        std::cout << std::endl;
    }
#endif

    // Kinect intrinsics TODO: verify that this is correct
    float dFx = 525.0;
    float dFy = 525.0;
    float dCx = 319.5f - x_offset;
    float dCy = 239.5f - y_offset;
    // original
    //  const unsigned int depth_width = 640;
    //  const unsigned int depth_height = 480;
    const unsigned int depth_width = 256;
    const unsigned int depth_height = 256;
    Matrix3f depth_intrinsics;
    depth_intrinsics << dFx, 0.0f, dCx,
        0.0f, dFy, dCy,
        0.0f, 0.0f, 1.0f;

    Matrix4f depth_extrinsics = Matrix4f::Identity(4, 4);
    // Backproject landmarks

    // Save camera coordinate of the detected landmark on image plane in the camera_landmarks
    std::vector<Vector3f> dlib_landmarks_vertexPos;
    for (Eigen::Vector2i pix_landmark : dlib_landmarks)
    {
        Vector3f position_screen;
        position_screen << pix_landmark.x(), pix_landmark.y(), 1.0;

        // backprojection)
        dlib_landmarks_vertexPos.push_back(depth_intrinsics.inverse() * (depth_map(pix_landmark.x(), pix_landmark.y()) * position_screen));
    }

    std::cout << "Backprojected dlib facial landmarks detected on " << dataset_folder + rgb_filename << std::endl;

    // Check if the number of detected ladnmark and BFM registred landmarks are identical
    assert(dlib_landmarks_vertexPos.size() == bfm_landmarks_vertexPos.size() && "Number of landmarks extracted form image must be equal to the ones form BFM");

    // Remove invalid points (Depth measurement available zero)
    unsigned int counter = 0;
    unsigned int num_available_landmarks = 0;
    std::vector<bool> available_landmark_mask(original_num_landmarks);
    unsigned int num_truncated_landmarks = 0;
    std::cout << "Depth of detected landmarks" << std::endl;

    // get the index list in the bfm vertex
    std::vector<std::pair<int, int>> map_dlib_bfmLandmarks = bfm.get_bfmLandmarks_indexList();
    std::vector<int> bfm_landmarkIndex_list;

    // Map 68 landmark id to id in available landmark set
    std::map<int, int> map_LMid2ALMid;
    int id_available_landmark = 0;

    while (counter < dlib_landmarks_vertexPos.size())
    {
        // if the depth of the dlib landmark is 0, we eliminate the point (remove outlier)
        if (dlib_landmarks_vertexPos[counter].z() <= 0.0)
        {
            available_landmark_mask[counter] = false;
            map_LMid2ALMid[counter] = -1;
            map_dlib_bfmLandmarks[counter].second = -1;
            num_truncated_landmarks++;
        }
        else
        {
            available_landmark_mask[counter] = true;
            map_LMid2ALMid[counter] = id_available_landmark;
            id_available_landmark++;
            bfm_landmarkIndex_list.push_back(map_dlib_bfmLandmarks[counter].second);
        }
        ++counter;
    }

#ifdef DEBUG

    std::cout << "list of index of bfm mesh" << std::endl;
    for (const auto &[dlibIdx, bfmIdx] : map_dlib_bfmLandmarks)
    {
        std::cout << "dlibIdx, bfmIdx" << dlibIdx << "," << bfmIdx << std::endl;
    }
    std::cout << "Map landmark id to available landmark id" << std::endl;
    for (auto &pair : map_LMid2ALMid)
    {
        std::cout << pair.first << ": " << pair.second << std::endl;
    }
#endif

    std::vector<Vector3f> available_bfm_landmarks_vertexPos;
    std::vector<Landmarks> available_face_landmarks;
    std::vector<Vector3f> available_dlib_landmarks_vertexPos;
    std::vector<Vector2i> available_dlib_landmarks;

    for (unsigned int j = 0; j < original_num_landmarks; j++)
    {
        if (available_landmark_mask[j])
        {
            available_bfm_landmarks_vertexPos.push_back(bfm_landmarks_vertexPos[j]);
            available_dlib_landmarks_vertexPos.push_back(dlib_landmarks_vertexPos[j]);
            available_face_landmarks.push_back(face_landmarks[j]);
            available_dlib_landmarks.push_back(dlib_landmarks[j]);
        }
    }

#ifdef DEBUG
    std::cout << "Dlib_landmarks: " << available_dlib_landmarks_vertexPos.size() << std::endl;
    for (unsigned int i = 0; i < available_dlib_landmarks_vertexPos.size(); i++)
    {
        std::cout << i << ": " << available_dlib_landmarks_vertexPos[i] << std::endl;
    }

    std::cout << "BFM_landmarks: " << available_bfm_landmarks_vertexPos.size() << std::endl;
    for (unsigned int i = 0; i < available_bfm_landmarks_vertexPos.size(); i++)
    {
        std::cout << i << ": " << available_bfm_landmarks_vertexPos[i] << std::endl;
    }

#endif

    num_available_landmarks = original_num_landmarks - num_truncated_landmarks;

    // get landmark image Mat
    cv::Mat landmark_cvMat = landmarks_extractor.get_landmarkImage(dlib_landmarks, false);
    // write landmark image
    cv::imwrite("detected_landmark.png", landmark_cvMat);
    // Get triangle list by delaunay triangulation
    std::vector<Vector3i> dlib_landmark_triangleIdLists = landmarks_extractor.get_triList_landmarks(landmark_cvMat, dlib_landmarks, false);
    std::vector<Vector3i> bfm_landmark_triangleIdLists = dlib_landmark_triangleIdLists;

    bfm.write_from_PointCloud_TriangleList("../output/withMesh_truncated_landmark_bfm.ply", dlib_landmarks_vertexPos, dlib_landmark_triangleIdLists);

    // vertex position list and 2d coordinate of landmark list for pose estimation
    std::vector<Vector3f> ps_bfm_landmarks_vertexPos;
    std::vector<Landmarks> ps_face_landmarks;
    std::vector<Vector3f> ps_dlib_landmarks_vertexPos;
    std::vector<Vector2i> ps_dlib_landmarks;

    // Selection of landmarks
    /*
     * Try with less correspondence for the higher probability to have all inliers
     * only consider some of detected/available (valid, non-zero depth) landmark
     */

    if (ONLY_SELECTED_LANDMARKS)
    {
        /*
         * select from range 0-66 (from 0 to 66)
         * e.g.,  std::vector<int> considered_landmarks_Ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 25, 26, 27, 28, 29, 30, 34, 35, 38, 41, 44, 47, 49, 51, 53, 55, 57};
         *
         */
        // std::vector<int> considered_landmarks_Ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 19, 24};
        std::vector<int> considered_landmarks_Ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 64}; //,
#ifdef DEBUG
        std::cout << "ONLY consider " << considered_landmarks_Ids.size() << " landmarks" << std::endl;
#endif

        std::vector<bool> landmark_mask(num_available_landmarks);
        std::fill(landmark_mask.begin(), landmark_mask.end(), false);

        for (auto &id : considered_landmarks_Ids)
        {
            int _landmark_id = map_LMid2ALMid[id];
            if (_landmark_id != -1)
            {
                landmark_mask[map_LMid2ALMid[id]] = true;
            }
        }

        for (unsigned int i = 0; i < num_available_landmarks; i++)
        {
            if (landmark_mask[i])
            {
                ps_dlib_landmarks_vertexPos.push_back(available_dlib_landmarks_vertexPos[i]);
                ps_bfm_landmarks_vertexPos.push_back(available_bfm_landmarks_vertexPos[i]);
                ps_face_landmarks.push_back(available_face_landmarks[i]);
                ps_dlib_landmarks.push_back(available_dlib_landmarks[i]);
            }
        }
    }
    else
    {
        ps_bfm_landmarks_vertexPos = available_bfm_landmarks_vertexPos;
        ps_dlib_landmarks_vertexPos = available_dlib_landmarks_vertexPos;
        ps_face_landmarks = available_face_landmarks;
        ps_dlib_landmarks = available_dlib_landmarks;
    }

    // Check if the number of detected ladnmark and BFM registred landmarks are identical
    assert(ps_dlib_landmarks_vertexPos.size() == ps_bfm_landmarks_vertexPos.size() == ps_face_landmarks.size() == ps_dlib_landmarks.size() && "Number of landmarks extracted form image must be equal to the ones form BFM");

    #ifdef DEBUG
        std::cout << "Check number of landmarks" << std::endl;
        std::cout << ps_dlib_landmarks_vertexPos.size() << std::endl;
        std::cout << ps_bfm_landmarks_vertexPos.size() << std::endl;
        std::cout << ps_face_landmarks.size() << std::endl;
        std::cout << ps_dlib_landmarks.size() << std::endl;
    #endif

    unsigned int num_ps_landmark = 0;
    num_ps_landmark = ps_dlib_landmarks_vertexPos.size();
    std::cout << num_ps_landmark << std::endl;
    // Write bfm_landmark point cloud
    FacePointCloud::writeFacePointCloudPly("../output/landmark_bfm.ply", ps_bfm_landmarks_vertexPos);

    // Write dlib_landmark point cloud
    std::cout << "Write .ply given backprojected dlib landmarks" << std::endl;
    FacePointCloud::writeFacePointCloudPly("../output/landmark_dlib.ply", ps_dlib_landmarks_vertexPos);

    // get landmark image Mat
    cv::Mat ps_landmark_cvMat = landmarks_extractor.get_landmarkImage(ps_dlib_landmarks, false);
    // write landmark image
    cv::imwrite("PS_considered_landmarks.png", ps_landmark_cvMat);
    // Get triangle list by delaunay triangulation
    std::vector<Vector3i> ps_dlib_landmark_triangleIdLists = landmarks_extractor.get_triList_landmarks(ps_landmark_cvMat, ps_dlib_landmarks, false);
    std::vector<Vector3i> ps_bfm_landmark_triangleIdLists = ps_dlib_landmark_triangleIdLists;

    // Create new instance for ProcrustesAligner
    ProcrustesAligner aligner;
    // Get SE(3)
    Matrix4f Procrustes_estimatedPose = aligner.estimatePose(ps_bfm_landmarks_vertexPos, ps_dlib_landmarks_vertexPos);

#ifdef DEBUG
    std::cout << "with Procrustes Estimate Pose" << std::endl;
    std::cout << Procrustes_estimatedPose << std::endl;
#endif

    // Get transformed BFM landmarks given estimatePose
    std::vector<Vector3f> transformed_onlyProcrustes_available_bfm_landmarks_vertexPos;
    std::vector<Vector3f> transformed_ps_bfm_landmarks_vertexPos;
    bfm.apply_SE3_to_BFMLandmarks(Procrustes_estimatedPose, available_bfm_landmarks_vertexPos, transformed_onlyProcrustes_available_bfm_landmarks_vertexPos, false);
    bfm.apply_SE3_to_BFMLandmarks(Procrustes_estimatedPose, ps_bfm_landmarks_vertexPos, transformed_ps_bfm_landmarks_vertexPos, false);

    // update the mean of vertex position in shape and expression in bfm_manager_first
    bfm.updatePose_bfmManager(Procrustes_estimatedPose, SHAPE, EXP, TEX);

    // update the parameter set as well
    //  bfm.update_Pose_ParameterSet(SHAPE, Procrustes_estimatedPose, DEBUG);
    //  bfm.update_Pose_ParameterSet(EXP, Procrustes_estimatedPose, DEBUG);

    // Write BFM landmarks point cloud which is transformed only by Procrustes
    std::cout << "Write .ply given only Procrustes transformed BFM landmarks" << std::endl;
    FacePointCloud::writeFacePointCloudPly("../output/transformed_onlyProcrustes_landmark_bfm.ply", transformed_onlyProcrustes_available_bfm_landmarks_vertexPos);

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

    FacePointCloud FPC_dlib_landmark{ps_dlib_landmarks_vertexPos, ps_dlib_landmark_triangleIdLists};
    FacePointCloud FPC_bfm_landmark{transformed_ps_bfm_landmarks_vertexPos, ps_bfm_landmark_triangleIdLists};

// check if the points are registered as members of the instance
#ifdef DEBUG
    std::cout << "FPC_dlib_landmark" << std::endl;
    FPC_dlib_landmark.print_points_and_normals();
    std::cout << "FPC_bfm_landmark" << std::endl;
    FPC_bfm_landmark.print_points_and_normals();
#endif

    // Create FacePointCloud instance for Dense ICP between Depthmap
    unsigned int downsampleFactor = 1;
    float maxDistance = 50.f; // 5cm = 50mm
    float minDepth = 500.f;   // 50cm = 500mm
    float maxDepth = 1000.f;  // 100cm = 1000,mm
    FacePointCloud FPC_depthMap{mat_rgb, depth_map, depth_intrinsics, depth_extrinsics, depth_width, depth_height, downsampleFactor, maxDistance, maxDepth, minDepth};

    /*
     * MatrixNormalMap(u, v) (u in horizontal, v in vertical)
     * Each entry has respective Vector3f representing normal vector
     * If we do not have valid normal vector ==> NormalMap(u, v) == Vector3f(MINF, MINF, MINF);
     */
    MatrixNormalMap normalmap = FPC_depthMap.getNormalMap();

#ifdef DEBUG
    for (unsigned int v = 0; v < normalmap.rows(); v++)
    {
        for (unsigned int u = 0; u < normalmap.cols(); u++)
        {
            std::cout << "RGBD_NormalMap(" << u << "," << v << ")"
                      << ": " << normalmap(u, v).transpose() << std::endl;
        }
    }
#endif

    /* TODO write point cloud of the depth map
     *  Above depth map is pixel coordinate x,y (0, 256), and depth
     *  Need to compute backprojection of each point as we do for detected landmark
     */
    FPC_depthMap.writeDepthMapPly("../output/depth_map.ply", 1.0f);
// check if the points are registered as members of the instance
#ifdef DEBUG
    std::cout << "FPC_depthMap" << std::endl;
    FPC_depthMap.print_points_and_normals();
#endif

    /*TODO: ICP for Pose estimation
     * Point to point and point to plane
     */
    // Setup the optimizer.
    FPC_CeresICPOptimizer *landmark_optimizer = nullptr;

    // ceres optimizer
    landmark_optimizer = new FPC_CeresICPOptimizer();

    landmark_optimizer->setMatchingMaxDistance(1e3);

    if (USE_POINT_TO_PLANE)
    {
        landmark_optimizer->usePointToPlaneConstraints(true);
        landmark_optimizer->setNbOfIterations(10); // 10
    }
    else
    {
        landmark_optimizer->usePointToPlaneConstraints(false);
        landmark_optimizer->setNbOfIterations(10);
    }

    // Estimate Pose by the ceres optimizer
    LOG(INFO)<<"Start pose estimation";
    Matrix4f ICP_estimatedPose = landmark_optimizer->estimatePose(FPC_dlib_landmark, FPC_bfm_landmark);
    LOG(INFO)<<"Finish pose estimation";

// check the estimated pose.
#ifdef DEBUG
    std::cout << "Estimate Pose" << std::endl;
    std::cout << ICP_estimatedPose << std::endl;
#endif

    // Get transformed BFM landmarks given estimatePose
    std::vector<Vector3f> transformed_ProcrustesAndICP_bfm_landmarks_vertexPos;
    bfm.apply_SE3_to_BFMLandmarks(ICP_estimatedPose, transformed_onlyProcrustes_available_bfm_landmarks_vertexPos, transformed_ProcrustesAndICP_bfm_landmarks_vertexPos, false);

    // Write BFM landmarks point cloud which is transformed only by Procrustes
    std::cout << "Write .ply given Procrustes/ICP transformed BFM landmarks" << std::endl;
    FacePointCloud::writeFacePointCloudPly("../output/transformed_ProcrustesAndICP_bfm_landmarks.ply", transformed_ProcrustesAndICP_bfm_landmarks_vertexPos);

    delete landmark_optimizer;

#endif

    // update the mean of vertex position in shape and expression in bfm_manager_first
    bfm.updatePose_bfmManager(ICP_estimatedPose, SHAPE, EXP, TEX);

    // update the parameter set as well
    //  bfm.update_Pose_ParameterSet(SHAPE, ICP_estimatedPose, DEBUG);
    //  bfm.update_Pose_ParameterSet(EXP, ICP_estimatedPose, DEBUG);
    bfm.setLandmarks();
    std::vector<Vector3f> post_poseEstimate_bfm_landmarks_vertexPos = bfm.getLandmarkPos();

    /* TODO: Parameter Estimation
     *   Energy term
     *       E(p) = E_dense(P) + E_sparse(P) + E_reg(P)
     *
     *       <Dense term>
     *       E_dense(p) = E_geom(P) + E_color(P)
     *
     *       E_geom(p) = E_point(P) + E_plane(P)
     *           //E_point;
     *                 Source (BFM face mesh)
     *                   - Vertex position
     *                       - [Parameter_set] SHAPE and EXPRESSION (to compute current blendshape)
     *                       - [std::vector<Vector3f>] Current_blendshape_vertex_position (to store current blendshape face mesh vertex position)
     *                 Target (RGBD)
     *                   - Depth Map
     *                       - [MatrixXi] RGBD image
     *                   - Depth camera intrinsic (for back-projection)
     *                       - [Matrix3f] depth_intrinsic
     */
    // list of the index which are corresponding to landmarks
        // map_dlib_bfmLandmarks
    // get the vertex positions which are corresponding to the landmarks

    // 1: get resulting vertex position and color of BFM mesh after pose estimation
    std::vector<Eigen::Vector3f> post_PoseEstimate_BFM_vertex_pos;
    std::vector<Eigen::Vector3f> post_PoseEstimate_BFM_vertex_rgb;
    std::vector<Vector3i> post_PoseEstimate_BFM_triangle_list;

    _withExpression = true;

    std::tie(post_PoseEstimate_BFM_vertex_pos, post_PoseEstimate_BFM_vertex_rgb, post_PoseEstimate_BFM_triangle_list) = bfm.writeAveBFMmesh("../output/pose_poseEstimation_bfmAveFace.ply", _withExpression);

    // target source landmarks mesh
    FacePointCloud targetLandmarksMesh{ps_dlib_landmarks_vertexPos, ps_dlib_landmark_triangleIdLists};

    // set target mesh (we had already set, just look it up)
    FacePointCloud targetMesh = FPC_depthMap;

#if DEBUG
    // for(unsigned int i = 0; i < postPE_bfm_landmarks_points.size(); i++){
    //     std::cout<<"landmark pos: "<<postPE_bfm_landmarks_points[i].transpose()<<std::endl;
    //     std::cout<<"landmark normal: "<<postPE_bfm_landmarks_normals[i].transpose()<<std::endl;
    // }
#endif

    // optimize for shape and texture
    ICPOptimizer *optimizer = nullptr;
    optimizer = new CeresICPOptimizer(BRUTEFORCE_MATCH_GEO);

    optimizer->setMatchingMaxDistance_sparse(1e3);
    optimizer->setMatchingMaxDistance_dense(1e2);
    optimizer->usePointToPlaneConstraints(false);
    optimizer->setNbOfIterations(10);

    std::vector<double> _initial_coefs_shape(global_num_shape_pcs);
    std::vector<double> _initial_coefs_tex(global_num_tex_pcs);
    std::vector<double> _initial_coefs_exp(global_num_exp_pcs);

    std::fill(_initial_coefs_shape.begin(), _initial_coefs_shape.end(), 0.0);
    std::fill(_initial_coefs_tex.begin(), _initial_coefs_tex.end(), 0.0);
    std::fill(_initial_coefs_exp.begin(), _initial_coefs_exp.end(), 0.0);

    std::vector<double> estimated_coefs_shape(global_num_shape_pcs);
    std::vector<double> estimated_coefs_tex(global_num_tex_pcs);
    std::vector<double> estimated_coefs_exp(global_num_exp_pcs);
    LOG(INFO) <<"Start shape and texture estimation";
    std::tie(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp) = optimizer->estimateParams_shape_tex(targetLandmarksMesh, targetMesh, SHAPE, TEX, EXP, _initial_coefs_shape, _initial_coefs_tex, _initial_coefs_exp, bfm, bfm_landmarkIndex_list, averageBFM_triangle_list);
    LOG(INFO) <<"Finish shape and texture estimation";

    delete optimizer;

    // optimize for exp and texture
    ICPOptimizer *optimizer_2 = nullptr;
    optimizer_2 = new CeresICPOptimizer(BRUTEFORCE_MATCH_GEO);

    optimizer_2->setMatchingMaxDistance_sparse(1e1);
    optimizer_2->setMatchingMaxDistance_dense(1e1);
    optimizer_2->usePointToPlaneConstraints(false);
    optimizer_2->setNbOfIterations(10);
    LOG(INFO) <<"Start expression and texture estimation";
    std::tie(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp) = optimizer_2->estimateParams_exp_tex(targetLandmarksMesh, targetMesh, SHAPE, TEX, EXP, estimated_coefs_shape, estimated_coefs_tex, _initial_coefs_exp, bfm, bfm_landmarkIndex_list, averageBFM_triangle_list);
    LOG(INFO) <<"Finish expression and texture estimation";
    delete optimizer_2;

    // optimization for color with 
    ICPOptimizer *optimizer_forColor = nullptr;

    optimizer_forColor = new CeresICPOptimizer(BRUTEFORCE_MATCH_TEX);

    optimizer_forColor->setMatchingMaxDistance_color(1e1);
    optimizer_forColor->usePointToPlaneConstraints(false);
    optimizer_forColor->setNbOfIterations(5);

    std::vector<double> final_coefs_shape(global_num_shape_pcs);
    std::vector<double> final_coefs_tex(global_num_tex_pcs);
    std::vector<double> final_coefs_exp(global_num_exp_pcs);
    std::tie(final_coefs_shape, final_coefs_tex, final_coefs_exp) = optimizer_forColor->estimateParams_colors(targetMesh, SHAPE, TEX, EXP, estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp, bfm, averageBFM_triangle_list);

    delete optimizer_forColor;

#ifdef DEBUG
    std::cout << left_line << "BFM Parameters and Components" << right_line << std::endl;
    // Printing all components is too expensive to print out, hence comment out now.
    std::cout << "Number of vertices: " << SHAPE.num_vertices << std::endl;

    std::cout << left_line << SHAPE.name << right_line << std::endl;
    std::cout << SHAPE.name << " MEAN, vector shape =" << SHAPE.mean.rows() << std::endl;
    // std::cout<<SHAPE.mean<<std::endl;
    std::cout << SHAPE.name << " VARIANCE, vector shape =" << SHAPE.variance.rows() << std::endl;
    // std::cout<<SHAPE.variance<<std::endl;
    std::cout << SHAPE.name << " Principal Components, matrix shape = [" << SHAPE.pc.rows() << ", " << SHAPE.pc.cols() << "]" << std::endl;
    std::cout << SHAPE.name << " Principal Components" << std::endl;

    // std::cout<<SHAPE.pc<<std::endl;
    // std::cout<<left_line<<"Shape parameter"<<right_line<<std::endl;

    std::cout << SHAPE.name << " parameter, shape = " << SHAPE.parameters.size() << std::endl;
    // for(auto & params : SHAPE.parameters){
    //     std::cout<<params<<", ";
    // }
    // std::cout<<std::endl;

    std::cout << left_line << TEX.name << right_line << std::endl;
    std::cout << TEX.name << " MEAN, vector shape =" << TEX.mean.rows() << std::endl;
    // std::cout<<TEX.mean<<std::endl;
    std::cout << TEX.name << " VARIANCE, vector shape =" << TEX.variance.rows() << std::endl;
    // std::cout<<TEX.variance<<std::endl;
    std::cout << TEX.name << " Principal Components, matrix shape = [" << TEX.pc.rows() << ", " << TEX.pc.cols() << "]" << std::endl;
    std::cout << TEX.name << " Principal Components" << std::endl;

    // PC matrix is too expensive to print out, hence comment out now.
    // std::cout<<SHAPE.pc<<std::endl;
    // std::cout<<left_line<<"Shape parameter"<<right_line<<std::endl;

    std::cout << TEX.name << " parameter, shape = " << TEX.parameters.size() << std::endl;
    // for(auto & params : TEX.parameters){
    //     std::cout<<params<<", ";
    // }
    // std::cout<<std::endl;

    std::cout << left_line << EXP.name << right_line << std::endl;
    std::cout << EXP.name << " MEAN, vector shape =" << EXP.mean.rows() << std::endl;
    // std::cout<<EXP.mean<<std::endl;
    std::cout << EXP.name << " VARIANCE, vector shape =" << EXP.variance.rows() << std::endl;
    // std::cout<<EXP.variance<<std::endl;
    std::cout << EXP.name << " Principal Components, matrix shape = [" << EXP.pc.rows() << ", " << EXP.pc.cols() << "]" << std::endl;
    std::cout << EXP.name << " Principal Components" << std::endl;

    // std::cout<<SHAPE.pc<<std::endl;
    // std::cout<<left_line<<"Shape parameter"<<right_line<<std::endl;

    std::cout << EXP.name << " parameter, shape = " << EXP.parameters.size() << std::endl;

    // for(auto & params : EXP.parameters){
    //     std::cout<<params<<", ";
    // }
    // std::cout<<std::endl;
    std::cout << left_line << right_line << std::endl;
#endif

    google::ShutdownGoogleLogging();

    return 0;
}
