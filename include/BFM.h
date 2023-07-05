#pragma once
#include "Eigen.h"
#include "bfm_manager.h"
#include "SimpleMesh.h"
#include "SimpleMesh_bfm.h"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <array>
#include <memory>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <tuple>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

const std::string LOG_PATH = R"(./log)";

struct Landmarks{
    int id;
    //id in the dlib assignment
    int dlib_id;
    //vertex id in the BFM template mesh
    int bfm_id;
    //vertex position
    Vertex v;
};

struct Parameter_set{
    std::string name;
    Eigen::VectorXd mean;
    Eigen::VectorXd variance;
    Eigen::MatrixXd pc;
    unsigned int num_vertices;
    std::vector<double> parameters;

};

class BFM{
public:

	BFM() = default;

    ~BFM(){
    	google::ShutdownGoogleLogging();
    }
    
    bool init(int argc, char *argv[], double dFx = 525.0, double dFy = 525.0, double dCx = 319.5f-192.0, double dCy = 239.5f - 74) {
        // logging
        google::InitGoogleLogging(argv[0]); 
        FLAGS_logtostderr = false;
        if(fs::exists(LOG_PATH)) 
            fs::remove_all(LOG_PATH);
        fs::create_directory(LOG_PATH);
        FLAGS_alsologtostderr = true;
        FLAGS_log_dir = LOG_PATH;
        FLAGS_log_prefix = true; 
        FLAGS_colorlogtostderr =true;

        // command options
        po::options_description opts("Options");
        po::variables_map vm;

        std::string sBfmH5Path, sLandmarkIdxPath;
        sBfmH5Path = "../data/model2017-1_bfm_nomouth.h5";
        sLandmarkIdxPath = "../data/map_dlib-bfm_rk.anl";

        try
        {
            po::store(po::parse_command_line(argc, argv, opts), vm);
        }
        catch(...)
        {
            LOG(ERROR) << "These exists undefined command options.";
            return -1;
        }

        po::notify(vm);
        if(vm.count("help"))
        {
            LOG(INFO) << opts;
            return 0;
        }

        std::array<double, N_INT_PARAMS> aIntParams = { dFx, dFy, dCx, dCy };
        std::unique_ptr<BfmManager> pBfmManager(new BfmManager(sBfmH5Path, aIntParams, sLandmarkIdxPath));
        BFM_Manager = *pBfmManager;
        num_vertices = BFM_Manager.getNVertices();
        num_triangles = BFM_Manager.getNFaces();
        //SimpleMesh averageFace is created here
        //Developed wrapper by using the function in the BfmManager

        //Set name
        SHAPE.name = "Shape";
        TEX.name = "Texture";
        EXP.name = "Expression";

        //Get all BFM components
        BFM_Manager.GetShapeComponents(SHAPE.mean, SHAPE.variance, SHAPE.pc);
        BFM_Manager.GetTexComponents(TEX.mean, TEX.variance, TEX.pc);
        BFM_Manager.GetExpComponents(EXP.mean, EXP.variance, EXP.pc);

        SHAPE.num_vertices = num_vertices;
        TEX.num_vertices = num_vertices;
        EXP.num_vertices = num_vertices;

        //get triagle lists
        getTriangleList();
        
        //get Average shape, texture, expression, shape  with expression (Blenshape)
        getAveFaceComponents();

        //set the landmarks
        setLandmarks();
        
        return true;
    }

    void write_from_PointCloud_TriangleList(std::string fn, std::vector<Vector3f> & point_cloud, std::vector<Vector3i> & triangle_list){
        std::ofstream out;
        /* Note: In Linux Cpp, we should use std::ios::out as flag, which is not necessary in Windows */
        out.open(fn, std::ios::out | std::ios::binary);
        if(!out.is_open())
        {
            LOG(ERROR) << "Creation of " << fn << " failed.";
            return;
        }

        std::cout<<"Writing face mesh ply ("<<fn<<")"<<"....";

        out << "ply\n";
        out << "format ascii 1.0\n";
        out << "comment Made from the 3D Morphable Face Model of the Univeristy of Basel, Switzerland.\n";
        out << "element vertex " << point_cloud.size() << "\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        out << "element face " << triangle_list.size() << "\n";
        out << "property list uchar int vertex_indices\n";
        out << "end_header\n";

        // unsigned long int cnt = 0;
        for (Vector3f point : point_cloud)
        {
            out<<point.x()<<" "<<point.y()<<" "<<point.z()<<"\n";
        }

        unsigned char N_VER_PER_FACE = 3;
        for (auto & t : triangle_list) 
        {
            out<<(int)N_VER_PER_FACE<<" "<<t.x()<<" "<<t.y()<<" "<<t.z()<<"\n";
        }

        out.close();
        
        std::cout<<"Finish face mesh ply ("<<fn<<")"<<std::endl;
    }

    void writeLandmarkPly(std::string fn, std::vector<Vector3f> & landmarks){
        std::ofstream out;
        /* Note: In Linux Cpp, we should use std::ios::out as flag, which is not necessary in Windows */
        out.open(fn, std::ios::out | std::ios::binary);
        if(!out.is_open())
        {
            LOG(ERROR) << "Creation of " << fn << " failed.";
            return;
        }

        std::cout<<"Writing face mesh ply ("<<fn<<")"<<"....";

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
        
        std::cout<<"Finish face mesh ply ("<<fn<<")"<<std::endl;
    }

    inline unsigned int getNumLandmarks(){
        return num_landmarks;
    }

    inline std::vector<Landmarks> getLandmarks(){
        return m_landmarks;
    }

    inline std::vector<std::pair<int, int>> getMapDlib2BFM_landmark(){
        return m_mapLandmark_ids;
    }

    inline void getTriangleList(){
        BFM_TriangleList = BFM_Manager.getTriangleList();
    }

    std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Vector3i>> writeBFMmesh(std::string f_name_ply, std::vector<double> & _coef_shape, std::vector<double> & _coef_tex, std::vector<double> & _coef_exp, bool _withExp){
        //write mesh using pBfManager->GenBFM()
        setCoefs(_coef_shape, _coef_tex, _coef_exp);
        std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Vector3i>> _result_mesh_components;
        _result_mesh_components = BFM_Manager.GetBFM(f_name_ply, Coefs_shape, Coefs_tex, Coefs_exp, false, _withExp);
        return _result_mesh_components;
    }

    std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Vector3i>> writeAveBFMmesh(std::string f_name_ply, bool _withExp){
        //write mesh using pBfManager->GenBFM()
        ResetCoefs();
        std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Vector3i>> _result_mesh_components;
        _result_mesh_components = BFM_Manager.GetBFM(f_name_ply, Coefs_shape, Coefs_tex, Coefs_exp, true, _withExp);
        return _result_mesh_components;
    }

    inline void setCoefs(std::vector<double> & _coef_shape, std::vector<double> & _coef_tex, std::vector<double> & _coef_exp){
        SHAPE.parameters.insert(SHAPE.parameters.begin(), _coef_shape.begin(), _coef_shape.end());
        TEX.parameters.insert(TEX.parameters.begin(), _coef_tex.begin(), _coef_tex.end());
        EXP.parameters.insert(EXP.parameters.begin(), _coef_exp.begin(), _coef_exp.end());
        Coefs_shape.insert(Coefs_shape.begin(), _coef_shape.begin(), _coef_shape.end());
        Coefs_tex.insert(Coefs_tex.begin(), _coef_tex.begin(), _coef_tex.end());
        Coefs_exp.insert(Coefs_exp.begin(), _coef_exp.begin(), _coef_exp.end());
    }

    inline void ResetCoefs(){
        std::fill(Coefs_shape.begin(), Coefs_shape.end(), 0.0);
        std::fill(Coefs_tex.begin(), Coefs_tex.end(), 0.0);
        std::fill(Coefs_exp.begin(), Coefs_exp.end(), 0.0);
    }

    inline void getAveFaceComponents(){
        m_AveShape = BFM_Manager.getCurrentShape();
        m_AveTex = BFM_Manager.getCurrentTex();
        m_AveExpr = BFM_Manager.getCurrentExpr();
        m_AveBlendshape = BFM_Manager.getCurrentBlendshape();
    }

    inline int float2color(const float & _f_color){
        return ((int)(_f_color*255));
    }


    void setLandmarks(){
        float x, y, z;
        float r, g, b;
        int dlibIdx, bfmIdx;
        landmarkPoints.reserve(num_landmarks);
        m_mapLandmark_ids = BFM_Manager.getMapLandmarkIndices();
        unsigned int counter = 0;
        for(const auto& [dlibIdx, bfmIdx] : m_mapLandmark_ids){
            //print
        	std::cout<<dlibIdx<<" "<<bfmIdx<<std::endl;
            Landmarks l;
            l.id = counter;
            l.dlib_id = dlibIdx;
            l.bfm_id = bfmIdx;

            x = float(m_AveBlendshape(bfmIdx*3));
            y = float(m_AveBlendshape(bfmIdx*3+1));
            z = float(m_AveBlendshape(bfmIdx*3+2));

            Vector3f _vertex_pos = {x, y, z};
            landmarkPoints.push_back(_vertex_pos);
           
            Vertex v;
            v.position.x() = x;
            v.position.y() = y;
            v.position.z() = z;
            v.position.w() = 1.f;
            l.v = v;
            
            Vector4i colorInt;
            r = float(m_AveTex(bfmIdx*3));
            g = float(m_AveTex(bfmIdx*3+1));
            b = float(m_AveTex(bfmIdx*3+2));

            colorInt.x() = float2color(r);
            colorInt.y() = float2color(g);
            colorInt.z() = float2color(b);
            colorInt.w() = 255;

			v.color = Vector4uc((unsigned char)colorInt.x(), (unsigned char)colorInt.y(), (unsigned char)colorInt.z(), (unsigned char)colorInt.w());

            l.v = v;
            m_landmarks.push_back(l);
            counter++;
        }
    }

    void setSimpleMesh(SimpleMesh_bfm averageFace){
        averageFace.setNumVertices(num_vertices);
        averageFace.setNumVertices(num_triangles);

        averageFace_mVertices.reserve(num_vertices);
        averageFace_mTriangles.reserve(num_triangles);

        //set vertices position
        for(unsigned int i = 0; i<num_vertices; i++){
            float x, y, z;
            float r, g, b;
            Vertex _v;
            x = float(m_AveBlendshape(i*3));
            y = float(m_AveBlendshape(i*3+1));
            z = float(m_AveBlendshape(i*3+2));

            _v.position.x() = x;
            _v.position.y() = y;
            _v.position.z() = z;
            _v.position.w() = 1.f;

            Vector4i colorInt;
            r = float(m_AveTex(i*3));
            g = float(m_AveTex(i*3+1));
            b = float(m_AveTex(i*3+2));

            colorInt.x() = float2color(r);
            colorInt.y() = float2color(g);
            colorInt.z() = float2color(b);
            colorInt.w() = 255;

			_v.color = Vector4uc((unsigned char)colorInt.x(), (unsigned char)colorInt.y(), (unsigned char)colorInt.z(), (unsigned char)colorInt.w());

            averageFace_mVertices.push_back(_v);
        }

        //set triangles lists
        for(unsigned int id_tri = 0; id_tri < num_triangles; id_tri++){
            unsigned int x, y, z;
            x = (unsigned int)(BFM_TriangleList(id_tri * 3));
            y = (unsigned int)(BFM_TriangleList(id_tri * 3+1));
            z = (unsigned int)(BFM_TriangleList(id_tri * 3+2));
            Triangle _tri(x, y, z);

            averageFace_mTriangles.push_back(_tri);
        }

        averageFace.setVertices(averageFace_mVertices);
        averageFace.setTriangles(averageFace_mTriangles);
    }

    inline std::vector<Vector3f>& getLandmarkPos(){
        return landmarkPoints;
    }

    inline Parameter_set getParameter_set_SHAPE(){
        return SHAPE;
    }

    inline Parameter_set getParameter_set_TEX(){
        return TEX;
    }

    inline Parameter_set getParameter_set_EXP(){
        return EXP;
    }

    void update_Pose_ParameterSet(Parameter_set & ps, Eigen::Matrix4f SE3, bool _debug){
        if(ps.name == "Texture"){
            std::cout<<"ERROR: Parameter set should be SHAPE or EXPRESSION"<<std::endl;
        }else{
            std::cout<<"Update parameter "<<ps.name<<"....";
        }
        Eigen::VectorXd means = ps.mean;
        unsigned int num_vertices = ps.num_vertices;

        for(unsigned int i = 0; i<num_vertices; i++){
            Eigen::Vector4f _point = {(float)means(3*i), (float)means(3*i+1), (float)means(3*i+2), 1.0};
            if(_debug){
                std::cout<<"Before: "<<_point.transpose()<<std::endl;
            }
            Eigen::Vector4f _transformed_point = SE3 *_point;
            means(3*i) = _transformed_point.x(); 
            means(3*i + 1) = _transformed_point.y(); 
            means(3*i + 2) = _transformed_point.z(); 
            if(_debug){
                std::cout<<"After: "<<_transformed_point.transpose()<<std::endl;
            }
        }

        std::cout<<"done"<<std::endl;
    }

    void updatePose_bfmManager(Eigen::Matrix4f & SE3){
        BFM_Manager.update_ShapeMu(SE3);
        BFM_Manager.update_ExprMu(SE3);
    }

    void apply_SE3_to_BFMLandmarks(Eigen::Matrix4f & estimate_pose, std::vector<Vector3f> & _bfm_landmarks_PosList, std::vector<Vector3f> & transformed_bfm_landmarks, bool _debug){
        Eigen::MatrixXf rotation = estimate_pose.block(0, 0, 3, 3);
        Eigen::VectorXf translation = estimate_pose.block(0, 3, 3, 1);
        unsigned int num_landmarks = _bfm_landmarks_PosList.size();

        if(_debug){
            std::cout<<"Rotation"<<std::endl;
            std::cout<<rotation<<std::endl;

            std::cout<<"translation"<<std::endl;
            std::cout<<translation<<std::endl;
        }

        for(auto & vertex : _bfm_landmarks_PosList){
            Eigen::VectorXf _transformed_point = rotation * vertex + translation;
            if(_debug){
                std::cout<<"Transformed point"<<std::endl;
                std::cout<<_transformed_point<<std::endl;
            }
            Vector3f _tp = {_transformed_point.x(), _transformed_point.y(), _transformed_point.z()};
            transformed_bfm_landmarks.push_back(_transformed_point);
        }

    }

     BfmManager * getBfmmanager(){
        return &BFM_Manager;
    }

private:
	Eigen::VectorXd m_AveShape;
	Eigen::VectorXd m_AveTex;
	Eigen::VectorXd m_AveExpr;
	Eigen::VectorXd m_AveBlendshape;
    std::vector<Vertex> averageFace_mVertices;
    std::vector<Triangle> averageFace_mTriangles;
	Eigen::Matrix<int, Eigen::Dynamic, 1> BFM_TriangleList;

    BfmManager BFM_Manager;

    //number of vertex
    unsigned int num_vertices;
    unsigned int num_triangles;

    unsigned int num_coefs_shape = 199;
    unsigned int num_coefs_tex = 199;
    unsigned int num_coefs_exp = 100;
    unsigned int num_landmarks = 68;

    Parameter_set SHAPE;
    Parameter_set TEX;
    Parameter_set EXP;

    //Shape mean, variacne, and Pc(Principal components)
    Eigen::VectorXd MEAN_shape;
    Eigen::VectorXd VAR_shape;
    Eigen::MatrixXd PC_shape;

    //Texture mean, variance, and Pc(Principal compoents)
    Eigen::VectorXd MEAN_tex;
    Eigen::VectorXd VAR_tex;
    Eigen::MatrixXd PC_tex;

    //Expression mean, variance, and Pc(Principal compoents)
    Eigen::VectorXd MEAN_exp;
    Eigen::VectorXd VAR_exp;
    Eigen::MatrixXd PC_exp;

    //coefficients
    std::vector<double> Coefs_shape;
    std::vector<double> Coefs_tex;
    std::vector<double> Coefs_exp;

    //landmark list
    std::vector<Landmarks> m_landmarks;
    std::vector<std::pair<int, int>> m_mapLandmark_ids;
    std::vector<Vector3f> landmarkPoints;

    //vertex list for neutral face (without expression)
    std::vector<Vertex> neutral_vertices;
    std::vector<Triangle> neutral_triangles;

    //vertex list for face mesh with facial expression
    std::vector<Vertex> blendshape_vertices;
    std::vector<Triangle> blendshape_triangles;
};