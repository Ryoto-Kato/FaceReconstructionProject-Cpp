#include <ceres/ceres.h>

#include "../include/Eigen.h"
#include "../include/RGBImage.h"
#include "../include/SimpleMesh.h"
#include "BFM.h"

constexpr int NUMBER_OF_SHAPE_PARAMS = 20;
constexpr int NUMBER_OF_EXPRESSION_PARAMS = 20;
constexpr int NUMBER_OF_RESIDUALS = 53490*4;

template <typename T>
static inline void fillVector(const T* input, std::vector<double>& output) {
    for(double& element : output)  {
        element = static_cast<double>(*input);
        ++input;
    }
}

class GeometricConstraint {
public:
    // use RGBDImage class to save RGBD input?
    GeometricConstraint(BFM* bfm_, const Matrix3f& intrinsic_, const Matrix4f& extrinsic_, const MatrixXf depth_map_, const float weightPoint_, const float weightPlane_) :
        bfm{ bfm_},
        intrinsic{ intrinsic_ },
        extrinsic{ extrinsic_ },
        depth_map{ depth_map_ },
        weightPoint{ weightPoint_ },
        weightPlane{ weightPlane_ }
    { }

    template <typename T>
    bool operator()(const T* shape_parameters, const T* expression_parameters, T* residuals) const {
        // generateMesh(parameters) function to generate mesh from given paramters
        //SimpleMesh templateMesh = generateMesh(parameters);
        // bfm->getMutableShapeCoef() = shape_parameters;
        // bfm->getMutableExprCoef() = expression_parameters;
        // bfm->genFace();
        // VectorXd templateVertices = bfm->getCurrentBlendshapeTransformed();

        std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Vector3i>> shape;

        // std::vector<double> coef_shape(shape_parameters, shape_parameters + NUMBER_OF_SHAPE_PARAMS);
        // std::vector<double> coef_tex = bfm->getParameter_set_TEX().parameters;
        // std::vector<double> coef_exp(expression_parameters, expression_parameters + NUMBER_OF_EXPRESSION_PARAMS);

        // float* shape_coef = reinterpret_cast<float*>(shape_parameters);
        // std::vector<double> coef_shape(shape_coef, shape_coef + NUMBER_OF_SHAPE_PARAMS);
        std::vector<double> coef_shape(199, 0.0); // = bfm->getParameter_set_SHAPE().parameters;
        std::vector<double> coef_tex   = bfm->getParameter_set_TEX().parameters;
        std::vector<double> coef_exp   = bfm->getParameter_set_EXP().parameters;

        fillVector(shape_parameters, coef_shape);

        auto [vertexPos, vertexRGB, triangleList] = bfm->writeBFMmesh("temp.ply", coef_shape, coef_tex, coef_exp, true);

        // extract vertices from mesh
        // std::vector<Vertex> templateVertices = templateMesh.GetVertices();

        // compute normals of mesh
        // std::vector<Vector3f> templateNormals = templateMesh.getNormals();

        // TODO: we need normals form the depth map

        // implementation "reprojection" error
        int vertexIndex = 0;
        for(const auto& vertexPosWorld : vertexPos) {
            Vector4f vertexPosWorldHom(4);
            vertexPosWorldHom << vertexPosWorld, 1.0f;

            // transform vertex coordinates from world to camera frame
            Vector4f vertexPosCameraHom = extrinsic * vertexPosWorldHom;
            Vector3f vertexPosCamera = vertexPosCameraHom.head(3);

            // project vertex into image
            Vector3f vertexPosImage = intrinsic * vertexPosCamera;
            float u = vertexPosImage[0] / vertexPosImage[2];
            float v = vertexPosImage[1] / vertexPosImage[2];

            // round computed pixel coordinate to integer and extract depth of pixel
            float depth = depth_map(round(u), round(v));
            // Reverse rounding or not?
            Vector3f vertexPosPixel(u, v, 1.0f);
            // backproject pixel back to world coordinates
            vertexPosPixel *= depth;
            Vector3f pixelPosCamera = intrinsic.inverse() * vertexPosPixel;
            Vector4f pixelPosCameraHom(4);
            pixelPosCameraHom << pixelPosCamera, 1.0f;
            Vector4f pixelPosWorldHom = extrinsic.inverse() * pixelPosCameraHom;
            Vector3f pixelPosWorld = pixelPosWorldHom.head(3);

            Vector3f differences = vertexPosWorld - pixelPosWorld;
            // add point to point distances between template vertex and backprojected pixel to residuals
            residuals[4*vertexIndex] = T(std::sqrt(weightPoint)) * T(differences[0]);
            residuals[4*vertexIndex + 1] = T(std::sqrt(weightPoint)) * T(differences[1]);
            residuals[4*vertexIndex + 2] = T(std::sqrt(weightPoint)) * T(differences[2]);

            // add point to plane distances between template vertex and backprojected pixel to residuals
            Vector3f normal = templateNormals[vertexIndex];
            residuals[4*vertexIndex + 3] = T(std::sqrt(weightPlane)) * T(differences.dot(normal));

            vertexIndex++;
            return true;
        }
    }

    // TODO: update number of residuals when adding back point ot plane
    static ceres::CostFunction* create(BFM* bfm, const Matrix3f& intrinsic, const Matrix4f& extrinsic, const MatrixXf depth_map, const float weightPoint, const float weightPlane) {
        assert(NUMBER_OF_RESIDUALS == bfm->getNVertices()*4);
        return new ceres::AutoDiffCostFunction<GeometricConstraint, NUMBER_OF_RESIDUALS, NUMBER_OF_SHAPE_PARAMS, NUMBER_OF_EXPRESSION_PARAMS>(
            new GeometricConstraint(bfm, intrinsic, extrinsic, depth_map, weightPoint, weightPlane));
    }

protected:
    BFM* bfm;
    const Matrix3f intrinsic;
    const Matrix4f extrinsic;
    const MatrixXf depth_map;
    const float weightPoint;
    const float weightPlane;
}; //GeometricConstraint

class BfmOptimizer {
public:
    BfmOptimizer() :
        m_nIterations{ 10 }
    {}

    void setNbOfIterations(unsigned nIterations) {
        m_nIterations = nIterations;
    }

    void estimateParameters(BFM* bfm, MatrixXf& depth_map, Matrix3f& intrinsic, Matrix4f& extrinsic) {

        //Params
        double weightPoint = 1.0;
        double weightPlane = 1.0;

        double shape_coef_increment [199];
        double expr_coef_increment [100];

        for (int i = 0; i < m_nIterations; ++i) {

            ceres::Problem problem;
            problem.AddResidualBlock(
                        GeometricConstraint::create(bfm, intrinsic, extrinsic, depth_map, weightPoint, weightPlane),
                        nullptr, shape_coef_increment, expr_coef_increment);

            // Configure options for the solver.
            ceres::Solver::Options options;
            configureSolver(options);

            // Run the solver (for one iteration).
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << std::endl;
            //std::cout << summary.FullReport() << std::endl;

            // Update parameter estimate (TODO: We might need to loop over the arrays here)
            // bfm->getMutableShapeCoef() = shape_coef_increment;
            // bfm->getMutableExprCoef() = expr_coef_increment;

        }
    }
private:
    int m_nIterations;

    void configureSolver(ceres::Solver::Options& options) {
        // Ceres options.
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.use_nonmonotonic_steps = false;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = 1;
        options.max_num_iterations = 1;
        options.num_threads = 8;
    }
}; //BfmOptimizer

// TODO: write function to compute normals for each vertex of template mesh for point to plane cost function
//       (integrate in mesh class)
// TODO: outlier rejection (threshold distances/depth, normals)
// TODO: write generateMesh() function