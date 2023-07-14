#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

#include "../include/Eigen.h"
#include "../include/RGBImage.h"
#include "../include/SimpleMesh.h"
#include "BFM.h"

constexpr int NUMBER_OF_SHAPE_PARAMS = 199;
constexpr int NUMBER_OF_EXPRESSION_PARAMS = 100;
constexpr int NUMBER_OF_VERTICES = 53149;
constexpr int NUMBER_OF_RESIDUALS = NUMBER_OF_VERTICES*3;

template <typename T>
T compute_dot_product(const Eigen::VectorXd & pc_row, const T *params, const int& dim) {
    T result{0};
    for(size_t i = 0; i < dim; i++) {
        result += T(pc_row(i)) * params[i];
    }
    return result;
}

class LeanBFM {
public:
    LeanBFM(BFM* bfm_):
    SHAPE{ bfm_->getParameter_set_SHAPE()},
    EXP{ bfm_->getParameter_set_EXP()},
    TEX{ bfm_->getParameter_set_TEX()}
    {}

    template <typename T>
    void get_vertex(const T* shape_parameters, const T* expression_parameters, T* vertex_pos) const
    {
        // std::cout << "in get_vertex()" << "\n";
        // std::cout << "SHAPE.mean.size():" << SHAPE.mean.size() << "\n";
        // std::cout << "SHAPE.pc.rows():" << SHAPE.pc.rows() << "\n";
        // std::cout << "SHAPE.pc.cols():" << SHAPE.pc.cols() << "\n";
        // std::cout << "EXP.pc.rows():" << EXP.pc.rows() << "\n";
        // std::cout << "EXP.pc.cols():" << EXP.pc.cols() << "\n";

        // Equation 1 from http://www.graphics.stanford.edu/~niessner/papers/2015/10face/thies2015realtime.pdf
        for(size_t i = 0 ; i < SHAPE.mean.size(); i++)
        {
            vertex_pos[i] = (T)SHAPE.mean(i) + compute_dot_product(SHAPE.pc.row(i), shape_parameters, NUMBER_OF_SHAPE_PARAMS) + compute_dot_product(EXP.pc.row(i), expression_parameters, NUMBER_OF_EXPRESSION_PARAMS);
        }
    }

private:
    Parameter_set SHAPE;
    Parameter_set EXP;
    Parameter_set TEX;
}; //LeanBFM


class GeometricConstraint {
public:
    // use RGBDImage class to save RGBD input?
    GeometricConstraint(BFM* bfm_, const Matrix3f& intrinsic_, const Matrix4f& extrinsic_, const MatrixXf depth_map_, const float weightPoint_, const float weightPlane_) :
        bfm{ bfm_},
        intrinsic{ intrinsic_ },
        extrinsic{ extrinsic_ },
        depth_grid{depth_map_.data(), 0, depth_map_.rows(), 0, depth_map_.cols()},
        depth_interpolator{depth_grid},
        weightPoint{ weightPoint_ },
        weightPlane{ weightPlane_ },
        lean_bfm{bfm_}
    {}

    template <typename T>
    bool operator()(const T* shape_parameters, const T* expression_parameters, T* residuals) const {

        assert(NUMBER_OF_RESIDUALS == bfm->getNVertices()*3);

        T* vertexPos = new T[NUMBER_OF_VERTICES * 3]{};

        lean_bfm.get_vertex<T>(shape_parameters, expression_parameters, vertexPos);

        T current_vertex_pos[3];
        for(size_t vertex_index = 0; vertex_index < NUMBER_OF_VERTICES; vertex_index++) {
            for (int i = 0; i < 3; i++) {
                current_vertex_pos[i] = vertexPos[3*vertex_index + i];
            }

            // std::cout << "operator() iteration: " << vertex_index << "\n";

            T fx = (T)intrinsic(0,0);
            T fy = (T)intrinsic(1,1);
            T cx = (T)intrinsic(0,2);
            T cy = (T)intrinsic(1,2);
            T Z = current_vertex_pos[2];

            T u = fx * current_vertex_pos[0]/Z + cx;
            T v = fy * current_vertex_pos[1]/Z + cy;

            T depth;
            depth_interpolator.Evaluate(u,v, &depth);

            T x_camera = ((u-cx)/fx) * (depth);
            T y_camera = ((v-cy)/fy) * (depth);
            T z_camera = depth;

            // add point to point distances between template vertex and backprojected pixel to residuals
            residuals[3*vertex_index]     = T(std::sqrt(weightPoint)) * (current_vertex_pos[0] - x_camera);
            residuals[3*vertex_index + 1] = T(std::sqrt(weightPoint)) * (current_vertex_pos[1] - y_camera);
            residuals[3*vertex_index + 2] = T(std::sqrt(weightPoint)) * (current_vertex_pos[2] - z_camera);
        }

        return true;
    }

    // TODO: update number of residuals when adding back point ot plane
    static ceres::CostFunction* create(BFM* bfm, const Matrix3f& intrinsic, const Matrix4f& extrinsic, const MatrixXf depth_map, const float weightPoint, const float weightPlane) {
        assert(NUMBER_OF_RESIDUALS == bfm->getNVertices()*3);
        return new ceres::AutoDiffCostFunction<GeometricConstraint, NUMBER_OF_RESIDUALS, NUMBER_OF_SHAPE_PARAMS, NUMBER_OF_EXPRESSION_PARAMS>(
            new GeometricConstraint(bfm, intrinsic, extrinsic, depth_map, weightPoint, weightPlane));
    }

protected:
    BFM* bfm;
    LeanBFM lean_bfm;
    const Matrix3f intrinsic;
    const Matrix4f extrinsic;
    ceres::Grid2D<float>  depth_grid;
    ceres::BiCubicInterpolator<ceres::Grid2D<float>> depth_interpolator;
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

        double shape_coef [NUMBER_OF_SHAPE_PARAMS] = {0.0};
        double tex_coef [NUMBER_OF_SHAPE_PARAMS] = {0.0};
        double expr_coef [NUMBER_OF_EXPRESSION_PARAMS] = {0.0};


        for (int i = 0; i < m_nIterations; ++i) {

            ceres::Problem problem;
            problem.AddResidualBlock(
                        GeometricConstraint::create(bfm, intrinsic, extrinsic, depth_map, weightPoint, weightPlane),
                        nullptr, shape_coef, expr_coef);

            // Configure options for the solver.
            ceres::Solver::Options options;
            configureSolver(options);

            // Run the solver (for one iteration).
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.BriefReport() << std::endl;
            std::cout << summary.FullReport() << std::endl;
        }

        // Update parameter estimate
        std::vector<double> shape_coef_vect(shape_coef, shape_coef + sizeof shape_coef / sizeof shape_coef[0]);
        std::vector<double> tex_coef_vect(tex_coef, tex_coef + sizeof tex_coef/ sizeof tex_coef[0]);
        std::vector<double> expr_coef_vet(expr_coef, expr_coef + sizeof expr_coef / sizeof expr_coef[0]);
        bfm->setCoefs(shape_coef_vect, tex_coef_vect, expr_coef_vet);
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



// TODO: Verify that BiCubicInterpolation of dept returns right values (compare against eigen matrix depth map)
// TODO: filter invalid values from depthmap