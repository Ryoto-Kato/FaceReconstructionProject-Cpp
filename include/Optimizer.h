#pragma once

// The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <flann/flann.hpp>

#include "SimpleMesh.h"
#include "NearestNeighbor.h"
#include "ProcrustesAligner.h"
#include "BFM.h"
#include "FacePointCloud.h"

#include <string>
#include <algorithm>

template <class T>
class CMatrix
{
public:
    int num_rows, num_cols;
	T **data;
	CMatrix( int rows, int cols)
    {
        setRow(rows);
        setCol(cols);

		num_rows = rows;
		num_cols = cols;

        data = new T*[rows]; // replaced "int" for "T"

        for (int i = 0; i < row; i++) {
            data[i] = new T [cols]; // replaced "int" for "T"
        }

        for(int i = 0; i < row; i++) {
            for(int j = 0; j < cols; j++) {
                data[i][j] = T(0); // replaced "int" for "T"
            }
        }
    }

    void print();
    void setRow(int r){row = r;}
    void setCol(int c){col = c;}
	int rows(){return row;}
	int cols(){return col;}
    T& operator()(int row, int col);
private:
    int row,col;
};

template <class T>
void CMatrix<T>::print ()
{
    int i,j;

    for (i=0;i < row;i++) // Here you used to have row hard coded to 4
    {
        for(j=0;j < col;j++) // Here you used to have col hard coded to 4
        {
            printf("%.1f    ",(float) data[i][j]);
        }
        printf("\n");
    }
}

// Recently added
template<class T> T& CMatrix<T>::operator()(int row, int col)
{
    return data[row][col];
}

/**
 * Helper methods for writing Ceres cost functions.
 */
template <typename T>
static inline void fillVector(const Vector3d &input, T *output)
{
	output[0] = T(input[0]);
	output[1] = T(input[1]);
	output[2] = T(input[2]);
}

template <typename T>
static void Generic_fillVect(const VectorXd &input, T *output)
{
	int num_rows = input.rows();

	for (auto r = 0; r < num_rows; r++)
	{
		output[r] = T(input(r));
	}
}

template <typename T>
static void Generic_fillMat(const MatrixXd &input, CMatrix<T> & output)
{
	int num_rows = input.rows();
	int num_cols = input.cols();

	for (int r = 0; r < num_rows; r++)
	{
		for(int c = 0; c < num_cols; c++){
			output(r,c) = T(input(r,c));
		}
	}
}

template <typename T>
static void Generic_fillMat(const CMatrix<T> & input, CMatrix<T> & output)
{
	int num_rows = input.num_rows;
	int num_cols = input.num_cols;
	for (int r = 0; r < num_rows; r++)
	{
		for(int c = 0; c < num_cols; c++){
			output(r,c) = input.data[r][c];
		}
	}
}


template <typename T>
static void Generic_DotProduct(const CMatrix<T> & input, const T *coefs, const int row_index, const int dim, T *output)
{
	T _output[3];
	_output[0] = T(0);
	_output[1] = T(0);
	_output[2] = T(0);

	int _int_row_index = row_index;
	int num_columns = dim;

	for (int c = 0; c < num_columns; c++)
	{
		T _out0 = input.data[0][c] * coefs[c];
		T _out1 = input.data[1][c] * coefs[c];
		T _out2 = input.data[2][c] * coefs[c];
		_output[0] += _out0;
		_output[1] += _out1;
		_output[2] += _out2;
	}
	output[0] = T(_output[0]);
	output[1] = T(_output[1]);
	output[2] = T(_output[2]);
}

VectorXd LinearizeEigenMat(MatrixXd & mat){
	int rows = mat.rows();
	int columns = mat.cols();

	VectorXd ans(rows*columns);

	for(int r = 0; r < rows; r++){
		for(int c = 0; c < columns; c++){
			int index = r*columns + c;
			ans(index) = mat(r,c);
		}
	}

	return ans;
}

class Constraint_bfmManager
{
public:
	Parameter_set SHAPE;
	Parameter_set TEX;
	Parameter_set EXP;

	Constraint_bfmManager(const Parameter_set &_SHAPE, const Parameter_set &_TEX, const Parameter_set &_EXP) : SHAPE{_SHAPE}, TEX{_TEX}, EXP{_EXP} {}
};

template <typename T>
class Generic_BFMUpdate
{
public:
	// array_shape. array_exp, array_tex = coefficients
	explicit Generic_BFMUpdate(const Parameter_set *_parameter_set, T *const _array, const int _dim) : parameter_set{_parameter_set}, m_array{_array}, dim{_dim}
	{
	}

	void setZero()
	{
		for (int i = 0; i < dim; i++)
		{
			m_array[i] = T(0);
		}
	}

	T *getData() const
	{
		return m_array;
	}

	/**
	 * Applies the parameters to average bfm mesh
	 * Output the resulting vertex positions.
	 */
	void apply_params(const int index, T *transformed_point) const
	{
		// input
		//  vecShapeMU: shape[53149*3] = 159447
		//  matShapePC: shape[53149, 199]
		// output
		//  current_shape: shape[53149*3]

		int _int_dim = int(dim);

		T tmpVecMu[_int_dim];
		Generic_fillVect(parameter_set->mean, tmpVecMu);

		T tmpCoef[_int_dim];
		for (auto i = 0; i < _int_dim; i++)
			tmpCoef[i] = m_array[i];

		T current_shape[3];

		const int cnt = 3 * index;
		T noise[3];
		noise[0] = T(0);
		noise[1] = T(0);
		noise[2] = T(0);

		int _int_cnt = cnt;

		CMatrix<T> _extracted_pc(3, _int_dim);
		for(unsigned int j = 0; j < _int_dim; j++){
			_extracted_pc.data[0][j] = T(parameter_set->pc(_int_cnt,j));
			_extracted_pc.data[1][j] = T(parameter_set->pc(_int_cnt+1,j));
			_extracted_pc.data[2][j] = T(parameter_set->pc(_int_cnt+2,j));
		}
		
		Generic_DotProduct(_extracted_pc, tmpCoef, cnt, dim, noise);
		current_shape[0] = tmpVecMu[_int_cnt] + noise[0];
		current_shape[1] = tmpVecMu[_int_cnt + 1] + noise[1];
		current_shape[2] = tmpVecMu[_int_cnt + 2] + noise[2];

		transformed_point[0] = current_shape[0];
		transformed_point[1] = current_shape[1];
		transformed_point[2] = current_shape[2];
	}

private:
	// m_shape. m_exp, m_tex = coefficients
	T *m_array;
	const Parameter_set * parameter_set;
	const int dim; //int
	const int num_vertices = 53149;
};

/**
 * Optimization constraints.
 */
class PointToPointConstraint
{
public:
	PointToPointConstraint(const Constraint_bfmManager *_constraint_bfmManager, const Vector3d &sourcePoint, const Vector3d &targetPoint, const double weight, const int _index) : constraint_bfmManager{_constraint_bfmManager}, m_sourcePoint{sourcePoint}, m_targetPoint{targetPoint}, m_weight{weight}, m_index{_index}
	{
	}

	template <typename T>
	bool operator()(const T *const _coefs_shape, const T *const _coefs_exp, T *residuals) const
	{

		// Given
		// POSE (to be optimized = M(SO(3)))
		//  pose: lie algebra(6DOF)
		//  pose[0,1,2] is angle-axis rotation.
		//  pose[3,4,5] is translation.

		// Source point p_s (red bunny point clouds)
		//  m_sourcePoint(Vector3f) in PointToPointConstraint (one vertex)

		// Target point p_t (green bunny point clouds)
		//  m_targetPoint(Vector3f) in PointToPointConstraint (one vertex)

		// weight (additional parameters to be optimized)
		//  m_weight (double) in PointToPointConstraint (one vertex)

		// poseIncrement.m_array(private) = pose
	 	// 159447 = 53149 * 3
		// T vecShapeMu[159447];
		// T vecTexMu[159447];
		// T vecExpMu[159447];

		// Generic_fillVect( constraint_bfmManager->SHAPE.mean, vecShapeMu);
		// Generic_fillVect(constraint_bfmManager->TEX.mean, vecTexMu);
		// Generic_fillVect( constraint_bfmManager->EXP.mean, vecExpMu);

		// CMatrix<T> matShapePc(159447,199);
		// CMatrix<T> matTexPc(159447,199);
		// CMatrix<T> matExpPc(159447,100);

		// Generic_fillMat(constraint_bfmManager->SHAPE.pc, matShapePc);
		// Generic_fillMat(constraint_bfmManager->TEX.pc, matTexPc);
		// Generic_fillMat(constraint_bfmManager->EXP.pc, matExpPc);

		Generic_BFMUpdate<T> bfmShapeUpdate = Generic_BFMUpdate<T>(&(constraint_bfmManager->SHAPE), const_cast<T *const>(_coefs_shape), dim_shape);
		Generic_BFMUpdate<T> bfmExpUpdate = Generic_BFMUpdate<T>(&(constraint_bfmManager->EXP), const_cast<T *const>(_coefs_exp), dim_exp);

		T updateShape_vertices[3];
		T updateExp_vertices[3];
		T sourcePoint[3];

		fillVector(m_sourcePoint, sourcePoint);

		const int _template_index = m_index;

		bfmShapeUpdate.apply_params(_template_index, updateShape_vertices);
		bfmExpUpdate.apply_params(_template_index, updateExp_vertices);

		T blendshape_vertices[3];
		blendshape_vertices[0] = updateShape_vertices[0] + updateExp_vertices[0];
		blendshape_vertices[1] = updateShape_vertices[1] + updateExp_vertices[1];
		blendshape_vertices[2] = updateShape_vertices[2] + updateExp_vertices[2];

		T targetPoint[3];
		fillVector(m_targetPoint, targetPoint);

		T subtract_st[3];

		subtract_st[0] = blendshape_vertices[0] - targetPoint[0];
		subtract_st[1] = blendshape_vertices[1] - targetPoint[1];
		subtract_st[2] = blendshape_vertices[2] - targetPoint[2];

		// double _weight = std::sqrt(m_weight);
		// double _LAMBDA = std::sqrt(LAMBDA);
		// T coeff = T(_weight * _LAMBDA);

		// Three residual
		// residuals[0] = subtract_st[0] * coeff;
		// residuals[1] = subtract_st[1] * coeff;
		// residuals[2] = subtract_st[2] * coeff;

		residuals[0] = subtract_st[0];
		residuals[1] = subtract_st[1];
		residuals[2] = subtract_st[2];

		return true;
	}

	static ceres::CostFunction *create(const Constraint_bfmManager *_constraint_bfmManager, const Vector3d &sourcePoint, const Vector3d &targetPoint, const double weight, const int index){
		return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 199, 100>(
			new PointToPointConstraint(_constraint_bfmManager, sourcePoint, targetPoint, weight, index)
			);
	}

protected:
	const Constraint_bfmManager *constraint_bfmManager;
	const Vector3d m_sourcePoint;
	const Vector3d m_targetPoint;
	const double m_weight;
	const int m_index;
	const int num_vertices = 53149;
	const double LAMBDA = 1.0;
	const int dim_shape = 199;
	const int dim_tex = 199;
	const int dim_exp = 100;
};

// class PointToPlaneConstraint {
// public:
// 	PointToPlaneConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const double weight) :
// 		m_sourcePoint{ sourcePoint },
// 		m_targetPoint{ targetPoint },
// 		m_targetNormal{ targetNormal },
// 		m_weight{ weight }
// 	{ }

// 	template <typename T>
// 	bool operator()(const T* const pose, T* residuals) const {

// 		//Given
// 		//POSE (to be optimized = M(SO(3)))
// 			// pose: lie algebra(6DOF)
// 			// pose[0,1,2] is angle-axis rotation.
// 			// pose[3,4,5] is translation.

// 		//Source point p_s (red bunny point clouds)
// 			// m_sourcePoint(Vector3f) in PointToPointConstraint (one vertex)

// 		//Target point p_t (green bunny point clouds)
// 			// m_targetPoint(Vector3f) in PointToPointConstraint (one vertex)

// 		//Target Normal (normal of vertex in green bunny point clouds)
// 			// m_targetNormal(Vector3f) in PointToPlaneConstraint (one vertex)

// 		//weight (additional parameters to be optimized)
// 			// m_weight (double) in PointToPointConstraint (one vertex)

// 		PoseIncrement<T> poseIncrement = PoseIncrement<T>(const_cast<T* const>(pose));

// 		// TODO: Implement the point-to-plane cost function.
// 		// The resulting 1D residual should be stored in the residuals array. To apply the pose
// 		// increment (pose parameters) to the source point, you can use the PoseIncrement class.
// 		// Important: Ceres automatically squares the cost function.

// 		/*
// 		*1: we pose-increment.apply()/transformed (rotate and translate) the source point with pose (we approximate so far)
// 		*2: subtract the target vertex coordinate from the transformed corresponding vertex coordinate
// 		*3: register as the residual
// 		*4: inner-product with subtracted vertex coordinates
// 		*/
// 		//poseIncrement.apply()
// 		T transformed_msP[3];

// 		//convert the Vector3f to T array
// 		T sourcePoint[3];
// 		fillVector(m_sourcePoint, sourcePoint);

// 		poseIncrement.apply(sourcePoint, transformed_msP);

// 		T subtract_st[3];

// 		T targetPoint[3];
// 		fillVector(m_targetPoint, targetPoint);

// 		subtract_st[0] = transformed_msP[0] - targetPoint[0];
// 		subtract_st[1] = transformed_msP[1] - targetPoint[1];
// 		subtract_st[2] = transformed_msP[2] - targetPoint[2];

// 		// residuals = m_targetNormal.dot(subtract_st);
// 		// auto dot_product = m_targetNormal[0]*subtract_st[0] + m_targetNormal[1]*subtract_st[1] + m_targetNormal[2]*subtract_st[2];

// 		double _weight = m_weight;
// 		double _LAMBDA = LAMBDA;
// 		T coeff = T(std::sqrt(_weight) * std::sqrt(_LAMBDA));

// 		T targetNormal[3];
// 		fillVector(m_targetNormal, targetNormal);

// 		//single residual
// 		residuals[0] = ceres::DotProduct(targetNormal, subtract_st) * coeff;
// 		return true;
// 	}

// 	static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const double weight) {
// 		return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, 6>(
// 			new PointToPlaneConstraint(sourcePoint, targetPoint, targetNormal, weight)
// 		);
// 	}

// protected:
// 	const Vector3f m_sourcePoint;
// 	const Vector3f m_targetPoint;
// 	const Vector3f m_targetNormal;
// 	const double m_weight;
// 	const double LAMBDA = 1.0f;
// };

/**
 * ICP optimizer - Abstract Base Class, using Ceres for optimization.
 */
class ICPOptimizer
{
public:
	ICPOptimizer() : m_bUsePointToPlaneConstraints{false},
					 m_nIterations{20},
					 m_nearestNeighborSearch{std::make_unique<NearestNeighborSearchFlann>()}
	{
	}

	void setMatchingMaxDistance(double maxDistance)
	{
		m_nearestNeighborSearch->setMatchingMaxDistance(maxDistance);
	}

	void usePointToPlaneConstraints(bool bUsePointToPlaneConstraints)
	{
		m_bUsePointToPlaneConstraints = bUsePointToPlaneConstraints;
	}

	void setNbOfIterations(unsigned nIterations)
	{
		m_nIterations = nIterations;
	}

	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams(const FacePointCloud &source, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM * bfm){};

protected:
	bool m_bUsePointToPlaneConstraints;
	unsigned m_nIterations;
	std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch;

	void pruneCorrespondences(const std::vector<Vector3d> &sourceNormals, const std::vector<Vector3d> &targetNormals, std::vector<Match> &matches)
	{
		const unsigned nPoints = sourceNormals.size();

		bool are_MINF = true;
		int counter_invalid_point = 0;

		for (unsigned i = 0; i < nPoints; i++)
		{
			Match &match = matches[i];
			if (match.idx >= 0)
			{
				const auto &sourceNormal = sourceNormals[i];
				const auto &targetNormal = targetNormals[match.idx];

				if(sourceNormal.allFinite() && targetNormal.allFinite()){
					are_MINF = false;
				}

				// TODO: Invalidate the match (set it to -1) if the angle between the normals is greater than 60
				// sourceNormal and targetNormal are normalized to length 1

				if(!are_MINF){
					double angle = acosf64x(sourceNormal.dot(targetNormal));
					if (angle > (double)(60.0))
					{
						match.idx = -1;
					}
				}
			}else{
				counter_invalid_point++;
			}
		}

		std::cout<<"invalid point: "<<counter_invalid_point<<std::endl;
	}
};

std::vector<Vector3d> convert_float2double(std::vector<Vector3f> &float_vector)
{
	int length = float_vector.size();
	std::vector<Vector3d> double_vector;
	for (int i = 0; i < length; i++)
	{
		Vector3d _temp = float_vector[i].cast<double>();
		double_vector.push_back(_temp);
	}
	return double_vector;
}

/**
 * ICP optimizer - using Ceres for optimization.
 */
class CeresICPOptimizer : public ICPOptimizer
{
public:
	CeresICPOptimizer() {}
	// shape tex, exp
	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams(const FacePointCloud &source, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM * bfm) override
	{
		// Build the index of the FLANN tree (for fast nearest neighbor lookup).
		int num_vertices = 53149;
		std::vector<Vector3f> _float_target_points = target.getPoints();
		std::cout<<"Target_points: "<<_float_target_points.size()<<std::endl;
		std::vector<Vector3f> _float_target_normals = target.getNormals();
		std::cout<<"Target_normals: "<<_float_target_normals.size()<<std::endl;
		std::vector<Vector3f> _float_source_points = source.getPoints();
		std::cout<<"Source_normals: "<<_float_source_points.size()<<std::endl;
		std::vector<Vector3d> target_points = convert_float2double(_float_target_points);
		std::vector<Vector3d> target_normals = convert_float2double(_float_target_normals);
		std::vector<Vector3d> source_points = convert_float2double(_float_source_points);

		m_nearestNeighborSearch->buildIndex(target_points);

		// simple addition shape and exp
		std::vector<double> estimated_coefs_shape;
		std::copy(_initial_coef_shape.begin(), _initial_coef_shape.end(), std::back_inserter(estimated_coefs_shape));
		std::vector<double> estimated_coefs_tex;
		std::copy(_initial_coef_tex.begin(), _initial_coef_tex.end(), std::back_inserter(estimated_coefs_tex));
		std::vector<double> estimated_coefs_exp;
		std::copy(_initial_coef_exp.begin(), _initial_coef_exp.end(), std::back_inserter(estimated_coefs_exp));

		std::cout<<"initial_coefs"<<std::endl;
		std::cout<<"shape: "<<estimated_coefs_shape.size()<<std::endl;
		std::cout<<"tex: "<<estimated_coefs_tex.size()<<std::endl;
		std::cout<<"exp: "<<estimated_coefs_exp.size()<<std::endl;

		double num_shapePc = 199;
		double num_texPc = 199;
		double num_expPc = 100;

		int _int_num_shapePc = 199;
		int _int_num_texPc = 199;
		int _int_num_expPc = 100;

		double _increment_coef_shape[199];
		double _increment_coef_tex[199];
		double _increment_coef_exp[100];

		// double vecMuShape[159447];
		// double vecMuTex[159447];
		// double vecMuExp[159447];

		// Generic_fillVect(SHAPE.mean, vecMuShape);
		// Generic_fillVect(TEX.mean, vecMuTex);
		// Generic_fillVect(EXP.mean, vecMuExp);

		// CMatrix<double> matPcShape(159447, 199);
		// CMatrix<double> matPcTex(159447, 199);
		// CMatrix<double> matPcExp(159447, 100);

		// Generic_fillMat(SHAPE.pc, matPcShape);
		// Generic_fillMat(TEX.pc, matPcTex);
		// Generic_fillMat(EXP.pc, matPcExp);

		auto bfmMeshShapeUpdate = Generic_BFMUpdate<double>(&SHAPE, _increment_coef_shape, num_shapePc);
		auto bfmMeshTexUpdate = Generic_BFMUpdate<double>(&TEX, _increment_coef_tex, num_texPc);
		auto bfmMeshExpUpdate = Generic_BFMUpdate<double>(&EXP, _increment_coef_exp, num_expPc);
		bfmMeshShapeUpdate.setZero();
		bfmMeshTexUpdate.setZero();
		bfmMeshExpUpdate.setZero();

		// iterative optimization
		for (int i = 0; i < m_nIterations; ++i)
		{
			// Compute the matches.
			std::cout << "Matching points ..." << std::endl;
			clock_t begin = clock();

			// get deformed vertices given estimated_shape_coefficients
			//  std::vector<Vector3f> deformed_source_vertices;
			//  std::vector<Vector3f> deformed_source_normals;
			// TODO:: create a function which can return deformed_source_vertices given coefficients for the shape
			std::vector<Vector3f> updated_BFM_vertex_pos;
			std::vector<Vector3f> updated_BFM_vertex_rgb;
			std::vector<Vector3i> updated_BFM_triangle_list;

			std::string f_name = "../output/transformed_mesh_" + std::to_string(i) + ".ply";
			std::tie(updated_BFM_vertex_pos, updated_BFM_vertex_rgb, updated_BFM_triangle_list) = bfm->transformedBFMMesh(f_name, estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp, true);

			FacePointCloud transformed_sourceMesh{updated_BFM_vertex_pos, updated_BFM_triangle_list};

			std::vector<Vector3d> d_transformed_vertex_pos = convert_float2double(transformed_sourceMesh.getPoints());
			std::vector<Vector3d> d_transformed_normals = convert_float2double(transformed_sourceMesh.getNormals());

			// transformed_sourceMesh.getPoints(); //<= return std::vector<Vector3f> points
			// transformed_sourceMesh.getNormals(); //<= return std::vector<Vector3f> normals

			// matches contains the correspondences
			auto matches = m_nearestNeighborSearch->queryMatches(d_transformed_vertex_pos);
			// for(unsigned int i = 0; i<matches.size(); i++){
			// 	std::cout<<i<<"th match: "<<matches[i].idx<<", weight: "<<matches[i].weight<<std::endl;
			// }
			pruneCorrespondences(d_transformed_normals, target_normals, matches);

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			// Prepare point-to-point and point-to-plane constraints.
			ceres::Problem problem;
			// Add residual w.r.t shape parameter
			prepareConstraints(d_transformed_vertex_pos, target_points, target_normals, matches, bfmMeshShapeUpdate, bfmMeshExpUpdate, problem, SHAPE, TEX, EXP);
			// Add residual w.r.t expression parameter
			// Configure options for the solver.
			ceres::Solver::Options options;
			configureSolver(options);

			// Run the solver (for one iteration).
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			// std::cout << summary.BriefReport() << std::endl;
			std::cout << summary.FullReport() << std::endl;

			double *delta_coefs_shape = bfmMeshShapeUpdate.getData();
			double *delta_coefs_tex = bfmMeshTexUpdate.getData();
			double *delta_coefs_exp = bfmMeshExpUpdate.getData();

			for (unsigned int i = 0; i < num_shapePc; i++)
			{
				estimated_coefs_shape[i] += delta_coefs_shape[i];
				estimated_coefs_tex[i] += delta_coefs_tex[i];
				if (i < num_expPc)
				{
					estimated_coefs_exp[i] += delta_coefs_exp[i];
				}
			}

			std::cout << "Optimization iteration done." << std::endl;
		}
		std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> _result_coeffs;
		_result_coeffs = std::make_tuple(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);
		return _result_coeffs;
	}

private:
	void configureSolver(ceres::Solver::Options &options)
	{
		// Ceres options.
		options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
		options.use_nonmonotonic_steps = false;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = 1;
		options.max_num_iterations = 1;
		options.num_threads = 8;
	}

	void prepareConstraints(const std::vector<Vector3d> &sourcePoints, const std::vector<Vector3d> &targetPoints, const std::vector<Vector3d> &targetNormals, const std::vector<Match> matches, const Generic_BFMUpdate<double> &shape_BFMUpdate, const Generic_BFMUpdate<double> &exp_BFMUpdate, ceres::Problem &problem, const Parameter_set &_SHAPE, const Parameter_set &_TEX, const Parameter_set &_EXP) const
	{
		// shape_BFMUpdate (containing the lie algebra) is to be optimized
		/* shape_BFMUpdate will return the address of array (6 elements)
		double* pose = shape_BFMUpdate.getData();
		double* rotation = pose;
		double* translation = pose + 3;
		*/
		const int nPoints = sourcePoints.size();

		std::cout<<"Nearest neighbor matched pairs: "<<matches.size()<<std::endl;

		for (int i = 0; i < nPoints; ++i)
		{
			// matches[i] contains that corresponding vertex index of i
			/*
			 *i: index of source point
			 *match.index: index of corresponding target point
			 *match.weight: weight of the correspondence
			 */
			const auto match = matches[i];
			if (match.idx >= 0)
			{
				const auto &sourcePoint = sourcePoints[i];
				const auto &targetPoint = targetPoints[match.idx];
				const auto &weight = match.weight;

				std::cout<<"weight: "<<match.weight<<std::endl;

				if (!sourcePoint.allFinite() || !targetPoint.allFinite())
					continue;

				// TODO: Create a new point-to-point cost function and add it as constraint (i.e. residual block)
				// to the Ceres problem

				// PointToPointConstraint ptpc = {sourcePoint, targetPoint, weight};
				Constraint_bfmManager constraint_bfmManager(_SHAPE, _TEX, _EXP);
				ceres::CostFunction *cost_function = PointToPointConstraint::create(&constraint_bfmManager, sourcePoint, targetPoint, weight, i);
				problem.AddResidualBlock(cost_function, nullptr, shape_BFMUpdate.getData(), exp_BFMUpdate.getData());

				// if (m_bUsePointToPlaneConstraints) {
				// 	const auto& targetNormal = targetNormals[match.idx];

				// 	if (!targetNormal.allFinite())
				// 		continue;

				// 	// TODO: Create a new point-to-plane cost function and add it as constraint (i.e. residual block)
				// 	// to the Ceres problem.

				// 	// PointToPlaneConstraint n_ptpc = {sourcePoint, targetPoint, targetNormal, weight};
				// 	ceres::CostFunction * cost_function = PointToPlaneConstraint::create(sourcePoint, targetPoint, targetNormal, weight);
				// 	problem.AddResidualBlock(cost_function, nullptr, shape_BFMUpdate.getData());

				// }
			}
		}
	}
};
