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
#include <tuple>
#include <set>

const unsigned int num_shape_pcs = 1;
const unsigned int num_tex_pcs = 1;
const unsigned int num_exp_pcs = 1;

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
static void Generic_DotProduct(const T** input, const T *coefs, const int row_index, const int dim, T *output)
{
	T _output[3];
	_output[0] = T(0);
	_output[1] = T(0);
	_output[2] = T(0);

	int _int_row_index = row_index;
	int num_columns = dim;

	for (int c = 0; c < num_columns; c++)
	{
		T _out0 = input[0][c] * coefs[c];
		T _out1 = input[1][c] * coefs[c];
		T _out2 = input[2][c] * coefs[c];
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

	Constraint_bfmManager(const Parameter_set &_SHAPE, const Parameter_set &_TEX, const Parameter_set &_EXP, std::vector<Vector3i> _triangle_list) : SHAPE{_SHAPE}, TEX{_TEX}, EXP{_EXP}, triangle_list{_triangle_list} {}

	inline Parameter_set* get_SHAPEParamSet(){
		return &SHAPE;
	}

	inline Parameter_set* get_TEXParamSet(){
		return &TEX;
	}

	inline Parameter_set* get_EXPParamSet(){
		return &EXP;
	}

	// inline MatrixXd * get_SHAPEPcs(){
	// 	return &(SHAPE.pc);
	// }

	// inline VectorXd * get_SHAPEMu(){
	// 	return &(SHAPE.mean);
	// }

	// inline MatrixXd * get_TEXPcs(){
	// 	return &(TEX.pc);
	// }

	// inline VectorXd * get_TEXMu(){
	// 	return &(TEX.mean);
	// }

	// inline MatrixXd * get_EXPPcs(){
	// 	return &(EXP.pc);
	// }

	// inline VectorXd * get_EXPMu(){
	// 	return &(EXP.mean);
	// }

	template <typename T>
	void get_SHAPEPc(const int _index, const int _dim, CMatrix<T> & pc){
		int index = int(_index);
		int dim = int(_dim);

		for(unsigned int j = 0; j < dim; j++){
			T x, y, z;
			x = T(SHAPE.pc(index, j));
			y = T(SHAPE.pc(index+1, j));
			z = T(SHAPE.pc(index+2, j));
			pc.data[0][j] = x;
			pc.data[1][j] = y;
			pc.data[2][j] = z;
		}
	}

	template <typename T>
	void get_TEXPc(const int _index, const int _dim, CMatrix<T> & pc){
		int index = int(_index);
		int dim = int(_dim);
		for(unsigned int j = 0; j < dim; j++){
			T x, y, z;
			x = T(TEX.pc(index, j));
			y = T(TEX.pc(index+1, j));
			z = T(TEX.pc(index+2, j));
			pc.data[0][j] = x;
			pc.data[1][j] = y;
			pc.data[2][j] = z;
		}
	}

	template <typename T>
	void get_EXPPc(const int _index, const int _dim, CMatrix<T> & pc){
		int index = int(_index);
		int dim = int(_dim);
		for(unsigned int j = 0; j < dim; j++){
			T x, y, z;
			x = T(EXP.pc(index, j));
			y = T(EXP.pc(index+1, j));
			z = T(EXP.pc(index+2, j));
			pc.data[0][j] = x;
			pc.data[1][j] = y;
			pc.data[2][j] = z;
		}
	}

	template <typename T>
	void get_SHAPEMu(T * means, int _num_vertices){

		for(unsigned int i = 0; i < _num_vertices; i++){
			T x, y, z;
			x = T(SHAPE.mean(3*i));
			y = T(SHAPE.mean(3*i+1));
			z = T(SHAPE.mean(3*i+2));
			means[3*i] = x;
			means[3*i+1] = y;
			means[3*i+2] = z;
		}

	}

	template <typename T>
	void get_TEXMu(T * means, int _num_vertices){

		for(unsigned int i = 0; i < _num_vertices; i++){
			T x, y, z;
			x = T(TEX.mean(3*i));
			y = T(TEX.mean(3*i+1));
			z = T(TEX.mean(3*i+2));
			means[3*i] = x;
			means[3*i+1] = y;
			means[3*i+2] = z;
		}

	}

	template <typename T>
	void get_EXPMu(T * means, int _num_vertices){

		for(unsigned int i = 0; i < num_vertices; i++){
			T x, y, z;
			x = T(EXP.mean(3*i));
			y = T(EXP.mean(3*i+1));
			z = T(EXP.mean(3*i+2));
			means[3*i] = x;
			means[3*i+1] = y;
			means[3*i+2] = z;
		}
	}

	inline double *GetPointer2array(std::vector<double> _coeffs){
		double * pointer = (double*)malloc(_coeffs.size()*sizeof(double));
		double * id = pointer;
		int counter = 0;

		while(counter < _coeffs.size()){
			*id = _coeffs[counter];
			counter++;
			id++;
		}
		
		return pointer;
	}

	template<typename Derived>
	Matrix<Derived, Dynamic, 1> get_component(const Derived *const aCoef, 
		const VectorXd &vecMu, const MatrixXd &matPc, unsigned int nLength) const
	{
		assert(aCoef != nullptr);
		// assert(nLength >= 0);

		Matrix<Derived, Dynamic, 1> tmpCoef(nLength);
		for(auto i = 0u; i < nLength; i++)
			tmpCoef(i) = aCoef[i];

		Matrix<Derived, Dynamic, 1> tmpMu = vecMu.cast<Derived>();
		Matrix<Derived, Dynamic, Dynamic> tmpPc = matPc.cast<Derived>();
		// return tmpMu + tmpPc * tmpCoef.cwiseProduct(tmpEv);
		return tmpMu + tmpPc * tmpCoef;
	}

	std::pair<std::vector<Vector3d>, std::vector<Vector3d>> get_tranformedBFMMesh(std::vector<double> & Coef_shape, std::vector<double> & Coef_tex, std::vector<double> & Coef_exp){
		int num_shapePcs = Coef_shape.size();
		int num_texPcs = Coef_tex.size();
		int num_expPcs = Coef_exp.size();
		
		double * p2shape = GetPointer2array(Coef_shape);
		double * p2tex = GetPointer2array(Coef_tex);
		double * p2exp = GetPointer2array(Coef_exp);

		if(num_shapePcs != num_shape_pcs || num_texPcs != num_tex_pcs || num_expPcs != num_exp_pcs){
			std::cout<<"Different number of coefficients are given"<<std::endl;
		}

		VectorXd vecCurrentShape = get_component(p2shape, SHAPE.mean, SHAPE.pc, num_shapePcs);
		VectorXd vecCurrentTex = get_component(p2tex, TEX.mean, TEX.pc, num_texPcs);
		VectorXd vecCurrentExp = get_component(p2exp, EXP.mean, EXP.pc, num_expPcs);
		VectorXd vecCurrentBlendshape = vecCurrentShape + vecCurrentExp;

		std::vector<Vector3d> current_vertices_pos;
		std::vector<Vector3d> current_vertices_rgb;
		unsigned int cnt = 0;
		for (unsigned int i = 0; i < num_vertices; i++){
			
			double *x, *y, *z;
			double *r, *g, *b;
			x = &(vecCurrentShape(i * 3));
			y = &(vecCurrentTex(i * 3 + 1));
			z = &(vecCurrentExp(i * 3 + 2));

			r = &(vecCurrentTex(i * 3));
			g = &(vecCurrentTex(i * 3 + 1));
			b = &(vecCurrentTex(i * 3 + 2));

			Vector3d _current_shape = {*x, *y, *z};
			Vector3d _current_rgb = {*r, *g, *b};

			current_vertices_pos.push_back(_current_shape);
			current_vertices_rgb.push_back(_current_rgb);

		}
		
		return {current_vertices_pos, current_vertices_rgb};
	}

private:
	Parameter_set SHAPE;
	Parameter_set TEX;
	Parameter_set EXP;
	std::vector<Vector3i> triangle_list;
	const int num_vertices = 53149;	
};

template <typename T>
class Generic_BFMUpdate
{
public:
	// array_shape. array_exp, array_tex = coefficients
	explicit Generic_BFMUpdate(T * _means, T ** _pcs, T *const _array, const int _dim) : means{_means}, pcs{_pcs}, m_array{_array}, dim{_dim}
	{
	}

	explicit Generic_BFMUpdate(Parameter_set * _parameter_set , T *const _array, const int _dim) : parameter_set{_parameter_set}, m_array{_array}, dim{_dim}{

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

		T tmpMu[3 * num_vertices];
		for (auto i = 0; i = num_vertices; i++){
			tmpMu[3*i] = T(means[3*i]);
			tmpMu[3*i+1] = T(means[3*i+1]);
			tmpMu[3*i+2] = T(means[3*i+2]);
		}

		T tmpPcs[3][_int_dim];

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < _int_dim; j++) {
                tmpPcs[i][j] = T(pcs[i][j]);
            }
        }

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

		for (int c = 0; c < _int_dim; c++)
		{
			T _out0 = tmpPcs[0][c] * tmpCoef[c];
			T _out1 = tmpPcs[1][c] * tmpCoef[c];
			T _out2 = tmpPcs[2][c] * tmpCoef[c];
			noise[0] += _out0;
			noise[1] += _out1;
			noise[2] += _out2;
		}

		current_shape[0] = tmpMu[_int_cnt] + noise[0];
		current_shape[1] = tmpMu[_int_cnt + 1] + noise[1];
		current_shape[2] = tmpMu[_int_cnt + 2] + noise[2];

		transformed_point[0] = current_shape[0];
		transformed_point[1] = current_shape[1];
		transformed_point[2] = current_shape[2];
	}

private:
	// m_shape. m_exp, m_tex = coefficients
	T *m_array;
	T **pcs;
	T *means;
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
	PointToPointConstraint(Constraint_bfmManager *_constraint_bfmManager, const Vector3d &sourcePoint, const Vector3d &targetPoint, const double weight, const int _index) : constraint_bfmManager{_constraint_bfmManager}, m_sourcePoint{sourcePoint}, m_targetPoint{targetPoint}, m_weight{weight}, m_index{_index}
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

		const int _template_index = m_index;
		const int _index = 3 * m_index;

		std::cout<<_index<<std::endl;

		CMatrix<T> _shape_pc(3, dim_shape);
		constraint_bfmManager->get_SHAPEPc<T>(_index, dim_shape, _shape_pc);
		T **_shape_matPc = _shape_pc.data;
		T _shape_mu[num_vertices];
		constraint_bfmManager->get_SHAPEMu<T>(_shape_mu, num_vertices);
		
		CMatrix<T> _exp_pc(3, dim_exp);
		constraint_bfmManager->get_EXPPc<T>(_index, dim_exp, _exp_pc);
		T ** _exp_matPc = _exp_pc.data;
		T _exp_mu[num_vertices];
		constraint_bfmManager->get_EXPMu<T>(_exp_mu, num_vertices);

		Generic_BFMUpdate<T> bfmShapeUpdate = Generic_BFMUpdate<T>(_shape_mu, _shape_matPc, const_cast<T *const>(_coefs_shape), dim_shape);
		Generic_BFMUpdate<T> bfmExpUpdate = Generic_BFMUpdate<T>(_exp_mu, _exp_matPc, const_cast<T *const>(_coefs_exp), dim_exp);

		T updateShape_vertices[3];
		T updateExp_vertices[3];
		T sourcePoint[3];

		fillVector(m_sourcePoint, sourcePoint);

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

	static ceres::CostFunction *create(Constraint_bfmManager *_constraint_bfmManager, const Vector3d &sourcePoint, const Vector3d &targetPoint, const double weight, const int index){
		return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 1, 1>(
			new PointToPointConstraint(_constraint_bfmManager, sourcePoint, targetPoint, weight, index)
			);
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	const Vector3d m_sourcePoint;
	const Vector3d m_targetPoint;
	const double m_weight;
	const int m_index;
	const int num_vertices = 53149;
	const double LAMBDA = 1.0;
	const int dim_shape = num_shape_pcs;
	const int dim_tex = num_tex_pcs;
	const int dim_exp = num_exp_pcs;
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

	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams(const FacePointCloud &source, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM & bfm){};

protected:
	bool m_bUsePointToPlaneConstraints;
	unsigned m_nIterations;
	std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch;

	void pruneCorrespondences(const std::vector<Vector3d> &sourceNormals, const std::vector<Vector3d> &targetNormals, std::vector<Match> &matches)
	{
		const unsigned nPoints = sourceNormals.size();

		int counter_invalid_point = 0;
		int counter_valid_point = 0;


		for (unsigned i = 0; i < nPoints; i++)
		{
			bool are_MINF = true;
			Match &match = matches[i];
			if (match.idx >= 0)
			{
				const auto &sourceNormal = sourceNormals[i];
				const auto &targetNormal = targetNormals[match.idx];

				if(sourceNormal.allFinite() && targetNormal.allFinite()){
					are_MINF = false;
					counter_valid_point++;
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
		std::cout<<"valid point: "<<counter_valid_point<<std::endl;
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

std::vector<Vector3f> convert_double2float(std::vector<Vector3d> &double_vector)
{
	int length = double_vector.size();
	std::vector<Vector3f> float_vector;
	for (int i = 0; i < length; i++)
	{
		Vector3f _temp = double_vector[i].cast<float>();
		float_vector.push_back(_temp);
	}
	return float_vector;
}

/**
 * ICP optimizer - using Ceres for optimization.
 */
class CeresICPOptimizer : public ICPOptimizer
{
public:
	CeresICPOptimizer() {}
	// shape tex, exp
	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams(const FacePointCloud &source, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM & bfm) override
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

		m_nearestNeighborSearch->buildIndex(_float_target_points);

		// simple addition shape and exp
		std::vector<double> estimated_coefs_shape;
		std::copy(_initial_coef_shape.begin(), _initial_coef_shape.end(), std::back_inserter(estimated_coefs_shape));
		std::vector<double> estimated_coefs_tex;
		std::copy(_initial_coef_tex.begin(), _initial_coef_tex.end(), std::back_inserter(estimated_coefs_tex));
		std::vector<double> estimated_coefs_exp;
		std::copy(_initial_coef_exp.begin(), _initial_coef_exp.end(), std::back_inserter(estimated_coefs_exp));
		
		std::cout<<"Estimated_coefs_shape"<<std::endl;
		for(auto item_shape: estimated_coefs_shape){
			std::cout<<item_shape<<std::endl;
		}

		std::cout<<"Estimated_coefs_tex"<<std::endl;
		for(auto item_tex: estimated_coefs_tex){
			std::cout<<item_tex<<std::endl;
		}
	
		std::cout<<"Estimated_coefs_exp"<<std::endl;
		for(auto item_exp: estimated_coefs_exp){
			std::cout<<item_exp<<std::endl;
		}

		std::cout<<"initial_coefs"<<std::endl;
		std::cout<<"shape: "<<estimated_coefs_shape.size()<<std::endl;
		std::cout<<"tex: "<<estimated_coefs_tex.size()<<std::endl;
		std::cout<<"exp: "<<estimated_coefs_exp.size()<<std::endl;

		int _int_num_shapePc = num_shape_pcs;
		int _int_num_texPc = num_tex_pcs;
		int _int_num_expPc = num_exp_pcs;

		double _increment_coef_shape[_int_num_shapePc];
		double _increment_coef_tex[_int_num_texPc];
		double _increment_coef_exp[_int_num_expPc];

		std::vector<Vector3f> before_BFM_vertex_pos;
		std::vector<Vector3f> before_BFM_vertex_rgb;
		std::vector<Vector3i> BFM_triangle_list;
			
		std::string f_name = "../output/before_paramEst.ply";
		std::tie(before_BFM_vertex_pos, before_BFM_vertex_rgb, BFM_triangle_list) = bfm.writeAveBFMmesh(f_name, false);


		Constraint_bfmManager constraint_bfmManager(SHAPE, TEX, EXP, BFM_triangle_list);

		// CMatrix<T> * _shape_pc = constraint_bfmManager->get_SHAPEPc(_index, dim_shape);
		// T **const _shape_matPc = _shape_pc->data;
		// T *const _shape_mu = constraint_bfmManager->get_SHAPEMu();
		// CMatrix<T> * _exp_pc = constraint_bfmManager->get_EXPPc(_index, dim_exp);
		// T **const _exp_matPc = _exp_pc->data;
		// T *const _exp_mu = constraint_bfmManager->get_EXPMu();

		auto bfmMeshShapeUpdate = Generic_BFMUpdate<double>(&SHAPE, _increment_coef_shape, _int_num_shapePc);
		auto bfmMeshTexUpdate = Generic_BFMUpdate<double>(&TEX, _increment_coef_tex, _int_num_texPc);
		auto bfmMeshExpUpdate = Generic_BFMUpdate<double>(&EXP, _increment_coef_exp, _int_num_expPc);
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

			//here transformed TODO
			std::vector<Vector3d> updated_BFM_vertex_pos;
			std::vector<Vector3d> updated_BFM_vertex_rgb;
			std::tie(updated_BFM_vertex_pos, updated_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);

			std::vector<Vector3f> float_updated_BFM_vertex_pos = convert_double2float(updated_BFM_vertex_pos);
			
			// this should be given std::vector<Vector3f> not double
			FacePointCloud transformed_sourceMesh{float_updated_BFM_vertex_pos, BFM_triangle_list};

			// std::cout<<"updated_BFM_vertex_pos"<<std::endl;
			// for(int s = 0; s < updated_BFM_vertex_pos.size(); s++){
			// 	std::cout<<s<<"th vertex pos: "<<updated_BFM_vertex_pos[s].transpose()<<std::endl;
			// }

			std::vector<Vector3d> d_transformed_vertex_pos = convert_float2double(transformed_sourceMesh.getPoints());
			std::vector<Vector3d> d_transformed_normals = convert_float2double(transformed_sourceMesh.getNormals());

			// transformed_sourceMesh.getPoints(); //<= return std::vector<Vector3f> points
			// transformed_sourceMesh.getNormals(); //<= return std::vector<Vector3f> normals

			// matches contains the correspondences
			auto matches = m_nearestNeighborSearch->queryMatches(transformed_sourceMesh.getPoints());
			// for(unsigned int i = 0; i<matches.size(); i++){
			// 	std::cout<<i<<"th match: "<<matches[i].idx<<", weight: "<<matches[i].weight<<std::endl;
			// }
			pruneCorrespondences(d_transformed_normals, target_normals, matches);

			// std::cout<<"d_transformed_vertex_pos"<<std::endl;
			// for(int s = 0; s < d_transformed_vertex_pos.size(); s++){
			// 	std::cout<<s<<"th vertex pos: "<<d_transformed_vertex_pos[s].transpose()<<std::endl;
			// }

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			// Prepare point-to-point and point-to-plane constraints.
			ceres::Problem problem;
			// Add residual w.r.t shape parameter
			prepareConstraints(d_transformed_vertex_pos, BFM_triangle_list, target_points, target_normals, matches, bfmMeshShapeUpdate, bfmMeshExpUpdate, problem, &constraint_bfmManager);
			// Add residual w.r.t expression parameter
			// Configure options for the solver.
			std::cout<<"Start to solve"<<std::endl;
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

			for (unsigned int i = 0; i < _int_num_shapePc; i++)
			{
				estimated_coefs_shape[i] += delta_coefs_shape[i];
				estimated_coefs_tex[i] += delta_coefs_tex[i];
				if (i < _int_num_expPc)
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

	void prepareConstraints(const std::vector<Vector3d> &sourcePoints, const std::vector<Vector3i> &source_triangle_list, const std::vector<Vector3d> &targetPoints, const std::vector<Vector3d> &targetNormals, const std::vector<Match> matches, const Generic_BFMUpdate<double> &shape_BFMUpdate, const Generic_BFMUpdate<double> &exp_BFMUpdate, ceres::Problem &problem, Constraint_bfmManager * _constraint_bfmManager) const
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
				// std::cout<<"source point"<<sourcePoints[i]<<std::endl;
				const auto &targetPoint = targetPoints[match.idx];
				// std::cout<<"target point"<<targetPoints[match.idx]<<std::endl;
				const auto &weight = match.weight;

				// std::cout<<i<<"th weight: "<<match.weight<<std::endl;
				std::cout<<i<<"point"<<std::endl;

				if (!sourcePoint.allFinite() || !targetPoint.allFinite() || !targetNormals[match.idx].allFinite())
					continue;
				

				// TODO: Create a new point-to-point cost function and add it as constraint (i.e. residual block)
				// to the Ceres problem

				// PointToPointConstraint ptpc = {sourcePoint, targetPoint, weight};
				ceres::CostFunction *cost_function = PointToPointConstraint::create(_constraint_bfmManager, sourcePoint, targetPoint, weight, i);
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
