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

const unsigned int global_num_shape_pcs = 75; //199 //50
const unsigned int global_num_tex_pcs = 50; //199  //50
const unsigned int global_num_exp_pcs = 50; //100  //50
const unsigned int global_num_vertices = 53149;
const double global_reg_lambda = (1.0)*1e1;
const double global_regSHAPE_lambda = 1e1; //1e6
const double global_regEXP_lambda = 1.0;  // 5
const double global_regTEX_lambda = 1.0; // ./1.0
const double global_sparse_lambda = 3.0; // 1.0
const double global_dense_lambda = 1.0; // 1.0
const double global_color_lambda = 2e1; // 1.0

class WeightLS{
public:
	const int num_weightLS;
	WeightLS(const int len): num_weightLS(len){}

	inline const int get_num_weightLS(){
		return num_weightLS;
	}
private:

};

void set_global_variables(int i0, int i1, int i2, int i3){
	const unsigned int global_num_shape_pcs = i0;
	const unsigned int global_num_tex_pcs = i1;
	const unsigned int global_num_exp_pcs = i2;
	const double global_reg_lambda = double(i3);
}

int inline float2color(const float & _f_color){
	return ((int)(_f_color*255));
}

void Generic_writeFaceMeshPly(std::string fn, std::vector<Vector3f> & point_clouds, std::vector<Vector3f> & point_colors, std::vector<Vector3i> & triangle_list){
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
        out << "element vertex " << point_clouds.size() << "\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
		out << "property uchar red\n";
		out << "property uchar green\n";
		out << "property uchar blue\n";
		out << "property uchar alpha\n";
		out << "element face " << triangle_list.size() << "\n";
		out << "property list uchar int vertex_indices\n";
		out << "end_header\n";

        // unsigned long int cnt = 0;
        for (int i = 0; i < point_clouds.size(); i++)
        {
			float x, y, z;
			int r,g,b;
			x = point_clouds[i].x();
			y = point_clouds[i].y();
			z = point_clouds[i].z();

			r = float2color(point_colors[i].x());
			g = float2color(point_colors[i].y());
			b = float2color(point_colors[i].z());

			if(point_clouds[i].allFinite()){
            	out<<x<<" "<<y<<" "<<z<<" "<<r<<" "<<g<<" "<<b<<" "<<"255"<<"\n";
			}else{
            	out<<"0.0 0.0 0.0"<<"\n";
			}
        }

		unsigned char N_VER_PER_FACE = 3;
		for (Vector3i tuple : triangle_list)
		{
			out<<(int)N_VER_PER_FACE<<" "<<tuple.x()<<" "<<tuple.y()<<" "<<tuple.z()<<"\n";
		}

        out.close();

        std::cout<<"Finish face mesh ply ("<<fn<<")"<<std::endl;
}

std::vector<Vector3f> get_GeoErrorPointColors(std::vector<float> & dists, float max, float min){
	//assign colors to the vertex according to the Geometric error
	int num_points = dists.size();
	float threshold = 4;
	static std::vector<Vector3f> error_RGB;
	error_RGB.reserve(num_points);

	if(num_points != global_num_vertices){
		std::cout<<"Errors are not provided for all vertices"<<std::endl;
	}

	float average_error = 0.0;

	std::cout<<"ratio"<<std::endl;

	for(int i = 0; i < num_points; i++){
		float error = dists[i];
		float r=1.0;
		float g=1.0;
		float b=1.0;
		float ratio = (error-min)/(max-min);
		float green_ratio = 0.0;
		float blue_ratio = ratio;

		if(ratio<=1.0){
			Vector3f color = {r*(ratio), 0.0,  g*(1-blue_ratio)};
			average_error+=(sqrt(error));
			error_RGB.push_back(color);
		}else{
			Vector3f color = {1.0, 0.0, 0.0};
			error_RGB.push_back(color);
		}
	}

	average_error/=num_points;
	std::cout<<"Average error among valid points: "<<average_error<<std::endl;

	return error_RGB;
}

template <class T>
class CMatrix
{
public:
	int num_rows, num_cols;
	T **data;
	CMatrix(int rows, int cols)
	{
		setRow(rows);
		setCol(cols);

		num_rows = rows;
		num_cols = cols;

		data = new T *[rows]; // replaced "int" for "T"

		for (int i = 0; i < row; i++)
		{
			data[i] = new T[cols]; // replaced "int" for "T"
		}

		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				data[i][j] = T(0); // replaced "int" for "T"
			}
		}
	}

	void print();
	void setRow(int r) { row = r; }
	void setCol(int c) { col = c; }
	int rows() { return row; }
	int cols() { return col; }
	T &operator()(int row, int col);

private:
	int row, col;
};

template <class T>
void CMatrix<T>::print()
{
	int i, j;

	for (i = 0; i < row; i++) // Here you used to have row hard coded to 4
	{
		for (j = 0; j < col; j++) // Here you used to have col hard coded to 4
		{
			printf("%.1f    ", (float)data[i][j]);
		}
		printf("\n");
	}
}

// Recently added
template <class T>
T &CMatrix<T>::operator()(int row, int col)
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
static inline void fillVectorUchar(const Vector3uc &input, T *output)
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
static void Generic_fillMat(const MatrixXd &input, CMatrix<T> &output)
{
	int num_rows = input.rows();
	int num_cols = input.cols();

	for (int r = 0; r < num_rows; r++)
	{
		for (int c = 0; c < num_cols; c++)
		{
			output(r, c) = T(input(r, c));
		}
	}
}

template <typename T>
static void Generic_fillMat(const CMatrix<T> &input, CMatrix<T> &output)
{
	int num_rows = input.num_rows;
	int num_cols = input.num_cols;
	for (int r = 0; r < num_rows; r++)
	{
		for (int c = 0; c < num_cols; c++)
		{
			output(r, c) = input.data[r][c];
		}
	}
}

template <typename T>
static void get_norm(T* vect, T & out){
	const T sq_x = vect[0]*vect[0];
	const T sq_y = vect[1]*vect[1];
	const T sq_z = vect[2]*vect[2];
	const T norm = sq_x + sq_y + sq_z;

	const T _temp_norm = sqrt(norm);
	out = _temp_norm;
}

template <typename T>
static void Generic_DotProduct(const T **input, const T *coefs, const int row_index, const int dim, T *output)
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

VectorXd LinearizeEigenMat(MatrixXd &mat)
{
	int rows = mat.rows();
	int columns = mat.cols();

	VectorXd ans(rows * columns);

	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < columns; c++)
		{
			int index = r * columns + c;
			ans(index) = mat(r, c);
		}
	}

	return ans;
}

class Constraint_bfmManager
{
public:
	Constraint_bfmManager(const Parameter_set &_SHAPE, const Parameter_set &_TEX, const Parameter_set &_EXP, std::vector<Vector3i> _triangle_list) : SHAPE{_SHAPE}, TEX{_TEX}, EXP{_EXP}, triangle_list{_triangle_list} {}

	inline Parameter_set *get_SHAPEParamSet()
	{
		return &SHAPE;
	}

	inline Parameter_set *get_TEXParamSet()
	{
		return &TEX;
	}

	inline Parameter_set *get_EXPParamSet()
	{
		return &EXP;
	}


	template <typename T>
	static void set_coefs(const std::vector<double> _in_coefs, T* out){
		int counter = 0;
		for(counter=0; counter < _in_coefs.size(); counter++){
			out[counter] = T(_in_coefs[counter]);
		}
	}

	template <typename T>
	void get_SHAPEPc(const int _index, const int _dim, T **pc)
	{
		int index = int(_index);
		int dim = int(_dim);

		for (unsigned int j = 0; j < dim; j++)
		{
			auto x = SHAPE.pc(index, j);
			auto y = SHAPE.pc(index + 1, j);
			auto z = SHAPE.pc(index + 2, j);
			pc[0][j] = T(x);
			pc[1][j] = T(y);
			pc[2][j] = T(z);
		}
	}

	template <typename T>
	void get_TEXPc(const int _index, const int _dim, T **pc)
	{
		int index = int(_index);
		int dim = int(_dim);
		for (unsigned int j = 0; j < dim; j++)
		{
			auto x = (TEX.pc(index, j));
			auto y = (TEX.pc(index + 1, j));
			auto z = (TEX.pc(index + 2, j));
			pc[0][j] = T(x);
			pc[1][j] = T(y);
			pc[2][j] = T(z);
		}
	}

	template <typename T>
	void get_EXPPc(const int _index, const int _dim, T **pc)
	{
		int index = int(_index);
		int dim = int(_dim);
		for (unsigned int j = 0; j < dim; j++)
		{
			auto x = (EXP.pc(index, j));
			auto y = (EXP.pc(index + 1, j));
			auto z = (EXP.pc(index + 2, j));
			pc[0][j] = T(x);
			pc[1][j] = T(y);
			pc[2][j] = T(z);
		}
	}

	template <typename T>
	void get_SHAPEMu(T *means, const int _index)
	{
		auto x = SHAPE.mean[_index];
		auto y = SHAPE.mean[_index + 1];
		auto z = SHAPE.mean[_index + 2];
		means[0] = T(x);
		means[1] = T(y);
		means[2] = T(z);
	}

	template <typename T>
	void get_TEXMu(T *means, const int _index)
	{
		auto x = TEX.mean[_index];
		auto y = TEX.mean[_index + 1];
		auto z = TEX.mean[_index + 2];
		means[0] = T(x);
		means[1] = T(y);
		means[2] = T(z);
	}

	template <typename T>
	void get_EXPMu(T *means, const int _index)
	{
		auto x = EXP.mean[_index];
		auto y = EXP.mean[_index + 1];
		auto z = EXP.mean[_index + 2];
		means[0] = T(x);
		means[1] = T(y);
		means[2] = T(z);
	}

	template <typename T>
	void get_SHAPEStd(T & stdard_div, const int _index){
		auto _std = SHAPE.variance[_index];
		_std = std::sqrt(_std);
		stdard_div = T(_std);
	}

	template <typename T>
	void get_EXPStd(T & stdard_div, const int _index){
		auto _std = EXP.variance[_index];
		_std = std::sqrt(_std);
		stdard_div = T(_std);
	}

	template <typename T>
	void get_TEXStd(T & stdard_div, const int _index){
		auto _std = TEX.variance[_index];
		_std = std::sqrt(_std);
		stdard_div = T(_std);
	}

	inline double *GetPointer2array(std::vector<double> _coeffs)
	{
		double *pointer = (double *)malloc(_coeffs.size() * sizeof(double));
		double *id = pointer;
		int counter = 0;

		while (counter < _coeffs.size())
		{
			*id = _coeffs[counter];
			counter++;
			id++;
		}

		return pointer;
	}

	template <typename Derived>
	Matrix<Derived, Dynamic, 1> get_component(const Derived *const aCoef,
											  const VectorXd &vecMu, const MatrixXd &matPc, unsigned int nLength) const
	{
		assert(aCoef != nullptr);
		Matrix<Derived, Dynamic, 1> tmpCoef(nLength);
		for (auto i = 0u; i < nLength; i++)
			tmpCoef(i) = aCoef[i];

		MatrixXd partPc = Eigen::MatrixXd::Zero(global_num_vertices * 3, nLength);

		for (unsigned int i = 0; i < global_num_vertices; i++)
		{
			for (unsigned int j = 0; j < nLength; j++)
			{
				partPc(3 * i, j) = matPc(3 * i, j);
				partPc(3 * i + 1, j) = matPc(3 * i + 1, j);
				partPc(3 * i + 2, j) = matPc(3 * i + 2, j);
			}
		}

		Matrix<Derived, Dynamic, 1> tmpMu = vecMu.cast<Derived>();
		Matrix<Derived, Dynamic, Dynamic> tmpPc = partPc.cast<Derived>();
		// return tmpMu + tmpPc * tmpCoef.cwiseProduct(tmpEv);
		return tmpMu + tmpPc * tmpCoef;
	}

	std::pair<std::vector<Vector3d>, std::vector<Vector3d>> get_tranformedBFMMesh(std::vector<double> &Coef_shape, std::vector<double> &Coef_tex, std::vector<double> &Coef_exp)
	{
		int num_shapePcs = Coef_shape.size();
		int num_texPcs = Coef_tex.size();
		int num_expPcs = Coef_exp.size();

		double *p2shape = GetPointer2array(Coef_shape);
		double *p2tex = GetPointer2array(Coef_tex);
		double *p2exp = GetPointer2array(Coef_exp);

		if (num_shapePcs != global_num_shape_pcs || num_texPcs != global_num_tex_pcs || num_expPcs != global_num_exp_pcs)
		{
			std::cout << "Different number of coefficients are given" << std::endl;
		}

		VectorXd vecCurrentShape = get_component(p2shape, SHAPE.mean, SHAPE.pc, num_shapePcs);
		VectorXd vecCurrentTex = get_component(p2tex, TEX.mean, TEX.pc, num_texPcs);
		VectorXd vecCurrentExp = get_component(p2exp, EXP.mean, EXP.pc, num_expPcs);
		VectorXd vecCurrentBlendshape = vecCurrentShape + vecCurrentExp;

		std::vector<Vector3d> current_vertices_pos;
		std::vector<Vector3d> current_vertices_rgb;
		unsigned int cnt = 0;
		for (unsigned int i = 0; i < num_vertices; i++)
		{

			double x, y, z;
			double r, g, b;
			x = (vecCurrentBlendshape(i * 3));
			y = (vecCurrentBlendshape(i * 3 + 1));
			z = (vecCurrentBlendshape(i * 3 + 2));

			r = (vecCurrentTex(i * 3));
			g = (vecCurrentTex(i * 3 + 1));
			b = (vecCurrentTex(i * 3 + 2));

			Vector3d _current_shape = {x, y, z};
			Vector3d _current_rgb = {r, g, b};

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
	explicit Generic_BFMUpdate(T *const _shape_coefs, T *const _exp_coefs, T *const _tex_coefs) : shape_coefs{_shape_coefs}, exp_coefs{_exp_coefs}, tex_coefs{_tex_coefs}
	{
	}


	void setZero()
	{
		for (int i = 0; i < dim_shape; i++)
		{
			shape_coefs[i] = T(0);
		}

		for (int i = 0; i < dim_exp; i++)
		{
			exp_coefs[i] = T(0);
		}

		for (int i = 0; i < dim_tex; i++)
		{
			tex_coefs[i] = T(0);
		}

	}

	T *getData_shape() const
	{
		return shape_coefs;
	}

	T *getData_exp() const
	{
		return exp_coefs;
	}

	T *getData_tex() const
	{
		return tex_coefs;
	}

private:
	T *shape_coefs;
	T *exp_coefs;
	T *tex_coefs;
	const int dim_shape = global_num_shape_pcs;
	const int dim_exp = global_num_exp_pcs;
	const int dim_tex = global_num_tex_pcs;
	const int num_vertices = global_num_vertices;
};

/**
 * Optimization constraints.
 */
class PointToPointConstraint
{
public:
	PointToPointConstraint(const Vector3d &targetPoint, const double weight, const int _index, Constraint_bfmManager *_constraint_bfmManager, const double lambda) : m_targetPoint{targetPoint}, m_weight{weight}, m_index{_index}, constraint_bfmManager{_constraint_bfmManager}, LAMBDA{lambda}
	{
	}

	template <typename T>
	bool operator()(const T *const _coefs_shape, const T *const _coefs_exp, T *residuals) const
	{
		const int _template_index = m_index;
		const int _index = 3 * m_index;

		T **_shape_matPc;
		_shape_matPc = new T *[3]; // replaced "int" for "T"

		for (int i = 0; i < 3; i++)
		{
			_shape_matPc[i] = new T[dim_shape]; // replaced "int" for "T"
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < dim_shape; j++)
			{
				_shape_matPc[i][j] = T(0); // replaced "int" for "T"
			}
		}

		constraint_bfmManager->get_SHAPEPc<T>(_index, dim_shape, _shape_matPc);

		T _shape_mu[3];
		_shape_mu[0] = T(0);
		_shape_mu[1] = T(0);
		_shape_mu[2] = T(0);

		constraint_bfmManager->get_SHAPEMu<T>(_shape_mu,_index);

		T **_exp_matPc;
		_exp_matPc = new T *[3]; // replaced "int" for "T"

		for (int i = 0; i < 3; i++)
		{
			_exp_matPc[i] = new T[dim_exp]; // replaced "int" for "T"
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < dim_exp; j++)
			{
				_exp_matPc[i][j] = T(0); // replaced "int" for "T"
			}
		}
		constraint_bfmManager->get_EXPPc<T>(_index, dim_exp, _exp_matPc);

		T _exp_mu[3];
		_exp_mu[0] = T(0);
		_exp_mu[1] = T(0);
		_exp_mu[2] = T(0);

		constraint_bfmManager->get_EXPMu<T>(_exp_mu, _index);

		T tmpShapeCoef[dim_shape];
		for (auto i = 0; i < dim_shape; i++)
			tmpShapeCoef[i] = _coefs_shape[i];

		T tmpExpCoef[dim_exp];
		for (auto i = 0; i < dim_exp; i++)
			tmpExpCoef[i] = _coefs_exp[i];

		T noise_shape[3];
		noise_shape[0] = T(0);
		noise_shape[1] = T(0);
		noise_shape[2] = T(0);

		for (int c = 0; c < dim_shape; c++)
		{
			T _out0 = T(_shape_matPc[0][c]) * T(tmpShapeCoef[c]);
			T _out1 = T(_shape_matPc[1][c]) * T(tmpShapeCoef[c]);
			T _out2 = T(_shape_matPc[2][c]) * T(tmpShapeCoef[c]);
			noise_shape[0] += T(_out0);
			noise_shape[1] += T(_out1);
			noise_shape[2] += T(_out2);
		}

		T noise_exp[3];
		noise_exp[0] = T(0);
		noise_exp[1] = T(0);
		noise_exp[2] = T(0);

		for (int c = 0; c < dim_exp; c++)
		{
			T _out0 = T(_exp_matPc[0][c]) * T(tmpExpCoef[c]);
			T _out1 = T(_exp_matPc[1][c]) * T(tmpExpCoef[c]);
			T _out2 = T(_exp_matPc[2][c]) * T(tmpExpCoef[c]);
			noise_exp[0] += T(_out0);
			noise_exp[1] += T(_out1);
			noise_exp[2] += T(_out2);
		}

		double _lam_shape = 1.0;
		double _lam_exp = 1.0;

		T current_shape[3];
		current_shape[0] = T(0);
		current_shape[1] = T(0);
		current_shape[2] = T(0);

		current_shape[0] = T(_shape_mu[0]) + T(noise_shape[0]);
		current_shape[1] = T(_shape_mu[1]) + T(noise_shape[1]);
		current_shape[2] = T(_shape_mu[2]) + T(noise_shape[2]);

		T current_exp[3];
		current_exp[0] = T(0);
		current_exp[1] = T(0);
		current_exp[2] = T(0);

		current_exp[0] = T(_exp_mu[0]) + T(noise_exp[0]);
		current_exp[1] = T(_exp_mu[1]) + T(noise_exp[1]);
		current_exp[2] = T(_exp_mu[2]) + T(noise_exp[2]);

		T blendshape[3];
		blendshape[0] = T(0);
		blendshape[1] = T(0);
		blendshape[2] = T(0);

		blendshape[0] = T(_lam_shape) * current_shape[0] + T(_lam_exp)*current_exp[0];
		blendshape[1] = T(_lam_shape) *current_shape[1] + T(_lam_exp)*current_exp[1];
		blendshape[2] = T(_lam_shape) *current_shape[2] + T(_lam_exp)*current_exp[2];

		T targetPoint[3];
		fillVector(m_targetPoint, targetPoint);

		T subtract_st[3];

		subtract_st[0] = blendshape[0] - targetPoint[0];
		subtract_st[1] = blendshape[1] - targetPoint[1];
		subtract_st[2] = blendshape[2] - targetPoint[2];

		double _weight = sqrt(m_weight);
		double _LAMBDA = sqrt(LAMBDA);
		T norm = T(0);
		get_norm(subtract_st, norm);
		double _minus = -1.0;
		T negative_norm = T(_minus)*norm;
		const T _em_weight = sqrt(exp(negative_norm));
		const T coeff = T(_LAMBDA) * T(_weight);//* T(_em_weight);

		residuals[0] = subtract_st[0] * coeff;
		residuals[1] = subtract_st[1] * coeff;
		residuals[2] = subtract_st[2] * coeff;

		return true;
	}

	static ceres::CostFunction *create(const Vector3d &targetPoint, const double weight, const int index, Constraint_bfmManager *_constraint_bfmManager, const double lambda)
	{
		return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, global_num_shape_pcs, global_num_exp_pcs>(
			new PointToPointConstraint(targetPoint, weight, index, _constraint_bfmManager, lambda));
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	const Vector3d m_targetPoint;
	const double m_weight;
	const int m_index;
	const int num_vertices = global_num_vertices;
	const double LAMBDA;
	const int dim_shape = global_num_shape_pcs;
	const int dim_tex = global_num_tex_pcs;
	const int dim_exp = global_num_exp_pcs;
};

class PointToPointConstraint_shape
{
public:
	PointToPointConstraint_shape(const Vector3d &targetPoint, const double weight, const int _index, Constraint_bfmManager *_constraint_bfmManager, const double lambda) : m_targetPoint{targetPoint}, m_weight{weight}, m_index{_index}, constraint_bfmManager{_constraint_bfmManager}, LAMBDA{lambda}
	{
	}

	template <typename T>
	bool operator()(const T *const _coefs_shape, T *residuals) const
	{
		const int _template_index = m_index;
		const int _index = 3 * m_index;


		T **_shape_matPc;
		_shape_matPc = new T *[3]; // replaced "int" for "T"

		for (int i = 0; i < 3; i++)
		{
			_shape_matPc[i] = new T[dim_shape]; // replaced "int" for "T"
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < dim_shape; j++)
			{
				_shape_matPc[i][j] = T(0); // replaced "int" for "T"
			}
		}

		constraint_bfmManager->get_SHAPEPc<T>(_index, dim_shape, _shape_matPc);

		T _shape_mu[3];
		_shape_mu[0] = T(0);
		_shape_mu[1] = T(0);
		_shape_mu[2] = T(0);

		constraint_bfmManager->get_SHAPEMu<T>(_shape_mu,_index);

		T _exp_mu[3];
		_exp_mu[0] = T(0);
		_exp_mu[1] = T(0);
		_exp_mu[2] = T(0);

		constraint_bfmManager->get_EXPMu<T>(_exp_mu, _index);

		T tmpShapeCoef[dim_shape];
		for (auto i = 0; i < dim_shape; i++)
			tmpShapeCoef[i] = _coefs_shape[i];

		T noise_shape[3];
		noise_shape[0] = T(0);
		noise_shape[1] = T(0);
		noise_shape[2] = T(0);

		for (int c = 0; c < dim_shape; c++)
		{
			T _out0 = T(_shape_matPc[0][c]) * T(tmpShapeCoef[c]);
			T _out1 = T(_shape_matPc[1][c]) * T(tmpShapeCoef[c]);
			T _out2 = T(_shape_matPc[2][c]) * T(tmpShapeCoef[c]);
			noise_shape[0] += T(_out0);
			noise_shape[1] += T(_out1);
			noise_shape[2] += T(_out2);
		}

		double _lam_shape = 1.0;
		double _lam_exp = 1.0;

		T current_shape[3];
		current_shape[0] = T(0);
		current_shape[1] = T(0);
		current_shape[2] = T(0);

		current_shape[0] = T(_shape_mu[0]) + T(_lam_shape) * T(noise_shape[0]);
		current_shape[1] = T(_shape_mu[1]) + T(_lam_shape) * T(noise_shape[1]);
		current_shape[2] = T(_shape_mu[2]) + T(_lam_shape) * T(noise_shape[2]);


		T blendshape[3];
		blendshape[0] = T(0);
		blendshape[1] = T(0);
		blendshape[2] = T(0);

		blendshape[0] = current_shape[0] + T(_exp_mu[0]);
		blendshape[1] = current_shape[1] + T(_exp_mu[1]);
		blendshape[2] = current_shape[2] + T(_exp_mu[2]);

		T targetPoint[3];
		fillVector(m_targetPoint, targetPoint);

		T subtract_st[3];

		subtract_st[0] = blendshape[0] - targetPoint[0];
		subtract_st[1] = blendshape[1] - targetPoint[1];
		subtract_st[2] = blendshape[2] - targetPoint[2];

		double _weight = sqrt(m_weight);
		double _LAMBDA = sqrt(LAMBDA);
		T norm = T(0);
		get_norm(subtract_st, norm);
		double _minus = -1.0;
		T negative_norm = T(_minus)*norm;
		const T _em_weight = sqrt(exp(negative_norm));
		const T coeff = T(_LAMBDA) * T(_weight);//* T(_em_weight);

		residuals[0] = subtract_st[0] * coeff;
		residuals[1] = subtract_st[1] * coeff;
		residuals[2] = subtract_st[2] * coeff;

		return true;
	}

	static ceres::CostFunction *create(const Vector3d &targetPoint, const double weight, const int index, Constraint_bfmManager *_constraint_bfmManager, const double lambda)
	{
		return new ceres::AutoDiffCostFunction<PointToPointConstraint_shape, 3, global_num_shape_pcs>(
			new PointToPointConstraint_shape(targetPoint, weight, index, _constraint_bfmManager, lambda));
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	const Vector3d m_targetPoint;
	const double m_weight;
	const int m_index;
	const int num_vertices = global_num_vertices;
	const double LAMBDA;
	const int dim_shape = global_num_shape_pcs;
	const int dim_tex = global_num_tex_pcs;
	const int dim_exp = global_num_exp_pcs;
};


class PointToPointConstraint_exp
{
public:
	PointToPointConstraint_exp(const Vector3d &targetPoint, const double weight, const int _index, Constraint_bfmManager *_constraint_bfmManager, const double lambda, std::vector<double> & _coefs_shape) : m_targetPoint{targetPoint}, m_weight{weight}, m_index{_index}, constraint_bfmManager{_constraint_bfmManager}, LAMBDA{lambda}, coefs_shape{_coefs_shape}
	{
	}

	template <typename T>
	bool operator()(const T *const _coefs_exp, T *residuals) const
	{
		const int _template_index = m_index;
		const int _index = 3 * m_index;

		T **_shape_matPc;
		_shape_matPc = new T *[3]; // replaced "int" for "T"

		for (int i = 0; i < 3; i++)
		{
			_shape_matPc[i] = new T[dim_shape]; // replaced "int" for "T"
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < dim_shape; j++)
			{
				_shape_matPc[i][j] = T(0); // replaced "int" for "T"
			}
		}

		constraint_bfmManager->get_SHAPEPc<T>(_index, dim_shape, _shape_matPc);

		T _shape_mu[3];
		_shape_mu[0] = T(0);
		_shape_mu[1] = T(0);
		_shape_mu[2] = T(0);

		constraint_bfmManager->get_SHAPEMu<T>(_shape_mu,_index);

		T **_exp_matPc;
		_exp_matPc = new T *[3]; // replaced "int" for "T"

		for (int i = 0; i < 3; i++)
		{
			_exp_matPc[i] = new T[dim_exp]; // replaced "int" for "T"
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < dim_exp; j++)
			{
				_exp_matPc[i][j] = T(0); // replaced "int" for "T"
			}
		}
		constraint_bfmManager->get_EXPPc<T>(_index, dim_exp, _exp_matPc);

		T _exp_mu[3];
		_exp_mu[0] = T(0);
		_exp_mu[1] = T(0);
		_exp_mu[2] = T(0);

		constraint_bfmManager->get_EXPMu<T>(_exp_mu, _index);

		T tmpShapeCoef[dim_shape];
		constraint_bfmManager->set_coefs<T>(coefs_shape, tmpShapeCoef);

		T tmpExpCoef[dim_exp];
		for (auto i = 0; i < dim_exp; i++)
			tmpExpCoef[i] = _coefs_exp[i];

		T noise_shape[3];
		noise_shape[0] = T(0);
		noise_shape[1] = T(0);
		noise_shape[2] = T(0);

		for (int c = 0; c < dim_shape; c++)
		{
			T _out0 = T(_shape_matPc[0][c]) * T(tmpShapeCoef[c]);
			T _out1 = T(_shape_matPc[1][c]) * T(tmpShapeCoef[c]);
			T _out2 = T(_shape_matPc[2][c]) * T(tmpShapeCoef[c]);
			noise_shape[0] += T(_out0);
			noise_shape[1] += T(_out1);
			noise_shape[2] += T(_out2);
		}

		T noise_exp[3];
		noise_exp[0] = T(0);
		noise_exp[1] = T(0);
		noise_exp[2] = T(0);

		for (int c = 0; c < dim_exp; c++)
		{
			T _out0 = T(_exp_matPc[0][c]) * T(tmpExpCoef[c]);
			T _out1 = T(_exp_matPc[1][c]) * T(tmpExpCoef[c]);
			T _out2 = T(_exp_matPc[2][c]) * T(tmpExpCoef[c]);
			noise_exp[0] += T(_out0);
			noise_exp[1] += T(_out1);
			noise_exp[2] += T(_out2);
		}

		double _lam_shape = 1.0;
		double _lam_exp = 1.0;

		T current_shape[3];
		current_shape[0] = T(0);
		current_shape[1] = T(0);
		current_shape[2] = T(0);

		current_shape[0] = T(_shape_mu[0]) + T(noise_shape[0]);
		current_shape[1] = T(_shape_mu[1]) + T(noise_shape[1]);
		current_shape[2] = T(_shape_mu[2]) + T(noise_shape[2]);

		T current_exp[3];
		current_exp[0] = T(0);
		current_exp[1] = T(0);
		current_exp[2] = T(0);

		current_exp[0] = T(_exp_mu[0]) + T(noise_exp[0]);
		current_exp[1] = T(_exp_mu[1]) + T(noise_exp[1]);
		current_exp[2] = T(_exp_mu[2]) + T(noise_exp[2]);

		T blendshape[3];
		blendshape[0] = T(0);
		blendshape[1] = T(0);
		blendshape[2] = T(0);

		blendshape[0] = T(_lam_shape) * current_shape[0] + T(_lam_exp)*current_exp[0];
		blendshape[1] = T(_lam_shape) *current_shape[1] + T(_lam_exp)*current_exp[1];
		blendshape[2] = T(_lam_shape) *current_shape[2] + T(_lam_exp)*current_exp[2];

		T targetPoint[3];
		fillVector(m_targetPoint, targetPoint);

		T subtract_st[3];

		subtract_st[0] = blendshape[0] - targetPoint[0];
		subtract_st[1] = blendshape[1] - targetPoint[1];
		subtract_st[2] = blendshape[2] - targetPoint[2];

		double _weight = sqrt(m_weight);
		double _LAMBDA = sqrt(LAMBDA);
		T norm = T(0);
		get_norm(subtract_st, norm);
		double _minus = -1.0;
		T negative_norm = T(_minus)*norm;
		const T _em_weight = sqrt(exp(negative_norm));
		const T coeff = T(_LAMBDA) * T(_weight); // * T(_em_weight);

		residuals[0] = subtract_st[0] * coeff;
		residuals[1] = subtract_st[1] * coeff;
		residuals[2] = subtract_st[2] * coeff;

		return true;
	}

	static ceres::CostFunction *create(const Vector3d &targetPoint, const double weight, const int index, Constraint_bfmManager *_constraint_bfmManager, const double lambda, std::vector<double> & _coefs_shape)
	{
		return new ceres::AutoDiffCostFunction<PointToPointConstraint_exp, 3, global_num_exp_pcs>(
			new PointToPointConstraint_exp(targetPoint, weight, index, _constraint_bfmManager, lambda, _coefs_shape));
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	std::vector<double> coefs_shape;
	const Vector3d m_targetPoint;
	const double m_weight;
	const int m_index;
	const int num_vertices = global_num_vertices;
	const double LAMBDA;
	const int dim_shape = global_num_shape_pcs;
	const int dim_tex = global_num_tex_pcs;
	const int dim_exp = global_num_exp_pcs;
};

/*
* Optimization constraints
* //source looking up a point color from BFM face mesh given parameters
  //target depth map point_color (which is stored in FacePointCloud)
*/
class ColorContraint
{
public:
	ColorContraint(const Vector3d &targetColor, const double weight, const int _index, Constraint_bfmManager *_constraint_bfmManager) : m_targetColor{targetColor}, m_weight{weight}, m_index{_index}, constraint_bfmManager{_constraint_bfmManager}
	{
	}

	template <typename T>
	bool operator()(const T *const _coefs_tex, T *residuals) const
	{
		const int _template_index = m_index;
		const int _index = 3 * m_index;

		T **_tex_matPc;
		_tex_matPc = new T *[3]; // replaced "int" for "T"

		for (int i = 0; i < 3; i++)
		{
			_tex_matPc[i] = new T[dim_tex]; // replaced "int" for "T"
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < dim_tex; j++)
			{
				_tex_matPc[i][j] = T(0); // replaced "int" for "T"
			}
		}

		constraint_bfmManager->get_TEXPc<T>(_index, dim_tex, _tex_matPc);

		T _tex_mu[3];
		_tex_mu[0] = T(0);
		_tex_mu[1] = T(0);
		_tex_mu[2] = T(0);

		constraint_bfmManager->get_TEXMu<T>(_tex_mu, _index);

		double _LAMBDA = std::sqrt(LAMBDA);
		T _lambda = T(_LAMBDA);
		T tmpTexCoef[dim_tex];
		for (auto i = 0; i < dim_tex; i++){
			double _sign = -1;
			T sign = T(_sign);
			tmpTexCoef[i] = _coefs_tex[i];
			tmpTexCoef[i] = tmpTexCoef[i]*_lambda;
		}

		T noise_tex[3];
		noise_tex[0] = T(0);
		noise_tex[1] = T(0);
		noise_tex[2] = T(0);

		for (int c = 0; c < dim_tex; c++)
		{
			T _out0 = T(_tex_matPc[0][c]) * T(tmpTexCoef[c]);
			T _out1 = T(_tex_matPc[1][c]) * T(tmpTexCoef[c]);
			T _out2 = T(_tex_matPc[2][c]) * T(tmpTexCoef[c]);
			noise_tex[0] += T(_out0);
			noise_tex[1] += T(_out1);
			noise_tex[2] += T(_out2);
		}

		T current_tex[3];
		current_tex[0] = T(0);
		current_tex[1] = T(0);
		current_tex[2] = T(0);

		T _scaler = T(255);
		current_tex[0] = T(_tex_mu[0]) + T(noise_tex[0]);
		current_tex[1] = T(_tex_mu[1]) + T(noise_tex[1]);
		current_tex[2] = T(_tex_mu[2]) + T(noise_tex[2]);

		current_tex[0]*=_scaler;
		current_tex[1]*=_scaler;
		current_tex[2]*=_scaler;


		T targetColor[3];
		fillVector(m_targetColor, targetColor);


		targetColor[0]*=_scaler;
		targetColor[1]*=_scaler;
		targetColor[2]*=_scaler;

		T subtract_st[3];

		subtract_st[0] = current_tex[0] - targetColor[0];
		subtract_st[1] = current_tex[1] - targetColor[1];
		subtract_st[2] = current_tex[2] - targetColor[2];


		double _weight = sqrt(m_weight);
		T norm = T(0);
		get_norm(subtract_st, norm);
		double _minus = -1.0;
		T negative_norm = T(_minus)*norm;
		const T _em_weight = sqrt(exp(negative_norm));
		const T coeff = T(_weight) * T(LAMBDA);

		// Three residual
		residuals[0] = subtract_st[0] * coeff;
		residuals[1] = subtract_st[1] * coeff;
		residuals[2] = subtract_st[2] * coeff;

		return true;
	}

	static ceres::CostFunction *create(const Vector3d &targetColor, const double weight, const int index, Constraint_bfmManager *_constraint_bfmManager)
	{
		return new ceres::AutoDiffCostFunction<ColorContraint, 3, global_num_tex_pcs>(
			new ColorContraint(targetColor, weight, index, _constraint_bfmManager));
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	const Vector3d m_targetColor;
	const double m_weight;
	const int m_index;
	const int num_vertices = global_num_vertices;
	const double LAMBDA = global_color_lambda;
	const int dim_shape = global_num_shape_pcs;
	const int dim_tex = global_num_tex_pcs;
	const int dim_exp = global_num_exp_pcs;
};


/**
 * Optimization constraints.
 */

class Regularizer_shape
{
public:
	Regularizer_shape(Constraint_bfmManager *_constraint_bfmManager) : constraint_bfmManager{_constraint_bfmManager}
	{
	}

	template <typename T>
	bool operator()(const T * const coef, T *residuals) const
	{

		for(int i = 0; i < dim; i++){
			T stdard_div;
			constraint_bfmManager->get_SHAPEStd(stdard_div, i);
			double lambda = std::sqrt(LAMBDA);
			residuals[i] = (T(lambda)*(coef[i]))/(stdard_div);
		}

		return true;
	}

	static ceres::CostFunction *create(Constraint_bfmManager *_constraint_bfmManager)
	{
		return new ceres::AutoDiffCostFunction<Regularizer_shape, global_num_shape_pcs, global_num_shape_pcs>(
			new Regularizer_shape(_constraint_bfmManager));
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	const double LAMBDA = global_regSHAPE_lambda;
	const int dim = global_num_shape_pcs;
};

class Regularizer_exp
{
public:
	Regularizer_exp(Constraint_bfmManager *_constraint_bfmManager) : constraint_bfmManager{_constraint_bfmManager}
	{
	}

	template <typename T>
	bool operator()(const T * const coef, T *residuals) const
	{
		for(int i = 0; i < dim; i++){
			T stdard_div;
			constraint_bfmManager->get_EXPStd(stdard_div, i);
			double lambda = std::sqrt(LAMBDA);
			residuals[i] = (T(lambda)*coef[i])/(stdard_div);
		}

		return true;
	}

	static ceres::CostFunction *create(Constraint_bfmManager *_constraint_bfmManager)
	{
		return new ceres::AutoDiffCostFunction<Regularizer_exp, global_num_exp_pcs, global_num_exp_pcs>(
			new Regularizer_exp(_constraint_bfmManager));
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	const double LAMBDA = global_regEXP_lambda;
	const int dim = global_num_exp_pcs;
};

class Regularizer_color
{
public:
	Regularizer_color(Constraint_bfmManager *_constraint_bfmManager) : constraint_bfmManager{_constraint_bfmManager}
	{
	}

	template <typename T>
	bool operator()(const T * const coef, T *residuals) const
	{
		for(int i = 0; i < dim; i++){
			T stdard_div;
			constraint_bfmManager->get_TEXStd(stdard_div, i);
			double lambda = std::sqrt(LAMBDA);
			residuals[i] = (T(lambda)*coef[i]);
		}
	}

	static ceres::CostFunction *create(Constraint_bfmManager *_constraint_bfmManager)
	{
		return new ceres::AutoDiffCostFunction<Regularizer_color, global_num_tex_pcs, global_num_tex_pcs>(
			new Regularizer_color(_constraint_bfmManager));
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	const double LAMBDA = global_regTEX_lambda;
	const int dim = global_num_tex_pcs;
};

/**
 * ICP optimizer - Abstract Base Class, using Ceres for optimization.
 */
class ICPOptimizer
{
public:
	ICPOptimizer() : m_bUsePointToPlaneConstraints{false},
					 m_nIterations{20},
					 m_nearestNeighborSearch_forSparse{std::make_unique<NearestNeighborSearchBruteForce>()},
					 m_nearestNeighborSearch_forDense{std::make_unique<NearestNeighborSearchFlann>()},
					 m_nearestNeighborSearch_forColor{std::make_unique<NearestNeighborSearchFlann>()}
	{
	}

	void setMatchingMaxDistance_sparse(double maxDistance)
	{
		m_nearestNeighborSearch_forSparse->setMatchingMaxDistance(maxDistance);
	}

	void setMatchingMaxDistance_dense(double maxDistance)
	{
		m_nearestNeighborSearch_forDense->setMatchingMaxDistance(maxDistance);
	}

	void setMatchingMaxDistance_color(double maxDistance)
	{
		m_nearestNeighborSearch_forColor->setMatchingMaxDistance(maxDistance);
	}

	void usePointToPlaneConstraints(bool bUsePointToPlaneConstraints)
	{
		m_bUsePointToPlaneConstraints = bUsePointToPlaneConstraints;
	}

	void setNbOfIterations(unsigned nIterations)
	{
		m_nIterations = nIterations;
	}

	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams_shape_tex(const FacePointCloud &target_landmarks, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM &bfm, std::vector<int> &bfm_landmarkIndex_list, std::vector<Vector3i> BFM_triangle_list){};
	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams_exp_tex(const FacePointCloud &target_landmarks, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM &bfm, std::vector<int> &bfm_landmarkIndex_list, std::vector<Vector3i> BFM_triangle_list){};
	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams_colors(const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM &bfm, std::vector<Vector3i> BFM_triangle_list){};

protected:
	bool m_bUsePointToPlaneConstraints;
	unsigned m_nIterations;
	std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch_forSparse;
	std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch_forDense;
	std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch_forColor;

	void pruneCorrespondences(const std::vector<Vector3d> &sourceNormals, const std::vector<Vector3d> &targetNormals, std::vector<Match> &matches_sparse)
	{
		const unsigned nPoints = sourceNormals.size();

		int counter_invalid_point = 0;
		int counter_valid_point = 0;
		int kernel_size = 10;

		bool landmark_flag = false;
		if (nPoints <= 67){
			landmark_flag = true;
		}

		for (unsigned i = 0; i < nPoints; i++)
		{
			bool are_MINF = true;
			Match &match = matches_sparse[i];
			if (match.idx >= 0)
			{
				const auto &sourceNormal = sourceNormals[i];
				const auto &targetNormal = targetNormals[match.idx];

				double angle = acosf64x(sourceNormal.dot(targetNormal));
				if (angle > (double)(60.0))
				{
					match.idx = -1;
				}

				if (!landmark_flag && (counter_valid_point % kernel_size != 0)){
					match.idx = -1;
				}
				counter_valid_point++;
			}
			else
			{
				counter_invalid_point++;
			}
		}

		std::cout << "invalid point: " << counter_invalid_point << std::endl;
		std::cout << "valid point: " << counter_valid_point << std::endl;
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
	CeresICPOptimizer(bool BRUTE_FORCE) {
		if(BRUTE_FORCE){
			m_nearestNeighborSearch_forSparse = nullptr;
			m_nearestNeighborSearch_forSparse = std::make_unique<NearestNeighborSearchBruteForce>();
			m_nearestNeighborSearch_forDense = nullptr;
			m_nearestNeighborSearch_forDense = std::make_unique<NearestNeighborSearchBruteForce>();
			m_nearestNeighborSearch_forColor = nullptr;
			m_nearestNeighborSearch_forColor = std::make_unique<NearestNeighborSearchBruteForce>();

		}
	}
	// shape tex, exp
	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams_shape_tex(const FacePointCloud &target_landmarks, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM &bfm, std::vector<int> &bfm_landmarkIndex_list, std::vector<Vector3i> BFM_triangle_list) override
	{
		/*source_landmarks
		- bfm landmarks vertex position
		- bfm landmarks vertex normal which is obtained by delaunay triangulation
		*/

		/*target_landmarks
		- dlib detected landmarks position which are back projected in kinect camera space
		- dlib detected landmarks vertex normal which is obtained by delaunay triangulation
		*/

		// Build the index of the FLANN tree (for fast nearest neighbor lookup).

		// Landmarks for sparse term
		int num_vertices = global_num_vertices;
		std::vector<Vector3f> _float_targetLandmarks_points = target_landmarks.getPoints();
		std::cout << "Target_points: " << _float_targetLandmarks_points.size() << std::endl;
		std::vector<Vector3f> _float_targetLandmarks_normals = target_landmarks.getNormals();
		std::cout << "Target_normals: " << _float_targetLandmarks_normals.size() << std::endl;
		std::vector<Vector3d> targetLandmarks_points = convert_float2double(_float_targetLandmarks_points);
		std::vector<Vector3d> targetLandmarks_normals = convert_float2double(_float_targetLandmarks_normals);
		std::vector<Vector3i> targetLandmarks_triangleLists = target_landmarks.getTriangleLists();
		std::vector<Vector3i> sourceLandmarks_triangleLists = targetLandmarks_triangleLists;


		// For dense term
		std::vector<Vector3f> _float_target_points = target.getPoints();
		std::vector<Vector3f> _float_target_normals = target.getNormals();
		std::vector<Vector3uc> _uchar_target_colors = target.getColors();
		std::vector<Vector3f> _coarse_float_target_points;
		std::vector<Vector3d> target_points;
		std::vector<Vector3d> target_normals;
		std::vector<Vector3d> target_colors;

		// reduction of target points and normals with giving kernel size
		std::cout<<"num of vertices of RGBD scan: "<<_float_target_points.size()<<std::endl;
		int kernel_size = 3;
		for(int i = 0; i < _float_target_points.size(); i=i+kernel_size){
			Vector3f _temp_f = _float_target_points[i].cast<float>();
			Vector3d _temp_d = _float_target_points[i].cast<double>();
			_coarse_float_target_points.push_back(_temp_f);
			target_points.push_back(_temp_d);
			Vector3d _temp_normal = _float_target_normals[i].cast<double>();
			target_normals.push_back(_temp_normal);
			double r,g,b;
			r = int(_uchar_target_colors[i].x())/255;
			g =	int(_uchar_target_colors[i].y())/255;
			b = int(_uchar_target_colors[i].z())/255;
			Vector3d _temp_color = {r, g, b};
			target_colors.push_back(_temp_color);
		}

		// Build the index of the FLANN tree (for fast nearest neighbor lookup).
		// float
		m_nearestNeighborSearch_forSparse->buildIndex(target_landmarks.getPoints());
		m_nearestNeighborSearch_forDense->buildIndex(_coarse_float_target_points);


		// copy initial coefficients
		// The initial estimate can be given as an argument.
		std::vector<double> estimated_coefs_shape;
		std::copy(_initial_coef_shape.begin(), _initial_coef_shape.end(), std::back_inserter(estimated_coefs_shape));
		std::vector<double> estimated_coefs_tex;
		std::copy(_initial_coef_tex.begin(), _initial_coef_tex.end(), std::back_inserter(estimated_coefs_tex));
		std::vector<double> estimated_coefs_exp;
		std::copy(_initial_coef_exp.begin(), _initial_coef_exp.end(), std::back_inserter(estimated_coefs_exp));

		// set constraint_bfmManager
		Constraint_bfmManager constraint_bfmManager(SHAPE, TEX, EXP, BFM_triangle_list);

		int _int_num_shapePc = global_num_shape_pcs;
		int _int_num_texPc = global_num_tex_pcs;
		int _int_num_expPc = global_num_exp_pcs;
		int _int_num_vertices = global_num_vertices;

		// we optimize parameter for shape, tex, and expression
		// the dimensionalities can be defined by the top as global arguments
		double _increment_coef_shape[_int_num_shapePc];
		double _increment_coef_tex[_int_num_texPc];
		double _increment_coef_exp[_int_num_expPc];

		// class to update the bfm mesh
		auto bfmMeshUpdate = Generic_BFMUpdate<double>(_increment_coef_shape, _increment_coef_exp, _increment_coef_tex);
		bfmMeshUpdate.setZero();
		// iterative optimization
		for (int i = 0; i < m_nIterations; ++i)
		{
			// Compute the matches_sparse.
			std::cout << "Matching points ..." << std::endl;
			clock_t begin = clock();

			// Get transformed bfm landmarks position and normals to create FacePointCloud
			std::vector<Vector3d> updated_BFM_vertex_pos;
			std::vector<Vector3d> updated_BFM_vertex_rgb;
			std::tie(updated_BFM_vertex_pos, updated_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);

			std::vector<Vector3f> float_updated_BFM_vertex_pos = convert_double2float(updated_BFM_vertex_pos);
			std::vector<Vector3f> float_updated_BFM_vertex_rgb = convert_double2float(updated_BFM_vertex_rgb);

			// This face point cloud is containing entire bfm mesh vertices
			FacePointCloud transform_sourceMesh{float_updated_BFM_vertex_pos, BFM_triangle_list};

			// Get landmarks points to create transformed_sourceLandmarksMesh
			std::vector<Vector3f> postPE_bfm_landmarks_points = transform_sourceMesh.get_selectedVertexPos(bfm_landmarkIndex_list);
			// Set transformed landmarks
			FacePointCloud transformed_sourceLandmarksMesh{postPE_bfm_landmarks_points, sourceLandmarks_triangleLists};

			// get vector of landamarks position in double and their normals
			std::vector<Vector3d> d_transformed_sourceLandmarks_pos = convert_float2double(transformed_sourceLandmarksMesh.getPoints());
			std::vector<Vector3d> d_transformed_sourceLandmarks_normals = convert_float2double(transformed_sourceLandmarksMesh.getNormals());

			// get vector of all vertices position in double and their normals
			std::vector<Vector3d> d_transformed_source_pos = convert_float2double(transform_sourceMesh.getPoints());
			std::vector<Vector3d> d_transformed_source_normals = convert_float2double(transform_sourceMesh.getNormals());

			// matches_sparse contains the correspondences
			// float
			auto matches_sparse = m_nearestNeighborSearch_forSparse->queryMatches(transformed_sourceLandmarksMesh.getPoints());
			auto matches_dense = m_nearestNeighborSearch_forDense->queryMatches(transform_sourceMesh.getPoints());

			std::cout<<"Average distance [mm] of sparse: "<<sqrt(m_nearestNeighborSearch_forSparse->get_aveDist())<<std::endl;
			std::cout<<"Average distance [mm] of dense: "<<sqrt(m_nearestNeighborSearch_forDense->get_aveDist())<<std::endl;


			if (i == 0){
				//Goal: apply the distance to the each vertex color attribute
				/*
				* Large error: red
				* Small error: blue
				* 	Error is normalized [0, 1] by dividing the current error by maximum
				*   Map the normalized error on to red[0, 255], and blue [0,255]
				*   P = 0.01 (Small error)
				*   Red = int(255*(P))
				* 	Blue = int(255*(1-P))
				*
				*/
				std::cout<<"=================="<<"INIT EVALUTAION"<<"===================="<<std::endl;

				//get max distance, min distance
				float distanceMax = m_nearestNeighborSearch_forDense->get_measuredMaxDist();
				float distanceMin = m_nearestNeighborSearch_forDense->get_measuredMinDist();

				//get the distance of each vertex from their neighbor
				std::vector<float> measuredDists = m_nearestNeighborSearch_forDense->get_measuredDists();

				std::cout<<"MAX distance: "<<distanceMax<<std::endl;
				std::cout<<"MIN distance: "<<distanceMin<<std::endl;

				std::cout<<"Length of the measureDists: "<<measuredDists.size()<<std::endl;

				std::string f_name_before = "../output/before_paramEst.ply";
				Generic_writeFaceMeshPly(f_name_before, float_updated_BFM_vertex_pos, float_updated_BFM_vertex_rgb, BFM_triangle_list);

				// get color attributes according to the error
				std::vector<Vector3f> error_ColorMap_before = get_GeoErrorPointColors(measuredDists, distanceMax, distanceMin);
				std::string f_name_before_errorMap = "../output/before_paramEst_errorMap.ply";
				Generic_writeFaceMeshPly(f_name_before_errorMap,float_updated_BFM_vertex_pos, error_ColorMap_before, BFM_triangle_list);

			}

			pruneCorrespondences(d_transformed_sourceLandmarks_normals, targetLandmarks_normals, matches_sparse);
			pruneCorrespondences(d_transformed_source_normals, target_normals, matches_dense);

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			// Prepare point-to-point constraints.
			ceres::Problem problem;

			// Add sparse cost functions w.r.t shape and exp parameter
			const int num_Landmark_vertex = d_transformed_sourceLandmarks_pos.size();
			prepareConstraints_Sparse(num_Landmark_vertex, targetLandmarks_points, targetLandmarks_normals, matches_sparse, bfmMeshUpdate, problem, &constraint_bfmManager, bfm_landmarkIndex_list);

			// Add dense const functions w.r.t shape, exp parameter
			const int num_BFMmesh_vertex = d_transformed_source_pos.size();
			prepareConstraints_Dense(num_BFMmesh_vertex, target_points, target_normals, target_colors, matches_dense, bfmMeshUpdate, problem, &constraint_bfmManager);

			// const int num_BFMmesh_vertex = d_transformed_source_pos.size();
			prepareConstraints_Color(num_BFMmesh_vertex, target_colors, matches_dense, bfmMeshUpdate, problem, &constraint_bfmManager);

			// Configure options for the solver.
			std::cout << "Start to solve" << std::endl;
			ceres::Solver::Options options;
			configureSolver(options);

			// Run the solver (for one iteration).
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			std::cout << summary.FullReport() << std::endl;

			double *delta_coefs_shape = bfmMeshUpdate.getData_shape();
			double *delta_coefs_tex = bfmMeshUpdate.getData_tex();
			double *delta_coefs_exp = bfmMeshUpdate.getData_exp();

			for (unsigned int i = 0; i < _int_num_shapePc; i++)
			{
				estimated_coefs_shape[i] =+ delta_coefs_shape[i];
			}

			for (unsigned int i = 0; i < _int_num_texPc; i++)
			{
				estimated_coefs_tex[i] =+ delta_coefs_tex[i];
			}

			bfmMeshUpdate.setZero();
			std::cout << "Optimization iteration done." << std::endl;
		}

		std::vector<Vector3d> final_BFM_vertex_pos;
		std::vector<Vector3d> final_BFM_vertex_rgb;
		std::tie(final_BFM_vertex_pos, final_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);
		std::vector<Vector3f> float_final_BFM_vertex_pos = convert_double2float(final_BFM_vertex_pos);
		std::vector<Vector3f> float_final_BFM_vertex_rgb = convert_double2float(final_BFM_vertex_rgb);

		std::cout<<"=================="<<"FINAL EVALUTAION"<<"===================="<<std::endl;
		//Evaluation
		FacePointCloud final_sourceMesh{float_final_BFM_vertex_pos, BFM_triangle_list};
		auto matches_Eval = m_nearestNeighborSearch_forDense->queryMatches(final_sourceMesh.getPoints());

		std::cout<<"Average distance [mm] of dense evaluation: "<<sqrt(m_nearestNeighborSearch_forDense->get_aveDist())<<std::endl;
		float final_MaxDist = m_nearestNeighborSearch_forDense->get_measuredMaxDist();
		float final_MinDist = m_nearestNeighborSearch_forDense->get_measuredMinDist();

		//get the distance of each vertex from their neighbor
		std::vector<float> measuredDists_final = m_nearestNeighborSearch_forDense->get_measuredDists();

		std::cout<<"MAX distance: "<<final_MaxDist<<std::endl;
		std::cout<<"MIN distance: "<<final_MinDist<<std::endl;
		std::cout<<"Length of the measureDists: "<<measuredDists_final.size()<<std::endl;

		// get color attributes according to the error
		std::vector<Vector3f> error_ColorMap_final = get_GeoErrorPointColors(measuredDists_final, final_MaxDist, final_MinDist);
		std::string f_name_errorMap = "../output/after_paramEst_errorMap.ply";
		Generic_writeFaceMeshPly(f_name_errorMap, float_final_BFM_vertex_pos, error_ColorMap_final, BFM_triangle_list);


		//get exp coeffs
		std::cout<<"EXP coeffs"<<std::endl;
		for(auto & item : estimated_coefs_exp){
			std::cout<<item<<",";
		}
		std::cout<<std::endl;

		std::string f_name1 = "../output/after_paramEst.ply";
		Generic_writeFaceMeshPly(f_name1,float_final_BFM_vertex_pos, float_final_BFM_vertex_rgb, BFM_triangle_list);
		std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> _result_coeffs;
		_result_coeffs = std::make_tuple(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);
		return _result_coeffs;
	}

	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams_exp_tex(const FacePointCloud &target_landmarks, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM &bfm, std::vector<int> &bfm_landmarkIndex_list, std::vector<Vector3i> BFM_triangle_list) override{
		/*source_landmarks
		- bfm landmarks vertex position
		- bfm landmarks vertex normal which is obtained by delaunay triangulation
		*/

		/*target_landmarks
		- dlib detected landmarks position which are back projected in kinect camera space
		- dlib detected landmarks vertex normal which is obtained by delaunay triangulation
		*/

		// Build the index of the FLANN tree (for fast nearest neighbor lookup).

		// Landmarks for sparse term
		int num_vertices = global_num_vertices;
		std::vector<Vector3f> _float_targetLandmarks_points = target_landmarks.getPoints();
		std::cout << "Target_points: " << _float_targetLandmarks_points.size() << std::endl;
		std::vector<Vector3f> _float_targetLandmarks_normals = target_landmarks.getNormals();
		std::cout << "Target_normals: " << _float_targetLandmarks_normals.size() << std::endl;
		std::vector<Vector3d> targetLandmarks_points = convert_float2double(_float_targetLandmarks_points);
		std::vector<Vector3d> targetLandmarks_normals = convert_float2double(_float_targetLandmarks_normals);
		std::vector<Vector3i> targetLandmarks_triangleLists = target_landmarks.getTriangleLists();
		std::vector<Vector3i> sourceLandmarks_triangleLists = targetLandmarks_triangleLists;


		// For dense term
		std::vector<Vector3f> _float_target_points = target.getPoints();
		std::vector<Vector3f> _float_target_normals = target.getNormals();
		std::vector<Vector3uc> _uchar_target_colors = target.getColors();
		std::vector<Vector3f> _coarse_float_target_points;
		std::vector<Vector3d> target_points;
		std::vector<Vector3d> target_normals;
		std::vector<Vector3d> target_colors;

		// reduction of target points and normals with giving kernel size
		std::cout<<"num of vertices of RGBD scan: "<<_float_target_points.size()<<std::endl;
		int kernel_size = 3;
		for(int i = 0; i < _float_target_points.size(); i=i+kernel_size){
			Vector3f _temp_f = _float_target_points[i].cast<float>();
			Vector3d _temp_d = _float_target_points[i].cast<double>();
			_coarse_float_target_points.push_back(_temp_f);
			target_points.push_back(_temp_d);
			Vector3d _temp_normal = _float_target_normals[i].cast<double>();
			target_normals.push_back(_temp_normal);
			double r,g,b;
			r = int(_uchar_target_colors[i].x())/255;
			g =	int(_uchar_target_colors[i].y())/255;
			b = int(_uchar_target_colors[i].z())/255;
			Vector3d _temp_color = {r, g, b};
			target_colors.push_back(_temp_color);
		}

		// Build the index of the FLANN tree (for fast nearest neighbor lookup).
		// float
		m_nearestNeighborSearch_forSparse->buildIndex(target_landmarks.getPoints());
		m_nearestNeighborSearch_forDense->buildIndex(_coarse_float_target_points);


		// copy initial coefficients
		// The initial estimate can be given as an argument.
		std::vector<double> estimated_coefs_shape;
		std::copy(_initial_coef_shape.begin(), _initial_coef_shape.end(), std::back_inserter(estimated_coefs_shape));
		std::vector<double> estimated_coefs_tex;
		std::copy(_initial_coef_tex.begin(), _initial_coef_tex.end(), std::back_inserter(estimated_coefs_tex));
		std::vector<double> estimated_coefs_exp;
		std::copy(_initial_coef_exp.begin(), _initial_coef_exp.end(), std::back_inserter(estimated_coefs_exp));

		// set constraint_bfmManager
		Constraint_bfmManager constraint_bfmManager(SHAPE, TEX, EXP, BFM_triangle_list);

		int _int_num_shapePc = global_num_shape_pcs;
		int _int_num_texPc = global_num_tex_pcs;
		int _int_num_expPc = global_num_exp_pcs;
		int _int_num_vertices = global_num_vertices;

		// we optimize parameter for shape, tex, and expression
		// the dimensionalities can be defined by the top as global arguments
		double _increment_coef_shape[_int_num_shapePc];
		double _increment_coef_tex[_int_num_texPc];
		double _increment_coef_exp[_int_num_expPc];

		// class to update the bfm mesh
		auto bfmMeshUpdate = Generic_BFMUpdate<double>(_increment_coef_shape, _increment_coef_exp, _increment_coef_tex);
		bfmMeshUpdate.setZero();
		// iterative optimization
		for (int i = 0; i < m_nIterations; ++i)
		{
			// Compute the matches_sparse.
			std::cout << "Matching points ..." << std::endl;
			clock_t begin = clock();

			// Get transformed bfm landmarks position and normals to create FacePointCloud
			std::vector<Vector3d> updated_BFM_vertex_pos;
			std::vector<Vector3d> updated_BFM_vertex_rgb;
			std::tie(updated_BFM_vertex_pos, updated_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);

			std::vector<Vector3f> float_updated_BFM_vertex_pos = convert_double2float(updated_BFM_vertex_pos);
			std::vector<Vector3f> float_updated_BFM_vertex_rgb = convert_double2float(updated_BFM_vertex_rgb);

			// This face point cloud is containing entire bfm mesh vertices
			FacePointCloud transform_sourceMesh{float_updated_BFM_vertex_pos, BFM_triangle_list};

			// Get landmarks points to create transformed_sourceLandmarksMesh
			std::vector<Vector3f> postPE_bfm_landmarks_points = transform_sourceMesh.get_selectedVertexPos(bfm_landmarkIndex_list);

			// Set transformed landmarks
			FacePointCloud transformed_sourceLandmarksMesh{postPE_bfm_landmarks_points, sourceLandmarks_triangleLists};

			// get vector of landamarks position in double and their normals
			std::vector<Vector3d> d_transformed_sourceLandmarks_pos = convert_float2double(transformed_sourceLandmarksMesh.getPoints());
			std::vector<Vector3d> d_transformed_sourceLandmarks_normals = convert_float2double(transformed_sourceLandmarksMesh.getNormals());

			// get vector of all vertices position in double and their normals
			std::vector<Vector3d> d_transformed_source_pos = convert_float2double(transform_sourceMesh.getPoints());
			std::vector<Vector3d> d_transformed_source_normals = convert_float2double(transform_sourceMesh.getNormals());

			// matches_sparse contains the correspondences
			// float
			auto matches_sparse = m_nearestNeighborSearch_forSparse->queryMatches(transformed_sourceLandmarksMesh.getPoints());
			auto matches_dense = m_nearestNeighborSearch_forDense->queryMatches(transform_sourceMesh.getPoints());

			std::cout<<"Average distance [mm] of sparse: "<<sqrt(m_nearestNeighborSearch_forSparse->get_aveDist())<<std::endl;
			std::cout<<"Average distance [mm] of dense: "<<sqrt(m_nearestNeighborSearch_forDense->get_aveDist())<<std::endl;


			if (i == 0){
				//Goal: apply the distance to the each vertex color attribute
				/*
				* Large error: red
				* Small error: blue
				* 	Error is normalized [0, 1] by dividing the current error by maximum
				*   Map the normalized error on to red[0, 255], and blue [0,255]
				*   P = 0.01 (Small error)
				*   Red = int(255*(P))
				* 	Blue = int(255*(1-P))
				*
				*/
				std::cout<<"=================="<<"INIT EVALUTAION"<<"===================="<<std::endl;

				//get max distance, min distance
				float distanceMax = m_nearestNeighborSearch_forDense->get_measuredMaxDist();
				float distanceMin = m_nearestNeighborSearch_forDense->get_measuredMinDist();

				//get the distance of each vertex from their neighbor
				std::vector<float> measuredDists = m_nearestNeighborSearch_forDense->get_measuredDists();

				std::cout<<"MAX distance: "<<distanceMax<<std::endl;
				std::cout<<"MIN distance: "<<distanceMin<<std::endl;

				std::cout<<"Length of the measureDists: "<<measuredDists.size()<<std::endl;

				std::string f_name_before = "../output/before_paramEst2.ply";
				Generic_writeFaceMeshPly(f_name_before, float_updated_BFM_vertex_pos, float_updated_BFM_vertex_rgb, BFM_triangle_list);

				// get color attributes according to the error
				std::vector<Vector3f> error_ColorMap_before = get_GeoErrorPointColors(measuredDists, distanceMax, distanceMin);
				std::string f_name_before_errorMap = "../output/before_paramEst2_errorMap.ply";
				Generic_writeFaceMeshPly(f_name_before_errorMap,float_updated_BFM_vertex_pos, error_ColorMap_before, BFM_triangle_list);

			}

			pruneCorrespondences(d_transformed_sourceLandmarks_normals, targetLandmarks_normals, matches_sparse);
			pruneCorrespondences(d_transformed_source_normals, target_normals, matches_dense);

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			// Prepare point-to-point constraints.
			ceres::Problem problem;

			// Add sparse cost functions w.r.t shape and exp parameter
			const int num_Landmark_vertex = d_transformed_sourceLandmarks_pos.size();
			prepareConstraints_Sparse_wrtEXP(num_Landmark_vertex, targetLandmarks_points, targetLandmarks_normals, matches_sparse, bfmMeshUpdate, problem, &constraint_bfmManager, bfm_landmarkIndex_list, estimated_coefs_shape);

			// Add dense const functions w.r.t shape, exp parameter
			const int num_BFMmesh_vertex = d_transformed_source_pos.size();
			prepareConstraints_Dense_wrtEXP(num_BFMmesh_vertex, target_points, target_normals, target_colors, matches_dense, bfmMeshUpdate, problem, &constraint_bfmManager, estimated_coefs_shape);

			// const int num_BFMmesh_vertex = d_transformed_source_pos.size();
			prepareConstraints_Color(num_BFMmesh_vertex, target_colors, matches_dense, bfmMeshUpdate, problem, &constraint_bfmManager);

			// Configure options for the solver.
			std::cout << "Start to solve" << std::endl;
			ceres::Solver::Options options;
			configureSolver(options);

			// Run the solver (for one iteration).
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			// std::cout << summary.BriefReport() << std::endl;
			std::cout << summary.FullReport() << std::endl;

			double *delta_coefs_shape = bfmMeshUpdate.getData_shape();
			double *delta_coefs_tex = bfmMeshUpdate.getData_tex();
			double *delta_coefs_exp = bfmMeshUpdate.getData_exp();

			for (unsigned int i = 0; i < _int_num_expPc; i++)
			{
				estimated_coefs_exp[i] =+ delta_coefs_exp[i];
			}

			for (unsigned int i = 0; i < _int_num_texPc; i++)
			{
				estimated_coefs_tex[i] =+ delta_coefs_tex[i];
			}

			bfmMeshUpdate.setZero();
			std::cout << "Optimization iteration done." << std::endl;
		}

		std::vector<Vector3d> final_BFM_vertex_pos;
		std::vector<Vector3d> final_BFM_vertex_rgb;
		std::tie(final_BFM_vertex_pos, final_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);
		std::vector<Vector3f> float_final_BFM_vertex_pos = convert_double2float(final_BFM_vertex_pos);
		std::vector<Vector3f> float_final_BFM_vertex_rgb = convert_double2float(final_BFM_vertex_rgb);

		std::cout<<"=================="<<"FINAL EVALUTAION"<<"===================="<<std::endl;
		//Evaluation
		FacePointCloud final_sourceMesh{float_final_BFM_vertex_pos, BFM_triangle_list};
		auto matches_Eval = m_nearestNeighborSearch_forDense->queryMatches(final_sourceMesh.getPoints());

		std::cout<<"Average distance [mm] of dense evaluation: "<<sqrt(m_nearestNeighborSearch_forDense->get_aveDist())<<std::endl;
		float final_MaxDist = m_nearestNeighborSearch_forDense->get_measuredMaxDist();
		float final_MinDist = m_nearestNeighborSearch_forDense->get_measuredMinDist();

		//get the distance of each vertex from their neighbor
		std::vector<float> measuredDists_final = m_nearestNeighborSearch_forDense->get_measuredDists();

		std::cout<<"MAX distance: "<<final_MaxDist<<std::endl;
		std::cout<<"MIN distance: "<<final_MinDist<<std::endl;
		std::cout<<"Length of the measureDists: "<<measuredDists_final.size()<<std::endl;

		// get color attributes according to the error
		std::vector<Vector3f> error_ColorMap_final = get_GeoErrorPointColors(measuredDists_final, final_MaxDist, final_MinDist);
		std::string f_name_errorMap = "../output/after_paramEst2_errorMap.ply";
		Generic_writeFaceMeshPly(f_name_errorMap, float_final_BFM_vertex_pos, error_ColorMap_final, BFM_triangle_list);


		//get exp coeffs
		std::cout<<"EXP coeffs"<<std::endl;
		for(auto & item : estimated_coefs_exp){
			std::cout<<item<<",";
		}
		std::cout<<std::endl;

		std::string f_name1 = "../output/after_paramEst2.ply";
		Generic_writeFaceMeshPly(f_name1,float_final_BFM_vertex_pos, float_final_BFM_vertex_rgb, BFM_triangle_list);
		std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> _result_coeffs;
		_result_coeffs = std::make_tuple(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);
		return _result_coeffs;
	}

	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams_colors(const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM &bfm, std::vector<Vector3i> BFM_triangle_list) override
	{
			/*source_landmarks
			- bfm landmarks vertex position
			- bfm landmarks vertex normal which is obtained by delaunay triangulation
			*/

			/*target_landmarks
			- dlib detected landmarks position which are back projected in kinect camera space
			- dlib detected landmarks vertex normal which is obtained by delaunay triangulation
			*/

			// Build the index of the FLANN tree (for fast nearest neighbor lookup).

			// For dense term
			std::vector<Vector3f> _float_target_points = target.getPoints();
			std::vector<Vector3f> _float_target_normals = target.getNormals();
			std::vector<Vector3uc> _uchar_target_colors = target.getColors();
			std::vector<Vector3f> _coarse_float_target_points;
			std::vector<Vector3d> target_points;
			std::vector<Vector3d> target_normals;
			std::vector<Vector3d> target_colors;

			// reduction of target points and normals with giving kernel size
			std::cout<<"num of vertices of RGBD scan: "<<_float_target_points.size()<<std::endl;
			int kernel_size = 3;
			for(int i = 0; i < _float_target_points.size(); i=i+kernel_size){
				Vector3f _temp_f = _float_target_points[i].cast<float>();
				Vector3d _temp_d = _float_target_points[i].cast<double>();
				_coarse_float_target_points.push_back(_temp_f);
				target_points.push_back(_temp_d);
				Vector3d _temp_normal = _float_target_normals[i].cast<double>();
				target_normals.push_back(_temp_normal);
				double r,g,b;
				r = int(_uchar_target_colors[i].x())/255;
				g =	int(_uchar_target_colors[i].y())/255;
				b = int(_uchar_target_colors[i].z())/255;
				Vector3d _temp_color = {r, g, b};
				target_colors.push_back(_temp_color);
			}

			m_nearestNeighborSearch_forColor->buildIndex(_coarse_float_target_points);

			// copy initial coefficients
			// The initial estimate can be given as an argument.
			std::vector<double> estimated_coefs_shape;
			std::vector<double> estimated_coefs_tex;
			std::vector<double> estimated_coefs_exp;
			std::copy(_initial_coef_exp.begin(), _initial_coef_exp.end(), std::back_inserter(estimated_coefs_exp));
			std::copy(_initial_coef_shape.begin(), _initial_coef_shape.end(), std::back_inserter(estimated_coefs_shape));
			std::copy(_initial_coef_tex.begin(), _initial_coef_tex.end(), std::back_inserter(estimated_coefs_tex));

			// set constraint_bfmManager
			Constraint_bfmManager constraint_bfmManager(SHAPE, TEX, EXP, BFM_triangle_list);

			int _int_num_shapePc = global_num_shape_pcs;
			int _int_num_texPc = global_num_tex_pcs;
			int _int_num_expPc = global_num_exp_pcs;
			int _int_num_vertices = global_num_vertices;

			// we optimize parameter for shape, tex, and expression
			// the dimensionalities can be defined by the top as global arguments
			double _increment_coef_shape[_int_num_shapePc];
			double _increment_coef_tex[_int_num_texPc];
			double _increment_coef_exp[_int_num_expPc];

			// class to update the bfm mesh
			auto bfmMeshUpdate = Generic_BFMUpdate<double>(_increment_coef_shape, _increment_coef_exp, _increment_coef_tex);
			bfmMeshUpdate.setZero();
			// iterative optimization
			for (int i = 0; i < m_nIterations; ++i)
			{
				// Compute the matches_sparse.
				std::cout << "Matching points ..." << std::endl;
				clock_t begin = clock();

				// Get transformed bfm landmarks position and normals to create FacePointCloud
				std::vector<Vector3d> updated_BFM_vertex_pos;
				std::vector<Vector3d> updated_BFM_vertex_rgb;
				std::tie(updated_BFM_vertex_pos, updated_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);


				std::vector<Vector3f> float_updated_BFM_vertex_pos = convert_double2float(updated_BFM_vertex_pos);
				std::vector<Vector3f> float_updated_BFM_vertex_rgb = convert_double2float(updated_BFM_vertex_rgb);

				FacePointCloud transform_sourceMesh{float_updated_BFM_vertex_pos, BFM_triangle_list};

				// get vector of all vertices position in double and their normals
				std::vector<Vector3d> d_transformed_source_pos = convert_float2double(transform_sourceMesh.getPoints());
				std::vector<Vector3d> d_transformed_source_normals = convert_float2double(transform_sourceMesh.getNormals());

				// matches_sparse contains the correspondences
				// float
				auto matches = m_nearestNeighborSearch_forColor->queryMatches(transform_sourceMesh.getPoints());
				std::cout<<"Average distance in Color space: "<<sqrt(m_nearestNeighborSearch_forColor->get_aveDist())<<std::endl;

			if (i == 0){
				//Goal: apply the distance to the each vertex color attribute
				/*
				* Large error: red
				* Small error: blue
				* 	Error is normalized [0, 1] by dividing the current error by maximum
				*   Map the normalized error on to red[0, 255], and blue [0,255]
				*   P = 0.01 (Small error)
				*   Red = int(255*(P))
				* 	Blue = int(255*(1-P))
				*
				*/
				std::cout<<"=================="<<"INIT EVALUTAION"<<"===================="<<std::endl;

				//get max distance, min distance
				float distanceMax = m_nearestNeighborSearch_forColor->get_measuredMaxDist();
				float distanceMin = m_nearestNeighborSearch_forColor->get_measuredMinDist();

				//get the distance of each vertex from their neighbor
				std::vector<float> measuredDists = m_nearestNeighborSearch_forColor->get_measuredDists();

				std::cout<<"MAX distance: "<<distanceMax<<std::endl;
				std::cout<<"MIN distance: "<<distanceMin<<std::endl;

				std::cout<<"Length of the measureDists: "<<measuredDists.size()<<std::endl;

				std::string f_name_before = "../output/before_ColorParamEst.ply";
				Generic_writeFaceMeshPly(f_name_before, float_updated_BFM_vertex_pos, float_updated_BFM_vertex_rgb, BFM_triangle_list);
			}

				pruneCorrespondences(d_transformed_source_normals, target_normals, matches);
				clock_t end = clock();
				double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
				std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

				// Prepare point-to-point constraints.
				ceres::Problem problem;

				const int num_BFMmesh_vertex = d_transformed_source_pos.size();
				prepareConstraints_Color(num_BFMmesh_vertex, target_colors, matches, bfmMeshUpdate, problem, &constraint_bfmManager);

				// Configure options for the solver.
				std::cout << "Start to solve" << std::endl;
				ceres::Solver::Options options;
				configureSolver(options);

				// Run the solver (for one iteration).
				ceres::Solver::Summary summary;
				ceres::Solve(options, &problem, &summary);
				std::cout << summary.FullReport() << std::endl;

				double *delta_coefs_tex = bfmMeshUpdate.getData_tex();
				for (unsigned int i = 0; i < _int_num_texPc; i++)
				{
					estimated_coefs_tex[i] =+ delta_coefs_tex[i];
				}

				bfmMeshUpdate.setZero();
				std::cout << "Optimization iteration done." << std::endl;
			}
			std::vector<Vector3d> final_BFM_vertex_pos;
			std::vector<Vector3d> final_BFM_vertex_rgb;
			std::tie(final_BFM_vertex_pos, final_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);
			std::vector<Vector3f> float_final_BFM_vertex_pos = convert_double2float(final_BFM_vertex_pos);
			std::vector<Vector3f> float_final_BFM_vertex_rgb = convert_double2float(final_BFM_vertex_rgb);
			std::string f_name1 = "../output/after_ColorparamEst.ply";
			Generic_writeFaceMeshPly(f_name1,float_final_BFM_vertex_pos, float_final_BFM_vertex_rgb, BFM_triangle_list);
			final_BFM_vertex_pos.clear();
			final_BFM_vertex_rgb.clear();
			float_final_BFM_vertex_pos.clear();
			float_final_BFM_vertex_rgb.clear();

			//44 Smile
			std::vector<double> explicit_exp_coefs = {-60.5002,27.46,-396.384,221.057,-351.23,16.7272,-343.676,-164.863,-21.9813,-259.791,53.972,-172.793,-29.7058,-138.396,141.019,-34.7468,-5.0239,90.2333,155.095,-173.744,78.703,-29.8605,-156.931,-114.539,88.5187,-4.64664,-67.6627,17.8961,78.5553,-93.099,-180.531,1.58158,-57.9478,-15.3739,11.7706,-16.5877,-2.51691,16.1146,-28.3671,8.35683,-46.2286,-35.3036,-6.39366,56.864,-29.0351,-29.7691,11.2961,-119.663,77.6659,1.30823};
			//21 Smile (Actually appears sad)
			// std::vector<double> explicit_exp_coefs = {4.04974,128.429,126.539,-9.12486,81.8324,85.9868,-67.0875,297.4,305.492,-32.1442,290.044,-96.6697,-79.0005,-35.9535,-24.4355,3.34077,147.237,-112.474,30.6774,-136.572,47.0948,24.4614,-42.1363,38.5682,-140.302,14.9899,-186.613,2.80567,-55.6361,-85.2455,18.306,23.8937,-8.2485,-85.6997,13.0689,1.33718,-46.1478,-219.79,-67.865,-106.109,-71.2794,-18.5828,3.31352,77.0736,-65.4339,-16.1404,-36.9204,2.82434,20.1113,-51.5541};

			std::vector<Vector3d> FaceReanact_BFM_vertex_pos;
			std::vector<Vector3d> FaceReanact_BFM_vertex_rgb;
			std::tie(FaceReanact_BFM_vertex_pos, FaceReanact_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, explicit_exp_coefs);
			std::vector<Vector3f> FaceReanact_float_BFM_vertex_pos = convert_double2float(FaceReanact_BFM_vertex_pos);
			std::vector<Vector3f> FaceReanact_float_BFM_vertex_rgb = convert_double2float(FaceReanact_BFM_vertex_rgb);
			std::string f_name_reanact = "../output/after_paramEst_FACEREANACMENT.ply";
			Generic_writeFaceMeshPly(f_name_reanact, FaceReanact_float_BFM_vertex_pos, FaceReanact_float_BFM_vertex_rgb, BFM_triangle_list);
			FaceReanact_BFM_vertex_pos.clear();
			FaceReanact_BFM_vertex_rgb.clear();
			FaceReanact_float_BFM_vertex_pos.clear();
			FaceReanact_float_BFM_vertex_rgb.clear();

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

	void prepareConstraints_Sparse(const int _nPoints, const std::vector<Vector3d> &targetPoints, const std::vector<Vector3d> &targetNormals, const std::vector<Match> matches_sparse, const Generic_BFMUpdate<double> &generic_BFMUpdate, ceres::Problem &problem, Constraint_bfmManager *_constraint_bfmManager, std::vector<int> &_bfm_landmarkIndex_list) const
	{
		// shape_BFMUpdate (containing the lie algebra) is to be optimized
		/* shape_BFMUpdate will return the address of array (6 elements)
		double* pose = shape_BFMUpdate.getData();
		double* rotation = pose;
		double* translation = pose + 3;
		*/
		const int nPoints = _nPoints;

		std::cout << "Nearest neighbor matched pairs: " << matches_sparse.size() << std::endl;

		for (int i = 0; i < nPoints; ++i)
		{
			// matches_sparse[i] contains that corresponding vertex index of i
			/*
			 *i: index of source point
			 *match.index: index of corresponding target point
			 *match.weight: weight of the correspondence
			 */

			const auto vertex_index = _bfm_landmarkIndex_list[i];
			const auto match = matches_sparse[i];
			if (match.idx >= 0)
			{
				const auto &targetPoint = targetPoints[match.idx];
				const auto &weight = match.weight;

				if (!targetPoint.allFinite())
					continue;

				ceres::CostFunction *cost_function = PointToPointConstraint_shape::create(targetPoint, weight, vertex_index, _constraint_bfmManager, global_sparse_lambda);
				problem.AddResidualBlock(cost_function, nullptr, generic_BFMUpdate.getData_shape());

			}
		}
		ceres::CostFunction *cost_function_regSHAPE = Regularizer_shape::create(_constraint_bfmManager);
		problem.AddResidualBlock(cost_function_regSHAPE, nullptr, generic_BFMUpdate.getData_shape());
	}

	void prepareConstraints_Sparse_wrtEXP(const int _nPoints, const std::vector<Vector3d> &targetPoints, const std::vector<Vector3d> &targetNormals, const std::vector<Match> matches_sparse, const Generic_BFMUpdate<double> &generic_BFMUpdate, ceres::Problem &problem, Constraint_bfmManager *_constraint_bfmManager, std::vector<int> &_bfm_landmarkIndex_list, std::vector<double> & _coefs_shape) const
	{
		// shape_BFMUpdate (containing the lie algebra) is to be optimized
		/* shape_BFMUpdate will return the address of array (6 elements)
		double* pose = shape_BFMUpdate.getData();
		double* rotation = pose;
		double* translation = pose + 3;
		*/
		const int nPoints = _nPoints;

		std::cout << "Nearest neighbor matched pairs: " << matches_sparse.size() << std::endl;

		for (int i = 0; i < nPoints; ++i)
		{
			// matches_sparse[i] contains that corresponding vertex index of i
			/*
			 *i: index of source point
			 *match.index: index of corresponding target point
			 *match.weight: weight of the correspondence
			 */

			const auto vertex_index = _bfm_landmarkIndex_list[i];
			const auto match = matches_sparse[i];
			if (match.idx >= 0)
			{
				// const auto &sourcePoint = sourcePoints[i];
				const auto &targetPoint = targetPoints[match.idx];
				const auto &weight = match.weight;

				if (!targetPoint.allFinite())
					continue;

				ceres::CostFunction *cost_function = PointToPointConstraint_exp::create(targetPoint, weight, vertex_index, _constraint_bfmManager, global_sparse_lambda, _coefs_shape);
				problem.AddResidualBlock(cost_function, nullptr, generic_BFMUpdate.getData_exp());
			}
		}

		ceres::CostFunction *cost_function_regEXP = Regularizer_exp::create(_constraint_bfmManager);
		problem.AddResidualBlock(cost_function_regEXP, nullptr, generic_BFMUpdate.getData_exp());
	}

	void prepareConstraints_Dense(const int _nPoints, const std::vector<Vector3d> &targetPoints, const std::vector<Vector3d> &targetNormals, const std::vector<Vector3d> &targetColors,const std::vector<Match> matches_dense, const Generic_BFMUpdate<double> &generic_BFMUpdate, ceres::Problem &problem, Constraint_bfmManager *_constraint_bfmManager) const
	{
		// shape_BFMUpdate (containing the lie algebra) is to be optimized
		/* shape_BFMUpdate will return the address of array (6 elements)
		double* pose = shape_BFMUpdate.getData();
		double* rotation = pose;
		double* translation = pose + 3;
		*/
		const int nPoints = _nPoints;

		std::cout << "Nearest neighbor matched pairs: " << matches_dense.size() << std::endl;

		for (int i = 0; i < nPoints; i = i+1)
		{
			// matches_dense[i] contains that corresponding vertex index of i
			/*
			 *i: index of source point
			 *match.index: index of corresponding target point
			 *match.weight: weight of the correspondence
			 */

			const auto vertex_index = i;
			const auto vertex_index_c = i;
			const auto match = matches_dense[i];
			if (match.idx >= 0)
			{
				const auto &targetPoint = targetPoints[match.idx];
				const auto &targetColor = targetColors[match.idx];
				const auto &weight = match.weight;
				const auto &weight_c = match.weight;

				if (!targetPoint.allFinite())
					continue;

				ceres::CostFunction *cost_function = PointToPointConstraint_shape::create(targetPoint, weight, vertex_index, _constraint_bfmManager, global_dense_lambda);
				problem.AddResidualBlock(cost_function, nullptr, generic_BFMUpdate.getData_shape());

			}
		}
		//regularizer
		ceres::CostFunction *cost_function_regSHAPE = Regularizer_shape::create(_constraint_bfmManager);
		problem.AddResidualBlock(cost_function_regSHAPE, nullptr, generic_BFMUpdate.getData_shape());
	}

	void prepareConstraints_Dense_wrtEXP(const int _nPoints, const std::vector<Vector3d> &targetPoints, const std::vector<Vector3d> &targetNormals, const std::vector<Vector3d> &targetColors,const std::vector<Match> matches_dense, const Generic_BFMUpdate<double> &generic_BFMUpdate, ceres::Problem &problem, Constraint_bfmManager *_constraint_bfmManager, std::vector<double> & _coefs_shape) const
	{
		// shape_BFMUpdate (containing the lie algebra) is to be optimized
		/* shape_BFMUpdate will return the address of array (6 elements)
		double* pose = shape_BFMUpdate.getData();
		double* rotation = pose;
		double* translation = pose + 3;
		*/
		const int nPoints = _nPoints;

		std::cout << "Nearest neighbor matched pairs: " << matches_dense.size() << std::endl;

		for (int i = 0; i < nPoints; i = i+1)
		{
			// matches_dense[i] contains that corresponding vertex index of i
			/*
			 *i: index of source point
			 *match.index: index of corresponding target point
			 *match.weight: weight of the correspondence
			 */

			const auto vertex_index = i;
			const auto vertex_index_c = i;
			const auto match = matches_dense[i];
			if (match.idx >= 0)
			{
				const auto &targetPoint = targetPoints[match.idx];
				const auto &targetColor = targetColors[match.idx];
				const auto &weight = match.weight;
				const auto &weight_c = match.weight;

				if (!targetPoint.allFinite())
					continue;

				ceres::CostFunction *cost_function = PointToPointConstraint_exp::create(targetPoint, weight, vertex_index, _constraint_bfmManager, global_dense_lambda, _coefs_shape);
				problem.AddResidualBlock(cost_function, nullptr, generic_BFMUpdate.getData_exp());

			}
		}
		ceres::CostFunction *cost_function_regEXP = Regularizer_exp::create(_constraint_bfmManager);
		problem.AddResidualBlock(cost_function_regEXP, nullptr, generic_BFMUpdate.getData_exp());

	}

	void prepareConstraints_Color(const int _nPoints, const std::vector<Vector3d> &targetColors, const std::vector<Match> matches_dense, const Generic_BFMUpdate<double> &generic_BFMUpdate, ceres::Problem &problem, Constraint_bfmManager *_constraint_bfmManager) const
	{
		// shape_BFMUpdate (containing the lie algebra) is to be optimized
		/* shape_BFMUpdate will return the address of array (6 elements)
		double* pose = shape_BFMUpdate.getData();
		double* rotation = pose;
		double* translation = pose + 3;
		*/
		const int nPoints = _nPoints;

		std::cout << "Nearest neighbor matched pairs: " << matches_dense.size() << std::endl;

		for (int i = 0; i < nPoints; i = i+1)
		{
			// matches_dense[i] contains that corresponding vertex index of i
			/*
			 *i: index of source point
			 *match.index: index of corresponding target point
			 *match.weight: weight of the correspondence
			 */

			const auto vertex_index = i;
			const auto match = matches_dense[i];
			if (match.idx >= 0)
			{
				const auto &targetColor = targetColors[match.idx];
				const auto &weight = match.weight;
				ceres::CostFunction *cost_function_color = ColorContraint::create(targetColor, weight, vertex_index, _constraint_bfmManager);
				problem.AddResidualBlock(cost_function_color, nullptr, generic_BFMUpdate.getData_tex());
			}
		}
	}
};
