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

const unsigned int global_num_shape_pcs = 199;
const unsigned int global_num_tex_pcs = 199;
const unsigned int global_num_exp_pcs = 100;
const unsigned int global_num_vertices = 53149;
const double global_reg_lambda = (2) * 1e1;

void set_global_variables(int i0, int i1, int i2, int i3){
	const unsigned int global_num_shape_pcs = i0;
	const unsigned int global_num_tex_pcs = i1;
	const unsigned int global_num_exp_pcs = i2;
	const double global_reg_lambda = double(i3);
}

void Generic_writeFaceMeshPly(std::string fn, std::vector<Vector3f> & point_clouds, std::vector<Vector3i> & triangle_list){
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
		out << "element face " << triangle_list.size() << "\n";
		out << "property list uchar int vertex_indices\n";
		out << "end_header\n";

        // unsigned long int cnt = 0;
        for (Vector3f point : point_clouds)
        {
			if(point.allFinite()){
            	out<<point.x()<<" "<<point.y()<<" "<<point.z()<<"\n";
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
	void get_SHAPEPc(const int _index, const int _dim, T **pc)
	{
		int index = int(_index);
		int dim = int(_dim);

		for (unsigned int j = 0; j < dim; j++)
		{
			auto x = SHAPE.pc(index, j);
			auto y = SHAPE.pc(index + 1, j);
			auto z = SHAPE.pc(index + 2, j);
			// std::cout<<"x, y, z (original): "<<x<<","<<y<<","<<z<<std::endl;
			pc[0][j] = T(x);
			pc[1][j] = T(y);
			pc[2][j] = T(z);
			// std::cout<<"x, y, z (copy): "<<pc[0][j]<<","<<pc[1][j]<<","<<pc[2][j]<<std::endl;
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
		// std::cout << "get shape MU" << std::endl;
		auto x = SHAPE.mean[_index];
		auto y = SHAPE.mean[_index + 1];
		auto z = SHAPE.mean[_index + 2];
		means[0] = T(x);
		means[1] = T(y);
		means[2] = T(z);
		// std::cout << "x,y,z" << means[0] << "," << means[1] << "," << means[2] << std::endl;

		// std::cout << std::endl;
	}

	template <typename T>
	void get_TEXMu(T *means, const int _index)
	{
		// std::cout << "get tex MU" << std::endl;
		auto x = TEX.mean[_index];
		auto y = TEX.mean[_index + 1];
		auto z = TEX.mean[_index + 2];
		means[0] = T(x);
		means[1] = T(y);
		means[2] = T(z);
		// std::cout << "x,y,z" << means[0] << "," << means[1] << "," << means[2] << std::endl;

		// std::cout << std::endl;
	}

	template <typename T>
	void get_EXPMu(T *means, const int _index)
	{
		// std::cout << "get EXP MU" << std::endl;
		auto x = EXP.mean[_index];
		auto y = EXP.mean[_index + 1];
		auto z = EXP.mean[_index + 2];
		means[0] = T(x);
		means[1] = T(y);
		means[2] = T(z);
		// std::cout << "x,y,z" << means[0] << "," << means[1] << "," << means[2] << std::endl;

		// std::cout << std::endl;
	}

	template <typename T>
	void get_SHAPEStd(T & std, const int _index){
		auto _std = SHAPE.variance[_index];
		_std = std::sqrt(_std);
		std = T(_std);
	}

	template <typename T>
	void get_EXPStd(T & std, const int _index){
		auto _std = EXP.variance[_index];
		_std = std::sqrt(_std);
		std = T(_std);
	}

	template <typename T>
	void get_TEXStd(T & std, const int _index){
		auto _std = TEX.variance[_index];
		_std = std::sqrt(_std);
		std = T(_std);
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
		// assert(nLength >= 0);

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

	void set_Parameters()
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

	/**
	 * Applies the parameters to average bfm mesh
	 * Output the resulting vertex positions.
	 */
	// void apply_params(const int index, T *transformed_point) const
	// {
	// 	// input
	// 	//  vecShapeMU: shape[53149*3] = 159447
	// 	//  matShapePC: shape[53149, 199]
	// 	// output
	// 	//  current_shape: shape[53149*3]

	// 	int _int_dim = int(dim);

	// 	T tmpMu[3 * num_vertices];
	// 	for (auto i = 0; i = num_vertices; i++){
	// 		tmpMu[3*i] = T(means[3*i]);
	// 		tmpMu[3*i+1] = T(means[3*i+1]);
	// 		tmpMu[3*i+2] = T(means[3*i+2]);
	// 	}

	// 	T **tmpPc;
	//     tmpPc = new T*[3]; // replaced "int" for "T"

	//     for (int i = 0; i < 3; i++) {
	//         tmpPc[i] = new T [_int_dim]; // replaced "int" for "T"
	//     }

	// 	for(int i = 0; i < 3; i++){
	// 		for(int j = 0; j < _int_dim; j++){
	// 			tmpPc[i][j] = T(pcs[i][j]);
	// 		}
	// 	}

	// 	T tmpCoef[_int_dim];
	// 	for (auto i = 0; i < _int_dim; i++)
	// 		tmpCoef[i] = m_array[i];

	// 	T current_shape[3];
	// 	current_shape[0] = T(0);
	// 	current_shape[1] = T(0);
	// 	current_shape[2] = T(0);

	// 	const int cnt = 3 * index;
	// 	T noise[3];
	// 	noise[0] = T(0);
	// 	noise[1] = T(0);
	// 	noise[2] = T(0);

	// 	int _int_cnt = cnt;

	// 	for (int c = 0; c < _int_dim; c++)
	// 	{
	// 		T _out0 = T(tmpPc[0][c]) * T(tmpCoef[c]);
	// 		T _out1 = T(tmpPc[1][c]) * T(tmpCoef[c]);
	// 		T _out2 = T(tmpPc[2][c]) * T(tmpCoef[c]);
	// 		noise[0] += T(_out0);
	// 		noise[1] += T(_out1);
	// 		noise[2] += T(_out2);
	// 	}

	// 	current_shape[0] = T(tmpMu[_int_cnt]) + T(noise[0]);
	// 	current_shape[1] = T(tmpMu[_int_cnt + 1]) + T(noise[1]);
	// 	current_shape[2] = T(tmpMu[_int_cnt + 2]) + T(noise[2]);

	// 	transformed_point[0] = T(current_shape[0]);
	// 	transformed_point[1] = T(current_shape[1]);
	// 	transformed_point[2] = T(current_shape[2]);
	// }

private:
	// m_shape. m_exp, m_tex = coefficients
	T *shape_coefs;
	T *exp_coefs;
	T *tex_coefs;
	// T ** shape_pcs;
	// T * shape_mu;
	// T ** exp_pcs;
	// T * exp_mu;
	// T ** tex_pcs;
	// T * tex_mu;
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
	PointToPointConstraint(const Vector3d &sourcePoint, const Vector3d &targetPoint, const double weight, const int _index, Constraint_bfmManager *_constraint_bfmManager) : m_sourcePoint{sourcePoint}, m_targetPoint{targetPoint}, m_weight{weight}, m_index{_index}, constraint_bfmManager{_constraint_bfmManager}
	{
	}

	template <typename T>
	bool operator()(const T *const _coefs_shape, const T *const _coefs_exp, T *residuals) const
	{
		const int _template_index = m_index;
		const int _index = 3 * m_index;

		// std::cout << "index: " << _index << std::endl;

		T **_shape_matPc;
		_shape_matPc = new T *[3]; // replaced "int" for "T"

		for (int i = 0; i < 3; i++)
		{
			_shape_matPc[i] = new T[dim_shape]; // replaced "int" for "T"
		}

		// std::cout << "Before set principal components" << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < dim_shape; j++)
			{
				_shape_matPc[i][j] = T(0); // replaced "int" for "T"
				// std::cout << _shape_matPc[i][j] << ",";
			}
			// std::cout << std::endl;
		}

		// std::cout << "Set SHAPE principal components" << std::endl;
		constraint_bfmManager->get_SHAPEPc<T>(_index, dim_shape, _shape_matPc);
		// std::cout << "DONE > Set SHAPE principal components" << std::endl;

		// for (unsigned int i = 0; i < 3; i++)
		// {
		// 	for (unsigned int j = 0; j < dim_shape; j++)
		// 	{
		// 		std::cout << _shape_matPc[i][j] << ",";
		// 	}
		// 	std::cout << std::endl;
		// }
	
		// std::cout << "Before set principal components" << std::endl;
		T _shape_mu[3];
		_shape_mu[0] = T(0);
		_shape_mu[1] = T(0);
		_shape_mu[2] = T(0);

		// std::cout<<"Set means for shape"<<std::endl;
		constraint_bfmManager->get_SHAPEMu<T>(_shape_mu,_index);
		// std::cout<<"DONE> Set means for shape"<<std::endl;

		// std::cout << "After: _shape_mu" << std::endl;
		// for (unsigned int i = 0; i < 3; i++)
		// {
		// 	std::cout << _shape_mu[i] << std::endl;
		// }

		T **_exp_matPc;
		_exp_matPc = new T *[3]; // replaced "int" for "T"

		for (int i = 0; i < 3; i++)
		{
			_exp_matPc[i] = new T[dim_shape]; // replaced "int" for "T"
		}

		// std::cout << "Before set principal components" << std::endl;
		// for (int i = 0; i < 3; i++)
		// {
		// 	for (int j = 0; j < dim_exp; j++)
		// 	{
		// 		_exp_matPc[i][j] = T(0); // replaced "int" for "T"
		// 		std::cout << _exp_matPc[i][j] << ",";
		// 	}
		// 	std::cout << std::endl;
		// }
		// std::cout << "Set EXP principal components" << std::endl;
		constraint_bfmManager->get_EXPPc<T>(_index, dim_exp, _exp_matPc);
		// std::cout << "DONE> Set EXP principal components" << std::endl;

		// std::cout << "After set EXP principal components" << std::endl;
		// for (unsigned int i = 0; i < 3; i++)
		// {
		// 	for (unsigned int j = 0; j < dim_shape; j++)
		// 	{
		// 		std::cout << _exp_matPc[i][j] << ",";
		// 	}
		// 	std::cout << std::endl;
		// }

		T _exp_mu[3];
		_exp_mu[0] = T(0);
		_exp_mu[1] = T(0);
		_exp_mu[2] = T(0);

		// std::cout << "Set EXP means"<< std::endl;
		constraint_bfmManager->get_EXPMu<T>(_exp_mu, _index);
		// std::cout << "Done> Set EXP means"<< std::endl;

		// std::cout << "After: _exp_mu" << std::endl;
		// for (unsigned int i = 0; i < 3; i++)
		// {
		// 	std::cout << _exp_mu[i] << std::endl;
		// }
		// Generic_BFMUpdate<T> bfmShapeUpdate = Generic_BFMUpdate<T>(_shape_mu, _shape_matPc, const_cast<T *const>(_coefs_shape), dim_shape);
		// Generic_BFMUpdate<T> bfmExpUpdate = Generic_BFMUpdate<T>(_exp_mu, _exp_matPc, const_cast<T *const>(_coefs_exp), dim_exp);

		// T updateShape_vertices[3];
		// updateShape_vertices[0] = T(0);
		// updateShape_vertices[1] = T(0);
		// updateShape_vertices[2] = T(0);

		// T updateExp_vertices[3];
		// updateExp_vertices[0] = T(0);
		// updateExp_vertices[1] = T(0);
		// updateExp_vertices[2] = T(0);

		// bfmShapeUpdate.apply_params(_template_index, updateShape_vertices);
		// bfmExpUpdate.apply_params(_template_index, updateExp_vertices);

		T tmpShapeCoef[dim_shape];
		for (auto i = 0; i < dim_shape; i++)
			tmpShapeCoef[i] = _coefs_shape[i];

		T tmpExpCoef[dim_shape];
		for (auto i = 0; i < dim_shape; i++)
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

		blendshape[0] = current_shape[0] + current_exp[0];
		blendshape[1] = current_shape[1] + current_exp[1];
		blendshape[2] = current_shape[2] + current_exp[2];

		T targetPoint[3];
		fillVector(m_targetPoint, targetPoint);

		T subtract_st[3];

		subtract_st[0] = blendshape[0] - targetPoint[0];
		subtract_st[1] = blendshape[1] - targetPoint[1];
		subtract_st[2] = blendshape[2] - targetPoint[2];

		double _weight = std::sqrt(m_weight);
		double _LAMBDA = std::sqrt(LAMBDA);
		T coeff = T(_LAMBDA) * T(_weight);

		// Three residual
		residuals[0] = subtract_st[0] * coeff;
		residuals[1] = subtract_st[1] * coeff;
		residuals[2] = subtract_st[2] * coeff;

		// residuals[0] = subtract_st[0];
		// residuals[1] = subtract_st[1];
		// residuals[2] = subtract_st[2];

		return true;
	}

	static ceres::CostFunction *create(const Vector3d &sourcePoint, const Vector3d &targetPoint, const double weight, const int index, Constraint_bfmManager *_constraint_bfmManager)
	{
		return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, global_num_shape_pcs, global_num_exp_pcs>(
			new PointToPointConstraint(sourcePoint, targetPoint, weight, index, _constraint_bfmManager));
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	const Vector3d m_sourcePoint;
	const Vector3d m_targetPoint;
	const double m_weight;
	const int m_index;
	const int num_vertices = global_num_vertices;
	const double LAMBDA = 1.0;
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
			T std;
			constraint_bfmManager->get_SHAPEStd(std, i);
			double lambda = std::sqrt(LAMBDA);
			residuals[i] = (T(lambda)*coef[i])/std;
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
	const double LAMBDA = global_reg_lambda;
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
			T std;
			constraint_bfmManager->get_EXPStd(std, i);
			double lambda = std::sqrt(LAMBDA);
			residuals[i] = (T(lambda)*coef[i])/std;
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
	const double LAMBDA = global_reg_lambda;
	const int dim = global_num_exp_pcs;
};

class Regularizer_tex
{
public:
	Regularizer_tex(Constraint_bfmManager *_constraint_bfmManager) : constraint_bfmManager{_constraint_bfmManager}
	{
	}

	template <typename T>
	bool operator()(const T * const coef, T *residuals) const
	{
		for(int i = 0; i < dim; i++){
			T std;
			constraint_bfmManager->get_TEXStd(std, i);
			double lambda = std::sqrt(LAMBDA);
			residuals[i] = (T(lambda)*coef[i])/std;
		}
	}

	static ceres::CostFunction *create(Constraint_bfmManager *_constraint_bfmManager)
	{
		return new ceres::AutoDiffCostFunction<Regularizer_tex, global_num_tex_pcs, global_num_tex_pcs>(
			new Regularizer_tex(_constraint_bfmManager));
	}

protected:
	Constraint_bfmManager *constraint_bfmManager;
	const double LAMBDA = global_reg_lambda;
	const int dim = global_num_tex_pcs;
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

	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams(const FacePointCloud &source, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM &bfm, std::vector<int> &bfm_landmarkIndex_list){};

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

				// if(sourceNormal.allFinite() && targetNormal.allFinite()){
				// 	are_MINF = false;
				// 	counter_valid_point++;
				// }
				counter_valid_point++;
				// TODO: Invalidate the match (set it to -1) if the angle between the normals is greater than 60
				// sourceNormal and targetNormal are normalized to length 1

				// if(!are_MINF){
				double angle = acosf64x(sourceNormal.dot(targetNormal));
				if (angle > (double)(60.0))
				{
					match.idx = -1;
				}
				// 	}
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
	CeresICPOptimizer() {}
	// shape tex, exp
	virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> estimateParams(const FacePointCloud &source, const FacePointCloud &target, Parameter_set &SHAPE, Parameter_set &TEX, Parameter_set &EXP, std::vector<double> _initial_coef_shape, std::vector<double> _initial_coef_tex, std::vector<double> _initial_coef_exp, BFM &bfm, std::vector<int> &bfm_landmarkIndex_list) override
	{
		/*source
		- bfm landmarks vertex position
		- bfm landmarks vertex normal which is obtained by delaunay triangulation
		*/

		/*target
		- dlib detected landmarks position which are back projected in kinect camera space
		- dlib detected landmarks vertex normal which is obtained by delaunay triangulation
		*/

		// Build the index of the FLANN tree (for fast nearest neighbor lookup).
		int num_vertices = global_num_vertices;
		std::vector<Vector3f> _float_target_points = target.getPoints();
		std::cout << "Target_points: " << _float_target_points.size() << std::endl;
		std::vector<Vector3f> _float_target_normals = target.getNormals();
		std::cout << "Target_normals: " << _float_target_normals.size() << std::endl;
		std::vector<Vector3f> _float_source_points = source.getPoints();
		std::cout << "Source_points: " << _float_source_points.size() << std::endl;
		std::vector<Vector3f> _float_source_normals = source.getNormals();
		std::cout << "Source_normals: " << _float_source_normals.size() << std::endl;
		std::vector<Vector3d> target_points = convert_float2double(_float_target_points);
		std::vector<Vector3d> target_normals = convert_float2double(_float_target_normals);
		std::vector<Vector3d> source_points = convert_float2double(_float_source_points);
		std::vector<Vector3d> source_normals = convert_float2double(_float_source_normals);
		std::vector<Vector3i> target_triangleLists = target.getTriangleLists();
		std::vector<Vector3i> source_triangleLists = source.getTriangleLists();

		// std::cout << "target_points (dlib landmark): " << target_points.size() << std::endl;
		// for (auto item : target_points)
		// {
		// 	std::cout << item << std::endl;
		// }

		// std::cout << "target_normals (dlib landmark): " << target_normals.size() << std::endl;
		// for (auto item : target_normals)
		// {
		// 	std::cout << item << std::endl;
		// }

		// std::cout << "source_points (source landmark): " << source_points.size() << std::endl;
		// for (auto item : source_points)
		// {
		// 	std::cout << item << std::endl;
		// }

		// std::cout << "source_normals (source landmark): " << source_normals.size() << std::endl;
		// for (auto item : source_normals)
		// {
		// 	std::cout << item << std::endl;
		// }

		// Build the index of the FLANN tree (for fast nearest neighbor lookup).
		// float
		m_nearestNeighborSearch->buildIndex(target.getPoints());

		// copy initial coefficients
		// The initial estimate can be given as an argument.
		std::vector<double> estimated_coefs_shape;
		std::copy(_initial_coef_shape.begin(), _initial_coef_shape.end(), std::back_inserter(estimated_coefs_shape));
		std::vector<double> estimated_coefs_tex;
		std::copy(_initial_coef_tex.begin(), _initial_coef_tex.end(), std::back_inserter(estimated_coefs_tex));
		std::vector<double> estimated_coefs_exp;
		std::copy(_initial_coef_exp.begin(), _initial_coef_exp.end(), std::back_inserter(estimated_coefs_exp));

		// std::cout << "Estimated_coefs_shape" << std::endl;
		// for (auto item_shape : estimated_coefs_shape)
		// {
		// 	std::cout << item_shape << std::endl;
		// }

		// std::cout << "Estimated_coefs_tex" << std::endl;
		// for (auto item_tex : estimated_coefs_tex)
		// {
		// 	std::cout << item_tex << std::endl;
		// }

		// std::cout << "Estimated_coefs_exp" << std::endl;
		// for (auto item_exp : estimated_coefs_exp)
		// {
		// 	std::cout << item_exp << std::endl;
		// }

		// std::cout << "initial_coefs" << std::endl;
		// std::cout << "shape: " << estimated_coefs_shape.size() << std::endl;
		// std::cout << "tex: " << estimated_coefs_tex.size() << std::endl;
		// std::cout << "exp: " << estimated_coefs_exp.size() << std::endl;

		// To get triangle list of the BFM mesh to set constraint_bfmManager
		std::vector<Vector3f> before_BFM_vertex_pos;
		std::vector<Vector3f> before_BFM_vertex_rgb;
		std::vector<Vector3i> BFM_triangle_list;

		std::string f_name = "../output/before_paramEst.ply";
		std::tie(before_BFM_vertex_pos, before_BFM_vertex_rgb, BFM_triangle_list) = bfm.writeAveBFMmesh(f_name, true);
		before_BFM_vertex_pos.clear();
		before_BFM_vertex_rgb.clear();

		// auto tuple1 = bfm.transformedBFMMesh("../output/before_paramEst_transformed_mesh.ply", estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp, true);

		// std::cout << "before_BFM_vertex_pos" << std::endl;
		// for (auto &item : before_BFM_vertex_pos)
		// {
		// 	std::cout << item.transpose() << std::endl;
		// }

		// std::cout << "before_BFM_vertex_rgb" << std::endl;
		// for (auto &item : before_BFM_vertex_rgb)
		// {
		// 	std::cout << item.transpose() << std::endl;
		// }


		// set constraint_bfmManager
		Constraint_bfmManager constraint_bfmManager(SHAPE, TEX, EXP, BFM_triangle_list);

		int _int_num_shapePc = global_num_shape_pcs;
		int _int_num_texPc = global_num_tex_pcs;
		int _int_num_expPc = global_num_exp_pcs;

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
			// Compute the matches.
			std::cout << "Matching points ..." << std::endl;
			clock_t begin = clock();

			// std::cout << "Estimated_coefs_shape" << std::endl;
			// for (auto item_shape : estimated_coefs_shape)
			// {
			// 	std::cout << item_shape << std::endl;
			// }

			// std::cout << "Estimated_coefs_tex" << std::endl;
			// for (auto item_tex : estimated_coefs_tex)
			// {
			// 	std::cout << item_tex << std::endl;
			// }

			// std::cout << "Estimated_coefs_exp" << std::endl;
			// for (auto item_exp : estimated_coefs_exp)
			// {
			// 	std::cout << item_exp << std::endl;
			// }

			// get deformed vertices given estimated_shape_coefficients
			//  std::vector<Vector3f> deformed_source_vertices;
			//  std::vector<Vector3f> deformed_source_normals;
			// TODO:: create a function which can return deformed_source_vertices given coefficients for the shape

			// Get transformed bfm landmarks position and normals to create FacePointCloud
			std::vector<Vector3d> updated_BFM_vertex_pos;
			std::vector<Vector3d> updated_BFM_vertex_rgb;
			std::tie(updated_BFM_vertex_pos, updated_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);
			std::vector<Vector3f> float_updated_BFM_vertex_pos = convert_double2float(updated_BFM_vertex_pos);


			// std::string f_name1 = "../output/after_sparse_iteration_"+std::to_string(i)+".ply";
			// Generic_writeFaceMeshPly(f_name1,float_updated_BFM_vertex_pos, BFM_triangle_list);
			// std::cout << "updated_BFM_vertex_pos" << std::endl;
			// for (auto &item : updated_BFM_vertex_pos)
			// {
			// 	std::cout << item.transpose() << std::endl;
			// }

			// std::cout << "updated_BFM_vertex_rgb" << std::endl;
			// for (auto &item : updated_BFM_vertex_rgb)
			// {
			// 	std::cout << item.transpose() << std::endl;
			// }

			// std::cout << "float_updated_BFM_vertex_pos" << std::endl;
			// for (auto &item : float_updated_BFM_vertex_pos)
			// {
			// 	std::cout << item.transpose() << std::endl;
			// }

			// This face point cloud is containing entire bfm mesh vertices
			FacePointCloud transformed_BFMMesh{float_updated_BFM_vertex_pos, BFM_triangle_list};

			// Get landmarks points to create transformed_sourceMesh
			std::vector<Vector3f> postPE_bfm_landmarks_points = transformed_BFMMesh.get_selectedVertexPos(bfm_landmarkIndex_list);

			// std::cout << "postPE_bfm_landmarks_points" << std::endl;
			// for (auto &item : postPE_bfm_landmarks_points)
			// {
			// 	std::cout << item.transpose() << std::endl;
			// }
			// Set transformed_sourceMesh
			FacePointCloud transformed_sourceMesh{postPE_bfm_landmarks_points, source_triangleLists};

			// std::cout<<"updated_BFM_vertex_pos"<<std::endl;
			// for(int s = 0; s < updated_BFM_vertex_pos.size(); s++){
			// 	std::cout<<s<<"th vertex pos: "<<updated_BFM_vertex_pos[s].transpose()<<std::endl;
			// }

			std::vector<Vector3d> d_transformed_vertex_pos = convert_float2double(transformed_sourceMesh.getPoints());
			std::vector<Vector3d> d_transformed_normals = convert_float2double(transformed_sourceMesh.getNormals());

			// std::cout << "d_transformed_vertex_pos" << std::endl;
			// for (auto &item : d_transformed_vertex_pos)
			// {
			// 	std::cout << item.transpose() << std::endl;
			// }

			// std::cout << "d_transformed_normals" << std::endl;
			// for (auto &item : d_transformed_normals)
			// {
			// 	std::cout << item.transpose() << std::endl;
			// }
			// transformed_sourceMesh.getPoints(); //<= return std::vector<Vector3f> points
			// transformed_sourceMesh.getNormals(); //<= return std::vector<Vector3f> normals

			// matches contains the correspondences
			// float
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

			// Prepare point-to-point constraints.
			ceres::Problem problem;

			// Add residual w.r.t shape parameter
			prepareConstraints(d_transformed_vertex_pos, target_points, target_normals, matches, bfmMeshUpdate, problem, &constraint_bfmManager, bfm_landmarkIndex_list);

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

			for (unsigned int i = 0; i < _int_num_shapePc; i++)
			{
				estimated_coefs_shape[i] =+ delta_coefs_shape[i];
			}

			for (unsigned int i = 0; i < _int_num_expPc; i++)
			{
				estimated_coefs_exp[i] =+ delta_coefs_exp[i];
			}

			// for (unsigned int i = 0; i < _int_num_texPc; i++)
			// {
			// 	estimated_coefs_tex[i] =+ delta_coefs_tex[i];
			// }

			bfmMeshUpdate.setZero();
			std::cout << "Optimization iteration done." << std::endl;
		}
		std::vector<Vector3d> final_BFM_vertex_pos;
		std::vector<Vector3d> final_BFM_vertex_rgb;
		std::tie(final_BFM_vertex_pos, final_BFM_vertex_rgb) = constraint_bfmManager.get_tranformedBFMMesh(estimated_coefs_shape, estimated_coefs_tex, estimated_coefs_exp);
		std::vector<Vector3f> float_updated_BFM_vertex_pos = convert_double2float(final_BFM_vertex_pos);

		std::string f_name1 = "../output/after_paramEst.ply";
		Generic_writeFaceMeshPly(f_name1,float_updated_BFM_vertex_pos, BFM_triangle_list);
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

	void prepareConstraints(const std::vector<Vector3d> &sourcePoints, const std::vector<Vector3d> &targetPoints, const std::vector<Vector3d> &targetNormals, const std::vector<Match> matches, const Generic_BFMUpdate<double> &generic_BFMUpdate, ceres::Problem &problem, Constraint_bfmManager *_constraint_bfmManager, std::vector<int> &_bfm_landmarkIndex_list) const
	{
		// shape_BFMUpdate (containing the lie algebra) is to be optimized
		/* shape_BFMUpdate will return the address of array (6 elements)
		double* pose = shape_BFMUpdate.getData();
		double* rotation = pose;
		double* translation = pose + 3;
		*/
		const int nPoints = sourcePoints.size();

		std::cout << "Nearest neighbor matched pairs: " << matches.size() << std::endl;

		for (int i = 0; i < nPoints; ++i)
		{
			// matches[i] contains that corresponding vertex index of i
			/*
			 *i: index of source point
			 *match.index: index of corresponding target point
			 *match.weight: weight of the correspondence
			 */

			const auto vertex_index = _bfm_landmarkIndex_list[i];
			const auto match = matches[i];
			if (match.idx >= 0)
			{
				const auto &sourcePoint = sourcePoints[i];
				// std::cout<<"source point"<<sourcePoints[i]<<std::endl;
				const auto &targetPoint = targetPoints[match.idx];
				// std::cout<<"target point"<<targetPoints[match.idx]<<std::endl;
				const auto &weight = match.weight;

				// std::cout<<i<<"th weight: "<<match.weight<<std::endl;
				// std::cout << i << "point" << std::endl;

				if (!sourcePoint.allFinite() || !targetPoint.allFinite())
					continue;

				// TODO: Create a new point-to-point cost function and add it as constraint (i.e. residual block)
				// to the Ceres problem

				// PointToPointConstraint ptpc = {sourcePoint, targetPoint, weight};
				ceres::CostFunction *cost_function = PointToPointConstraint::create(sourcePoint, targetPoint, weight, vertex_index, _constraint_bfmManager);
				problem.AddResidualBlock(cost_function, nullptr, generic_BFMUpdate.getData_shape(), generic_BFMUpdate.getData_exp());

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

		//regularizer
		ceres::CostFunction *cost_function_regSHAPE = Regularizer_shape::create(_constraint_bfmManager);
		problem.AddResidualBlock(cost_function_regSHAPE, nullptr, generic_BFMUpdate.getData_shape());

		ceres::CostFunction *cost_function_regEXP = Regularizer_exp::create(_constraint_bfmManager);
		problem.AddResidualBlock(cost_function_regEXP, nullptr, generic_BFMUpdate.getData_exp());

		// for(int i = 0; i < global_num_tex_pcs; i++){
		// 		ceres::CostFunction *cost_function = Regularizer_tex::create(_constraint_bfmManager, i);
		// 		problem.AddResidualBlock(cost_function, nullptr, generic_BFMUpdate.getData_tex());
		// }

	}
};
