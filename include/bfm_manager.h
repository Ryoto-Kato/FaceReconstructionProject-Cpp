﻿// This file is part of BFM Manager (https://github.com/Great-Keith/BFM-tools).


#ifndef BFM_MANAGER_H
#define BFM_MANAGER_H


#include "H5Cpp.h"
#include "data.hpp"
#include "random.hpp"
#include "transform.hpp"
#include "type_utils.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <opencv2/core/eigen.hpp>

#include "glog/logging.h"

#include <string>
#include <fstream>
#include <map>
#include <memory>
#include <array>
#include <stdexcept>
#include <new>
#include <vector>
#include <boost/filesystem.hpp>

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Dynamic;

namespace fs = boost::filesystem;


/*
 * @Class BfmManager
 * 		This class manages related data of Basel Face Model, which may be from official download of personal
 * custom. Users could use the class to read, write or draw face model.
 */

class BfmManager {


public:


	/*
	 * @Function Constructor
	 * @Parameters
	 * 		strModelPath: H5 file storing Basel Face Model;
	 *		aIntParams: Camera intrinsic parameters, array of four double number (fx, fy, cx, cy); 
	 *		strLandmarkIdxPath: File path storing index of landmarks("" means no landmark).
	 */
	
	BfmManager() = default;
	BfmManager(
		const std::string& strModelPath,
		const std::array<double, 4>& aIntParams, 
		const std::string& strLandmarkIdxPath = ""
	);


	/*
	 * @Function genRndFace
	 * 		Generate random face.
	 * @Usage
	 * 		bfmManager.genRndFace(1.0);
	 * 		bfmManager.genRndFace(1.0, 1.0, 1.0);
	 * @Parameters
	 * 		dScale: Scale for shape, texture and expression (default is 0.0);
	 * 		dShapeScale: Scale for shape;
	 * 		dTexScale: Scale for texture;
	 * 		dExprScale: Scale for expression.
	 */

	void genRndFace(double dScale = 0.0);
	void genRndFace(double dShapeScale, double dTexScale, double dExprScale);


	/*
	 * @Function genAvgFace
	 * 		Generate average face. Actually call genRndFace function.
	 * @Usage
	 * 		bfmManager.genAvgFace();
	 */ 

	void genAvgFace() { this->genRndFace(0.0); }


	/*
	 * @Function genFace
	 * 		Generate current face model using current coefficients. The result is saved in current 
	 * shape/tex/expression variables.
	 * @Usage
	 * 		bfmManager.genFace();
	 */

	void genFace();


	/* 
	 * @Function genLandmarkBlendshape
	 * 		Generate current landmark face model using current coefficients. The result is saved in 
	 * current landmark shape/tex/expression variables.
	 * @Usage
	 * 		bfmManager.genLandmarkBlendshape();
	 * 		VectorXd vecLandmarkBlendshape = BfmManager.genLandmarkBlendshape(aShapeCoef, aExprCoef);
	 * @Parameters
	 * 		aShapeCoef: array of shape coefficients;
	 * 		aExprCoef: array of expression coefficients.
	 * @Return:
	 * 		Landmark blendshape generated by shape and expression coefficients.
	 */

	void genLandmarkBlendshape();
	template<typename _Tp>
	Matrix<_Tp, Dynamic, 1> genLandmarkBlendshape(const _Tp* const aShapeCoef,const _Tp* const aExprCoef) const 
	{
		return this->coef2Object(aShapeCoef, m_vecLandmarkShapeMu, m_matLandmarkShapePc, m_vecShapeEv, m_nIdPcs) +
			this->coef2Object(aExprCoef, m_vecLandmarkExprMu, m_matLandmarkExprPc, m_vecExprEv, m_nExprPcs);
	}


	/*
     * @Function genLandmarkBlendshapeByShape
	 * @Function genLandmarkBlendshapeByExpr
	 * 		Generate landmark blendshape by shape or expression.
	 * @Usage
	 * 		VectorXd vecLandmarkBlendshape0 = BfmManager.genLandmarkBlendshapeByShape(aShapeCoef);
	 * 		VectorXd vecLandmarkBlendshape1 = BfmManager.genLandmarkBlendshapeByExpr(aExprCoef);
	 * @Parameters
	 * 		aShapeCoef: array of shape coefficients;
	 * 		aExprCoef: array of expression coefficients.
     */

	template<typename _Tp>
	Matrix<_Tp, Dynamic, 1> genLandmarkBlendshapeByShape(const _Tp * const aShapeCoef) const 
	{
		return this->coef2Object(aShapeCoef, m_vecLandmarkShapeMu, m_matLandmarkShapePc, m_vecShapeEv, m_nIdPcs) +
			m_vecLandmarkCurrentExpr.template cast<_Tp>();
	}
	template<typename _Tp>
	Matrix<_Tp, Dynamic, 1> genLandmarkBlendshapeByExpr(const _Tp * const aExprCoef) const 
	{
		return m_vecLandmarkCurrentShape.template cast<_Tp>() + 
			this->coef2Object(aExprCoef, m_vecLandmarkExprMu, m_matLandmarkExprPc, m_vecExprEv, m_nExprPcs);
	}


	/*
	 * @Function genTransMat
	 * @Function genRMat
	 * @Function genTVec
	 * 		Generate rotation matrix and/or translate vector.
	 * @Usage
	 * 		bfmManager.genTransMat();
	 * 		bfmManager.genRMat();
	 * 		bfmManager.genTVec();
	 */

	void genTransMat();
	void genRMat();
	void genTVec();


	/*
	 * @Function genExtParams
	 * 		Generate extrinsic parameters from rotation mat and translation vector.
	 * @Usage
	 * 		bfmManager.genExtParams();
	 */

	void genExtParams();


	/* 
	 * @Function accExtParams
	 * 		Accumulate extrinsic parameters. Actually we accumulate rotation matrix and translation
	 * vector. Hence, after whole accumulation, you should call `genExtParams` by yourselves to make
	 * sure that transform matrix is be updated with extrinsic paramters.
	 * @Usage
	 * 		aExtParams: array of extrinsic parameters.
	 * @Note
	 * 		TODO: Maybe we could use a flag to show which data have been writen and not updated.
	 */

	void accExtParams(double *aExtParams);


	/* @Function writePly
	 * @Function writeLandmarkPly
	 * 		Write face model or landmark into .ply format file.
	 * @Usage
	 * 		bfmManager.writePly();
	 * 		bfmManager.writeLandmarkPly("landmarks.ply");
	 * @Parameters
	 * 		fn: filename to be written;
	 * 		mode:
	 * 			ModelWriteMode_Invalid: Invalid;
	 *			ModelWriteMode_None: Common mode, draw face model;
	 *			ModelWriteMode_PickLandmark: Pick landmarks in face;
	 *			ModelWriteMode_CameraCoord: Transform using extrinsic parameters;
	 *			ModelWriteMode_NoExpr: Don't use expression.
	 * @Note
	 * 		Users could use `&` to combine different modes. 
	 */

	void writePly(std::string fn = "face.ply", long mode = ModelWriteMode_None) const;
	void writeLandmarkPly(std::string fn = "landmarks.ply") const;

	
	/*
	 * @Function clrExtParams
	 * 		Clear extrinsic parameters (set to 0).
	 * @Usage
	 * 		bfmManager.clrExtParams();
	 */

	void clrExtParams();

	/*
	 * @Function GetBFM
	 * 		return mesh given parameters
	 * 		paramter;
	 * 				m_aShapeCoef: Shape, dim = 199,  d_type = double, array (contiguous memory allocation, double, pointer to the first element)
	 * 				m_aTexCoef	: Albedo, dim = 199, d_type = double, array (contiguous memory allocation, double, pointer to the first element)
	 * 				m_aExprCoef : Expression, dim = 100, d_type = double, array (contiguous memory allocation, double, pointer to the first element)
	 * @Parameter (std::vector type)
	 * 		f_name_ply: string, 
	 * 		Coef_shape; std::vector<double>, [199, 1]
	 * 		Coef_Tex:	std::vector<double>, [199, 1]
	 * 		Coef_Exp:	std::vector<double>, [100, 1]
	 * @Usage
	 * 		bfmManager.GetBFM(std::vector<std::double> & Coef_shape, std::vector<std::double> & Coef_Tex, std::vector<std::double> & Coef_exp)
	*/

	void GetBFM(std::string f_name_ply, std::vector<double> & Coef_shape, std::vector<double> & Coef_Tex, std::vector<double> & Coef_exp,  bool average, bool withExp);

	/*
	 * @Function GetPointer2array
	 *		return pointer (double*) to the first element in the allocated memory for given std::vector<double> coef_X
	 * 		
	 * @Parameter (std::vector type)
	 * 		an arbitrary std::vector<double>
	 * 
	 * 
	 * @Usage
	 * 		bfmManage.GetPointer2array(std::vector<double> & _coeffs)
	 * 
	 * 
	*/

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
	/*
	 * @Function rk_Meshwriter
	 *		mesh writer
	 * 		
	 * @Parameter (std::vector type)
	 * 		file name
	 * 		bool withExp
	 * 
	 * @Usage
	 * 		bfmManage.rk_Meshwriter(std::string f_name_ply, bool withExp)
	 * 
	 * 
	*/

	void rk_Meshwriter(std::string f_name_ply, bool withExp);

	void GetShapeComponents(VectorXd & vec_Mu_shape, VectorXd & vec_Ev_Shape, MatrixXd & mat_Pc_Shape);
	void GetTexComponents(VectorXd & vec_Mu_Tex, VectorXd & vec_Ev_Tex, MatrixXd & mat_Pc_Tex);
	void GetExpComponents(VectorXd & vec_Mu_Exp, VectorXd & vec_Ev_Exp, MatrixXd & mat_Pc_Exp);
/*************************************************************************************************************/
/***************************************** Set & Get Functions ***********************************************/
/*************************************************************************************************************/

	inline const std::string& getStrModelPath() const { return m_strModelPath; }

	inline unsigned int getNIdPcs() const { return m_nIdPcs; }
	inline unsigned int getNExprPcs() const { return m_nExprPcs; }
	inline unsigned int getNFaces() const { return m_nFaces; }
	inline unsigned int getNVertices() const { return m_nVertices; }
	inline unsigned int getNLandmarks() const { return m_mapLandmarkIndices.size(); }
	
	inline double *getMutableShapeCoef() { return m_aShapeCoef; }
	inline double *getMutableTexCoef() { return m_aTexCoef; }
	inline double *getMutableExprCoef() { return m_aExprCoef; }
	inline std::array<double, N_EXT_PARAMS>& getMutableExtParams() { return m_aExtParams; } 
	inline std::array<double, N_INT_PARAMS>& getMutableIntParams() { return m_aIntParams; } 

	inline double& getMutableScale() { return m_dSc; }
	inline double getScale() const { return m_dSc; }
	inline const std::array<double, N_EXT_PARAMS>& getExtParams() { return m_aExtParams; } 
	inline const std::array<double, N_INT_PARAMS>& getIntParams() { return m_aIntParams; } 

	inline const Matrix3d &getMatR() const { return m_matR; }
	inline const Vector3d &getVecT() const { return m_vecT; }

	inline double getFx() const { return m_aIntParams[0]; }
	inline double getFy() const { return m_aIntParams[1]; }
	inline double getCx() const { return m_aIntParams[2]; }
	inline double getCy() const { return m_aIntParams[3]; }
	inline double getRoll() const { return m_aExtParams[0]; }
	inline double getYaw() const { return m_aExtParams[1]; }
	inline double getPitch() const { return m_aExtParams[2]; }
	inline double getTx() const { return m_aExtParams[3]; }
	inline double getTy() const { return m_aExtParams[4]; }
	inline double getTz() const { return m_aExtParams[5]; }
	void setRoll(double dRoll)   { m_aExtParams[0] = dRoll;  this->genRMat();}
	void setYaw(double dYaw)     { m_aExtParams[1] = dYaw;   this->genRMat();}
	void setPitch(double dPitch) { m_aExtParams[2] = dPitch; this->genRMat();}
	void setRotation(double dRoll, double dYaw, double dPitch) 
	{
		m_aExtParams[0] = dRoll;
		m_aExtParams[1] = dYaw;
		m_aExtParams[2] = dPitch; 
		this->genRMat();
	}
	inline void setTx(double tx) { m_aExtParams[3] = tx; m_vecT(0) = tx; }
	inline void setTy(double ty) { m_aExtParams[4] = ty; m_vecT(1) = ty; }
	inline void setTz(double tz) { m_aExtParams[5] = tz; m_vecT(2) = tz; }
	inline void setMatR(const Matrix3d &r_mat) { m_matR = r_mat; }
	inline void setMatR(const cv::Mat &r_mat) { cv::cv2eigen(r_mat, m_matR); }
	inline void setMatR(CvMat *r_mat)
	{
		m_matR(0, 0) = cvmGet(r_mat, 0, 0); m_matR(0, 1) = cvmGet(r_mat, 0, 1); m_matR(0, 2) = cvmGet(r_mat, 0, 2);
		m_matR(1, 0) = cvmGet(r_mat, 1, 0); m_matR(1, 1) = cvmGet(r_mat, 1, 1); m_matR(1, 2) = cvmGet(r_mat, 1, 2);
		m_matR(2, 0) = cvmGet(r_mat, 2, 0); m_matR(2, 1) = cvmGet(r_mat, 2, 1); m_matR(2, 2) = cvmGet(r_mat, 2, 2);		
	}
	inline void setVecT(const Vector3d &t_vec) { m_vecT = t_vec; }
	inline void setVecT(const cv::Mat &t_vec) { cv::cv2eigen(t_vec, m_vecT); }
	inline void setVecT(CvMat *t_vec)
	{
		m_vecT(0) = cvmGet(t_vec, 0, 0);
		m_vecT(1) = cvmGet(t_vec, 1, 0);
		m_vecT(2) = cvmGet(t_vec, 2, 0);		
	}

	inline const VectorXd &getCurrentShape() const { return m_vecCurrentShape; }
	inline const VectorXd &getCurrentTex() const { return m_vecCurrentTex; }
	inline VectorXd getStdTex() const 
	{
		if(m_bIsTexStd)
			return m_vecTexMu;
		VectorXd res(m_nVertices * 3);
		for(auto i = 0; i < res.size(); i++)
			res(i) = m_vecTexMu(i) / 255.0;
		return res;
	} 
	inline const VectorXd &getCurrentExpr() const { return m_vecCurrentExpr; }
	inline const VectorXd &getCurrentBlendshape() const { return m_vecCurrentBlendshape; }
	inline const VectorXd &getLandmarkCurrentBlendshape() const { return m_vecLandmarkCurrentBlendshape; }
	VectorXd getLandmarkCurrentBlendshapeTransformed() const { return bfm_utils::TransPoints(m_matR, m_vecT, m_vecLandmarkCurrentBlendshape); }
	VectorXd getCurrentBlendshapeTransformed() const { return bfm_utils::TransPoints(m_matR, m_vecT, m_vecCurrentBlendshape); }
	inline const Matrix<int, Dynamic, 1> &getTriangleList() const { return m_vecTriangleList; }
	inline const std::vector<std::pair<int, int>>& getMapLandmarkIndices() const { return m_mapLandmarkIndices; }
	inline const VectorXd &get_ShapeMu() const {return m_vecShapeMu;}
	inline const VectorXd &get_ShapeEv() const {return m_vecShapeEv;}
	inline const MatrixXd &get_ShapePc() const {return m_matShapePc;}
	inline const VectorXd &get_TexMu() const {return m_vecTexMu;}
	inline const VectorXd &get_TexEv() const {return m_vecTexEv;}
	inline const MatrixXd &get_TexPc() const {return m_matTexPc;}
	inline const VectorXd &get_ExprMu() const {return m_vecExprMu;}
	inline const VectorXd &get_ExprEv() const {return m_vecExprEv;}
	inline const MatrixXd &get_ExprPc() const {return m_matExprPc;}

	void update_ShapeMu(Eigen::Matrix4f & SE3){
		unsigned int num_vertex = getNVertices();
		for(unsigned int i = 0; i < num_vertex; i++){
			Eigen::Vector4f _point = {(float)m_vecShapeMu(3*i), (float)m_vecShapeMu(3*i+1), (float)m_vecShapeMu(3*i+2), 1.f};
			Eigen::Vector4f _transformed_point = SE3 * _point; 
			m_vecShapeMu(3*i) = (double)_transformed_point.x();
			m_vecShapeMu(3*i+1) = (double)_transformed_point.y();
			m_vecShapeMu(3*i+2) = (double)_transformed_point.z();
		}
	}

	void update_ExprMu(Eigen::Matrix4f & SE3){
		unsigned int num_vertex = getNVertices();
		for(unsigned int i = 0; i < num_vertex; i++){
			Eigen::Vector4f _point = {(float)m_vecExprMu(3*i), (float)m_vecExprMu(3*i+1), (float)m_vecExprMu(3*i+2), 1.f};
			Eigen::Vector4f _transformed_point = SE3 * _point; 
			m_vecExprMu(3*i) = _transformed_point.x();
			m_vecExprMu(3*i+1) = _transformed_point.y();
			m_vecExprMu(3*i+2) = _transformed_point.z();
		}
	}
/*************************************************************************************************************/
/************************************** Print Function (for Debug) *******************************************/
/*************************************************************************************************************/


	/*
	 * @Function check
	 * 		Show your loaded data and reference data. Users could use thi 
	 * function to check whether loading is successful.
	 * @Usage
	 * 		bfmManager.check();
	 */

	inline void check() const
	{
		LOG(INFO) << "Check data.";
		LOG(INFO) << "------------------------------------------------------------------------";
		LOG(INFO) << "| Data\t\t\t\t\t| Reference\t| Yours";
		LOG(INFO) << "------------------------------------------------------------------------";
		LOG(INFO) << "| Shape\t\t| Average\t\t| -57239\t| " << m_vecShapeMu(0);
		LOG(INFO) << "| \t\t| Variance\t\t| 884340\t| " << m_vecShapeEv(0);
		LOG(INFO) << "| \t\t| Principle component\t| -0.0024\t| " << m_matShapePc(0, 0);
		LOG(INFO) << "------------------------------------------------------------------------";
		LOG(INFO) << "| Texture\t| Average\t\t| 182.8750\t| " << m_vecTexMu(0);
		LOG(INFO) << "| \t\t| Variance\t\t| 4103.2\t| " << m_vecTexEv(0);
		LOG(INFO) << "| \t\t| Principle component\t| -0.0028\t| " << m_matTexPc(0, 0);
		LOG(INFO) << "------------------------------------------------------------------------";
		LOG(INFO) << "| Expression\t| Average\t\t| 182.875\t| " << m_vecExprMu(0);
		LOG(INFO) << "| \t\t| Variance\t\t| 4103.2\t| " << m_vecExprEv(0);
		LOG(INFO) << "| \t\t| Principle component\t| -0.0028\t| " << m_matExprPc(0, 0);
		LOG(INFO) << "------------------------------------------------------------------------";
		LOG(INFO) << "| Triangle list\t\t\t\t| 1\t\t| " << m_vecTriangleList(0);
		LOG(INFO) << "------------------------------------------------------------------------";
	}


	inline void printExtParams() const 
	{
		LOG(INFO) << "Rotation\t\tRoll:\t" << m_aExtParams[0] << " ( " << m_aExtParams[0] * 180.0 / M_PI << " )";
		LOG(INFO) << "\t\t\tYaw:\t" << m_aExtParams[1] << " ( " << m_aExtParams[1] * 180.0 / M_PI << " )";
		LOG(INFO) << "\t\t\tPitch:\t" << m_aExtParams[2] << " (" << m_aExtParams[2] * 180.0 / M_PI << " )";
		LOG(INFO) << "Translation\tTx:\t" << m_aExtParams[3];
		LOG(INFO) << "\t\t\tTy:\t" << m_aExtParams[4];
		LOG(INFO) << "\t\t\tTz:\t" << m_aExtParams[5];
	}


	inline void printIntParams() const
	{
		LOG(INFO) << "Fx:\t" << m_aIntParams[0];
		LOG(INFO) << "Fy:\t" << m_aIntParams[1];
		LOG(INFO) << "Cx:\t" << m_aIntParams[2];
		LOG(INFO) << "Cy:\t" << m_aIntParams[3];
	}
	

	void printShapeCoefTopK(unsigned short nK) const 
	{ 
		LOG(INFO) << "Top k of shape coefficients:";

		auto&& log = COMPACT_GOOGLE_LOG_INFO;
		for(auto i = 0; i < nK; ++i)
			log.stream() << " " << m_aShapeCoef[i];
		log.stream() << "\n";
	}	
	

	void printExprCoefTopK(unsigned short nK) const 
	{ 
		LOG(INFO) << "Top k of expression coefficients:\n";
	
		auto&& log = COMPACT_GOOGLE_LOG_INFO;
		for(auto i = 0; i < nK; ++i)
			log.stream() << " " << m_aExprCoef[i];
		log.stream() << "\n";
	}


	void printRMat() const 
	{ 
		LOG(INFO) << m_matR;
	}


	void printTVec() const 
	{
		LOG(INFO) << m_vecT;
	}


private:


	/*
	 * @Function alloc
	 * @Function load
	 * 		Allocate memory and load data. `alloc` must be used before `load`.
	 * @Usage
	 * 		bfmManager.alloc();
	 * 		bfmManager.load();
	 */

	void alloc();
	bool load();


	/*
	 * @Function extractLandmarks
	 * 		Extract landmarks and save in specific variables. This function must be used after `alloc` 
	 * and `load`. And it should not be used if landmark information are not be detected.
	 * @Usage
	 * 		bfmManager.extractLandmarks();
	 */

	void extractLandmarks();


	/*
	 * @Function coef2Object
	 * 		Use coefficients combined with shape/tex/expression average, variance and principle component
	 * to generate result (using PCA).
	 * @Usage
	 * 		VectorXd shape = bfmManager.coef2Object(
	 * 							bfmManager.m_aShapeCoef, 
	 * 							bfmManager.m_vecShapeMu,
	 * 							bfmManager.m_matShapePc,
	 * 							bfmManager.m_vecShapeEv,
	 * 							bfmManager.m_nIdPcs);
	 * @Parameters
	 * 		aCoef: array of coefficients;
	 * 		vecMu: vector of average;
	 * 		matPc: matrix of principle component;
	 * 		vecEv: vector of variance;
	 * 		nLength: length of coefficients.
	 */
	
	template<typename Derived>
	Matrix<Derived, Dynamic, 1> coef2Object(const Derived *const aCoef, 
		const VectorXd &vecMu, const MatrixXd &matPc, const VectorXd &vecEv, unsigned int nLength) const
	{
		assert(aCoef != nullptr);
		// assert(nLength >= 0);

		Matrix<Derived, Dynamic, 1> tmpCoef(nLength);
		for(auto i = 0u; i < nLength; i++)
			tmpCoef(i) = aCoef[i];

		Matrix<Derived, Dynamic, 1> tmpMu = vecMu.cast<Derived>();
		Matrix<Derived, Dynamic, 1> tmpEv = vecEv.cast<Derived>();
		Matrix<Derived, Dynamic, Dynamic> tmpPc = matPc.cast<Derived>();
		// return tmpMu + tmpPc * tmpCoef.cwiseProduct(tmpEv);
		return tmpMu + tmpPc * tmpCoef;
	}


	std::string m_strVersion;

	// file path
	std::string m_strModelPath;
	std::string m_strLandmarkIdxPath;

	// H5 dataset path
	std::string m_strShapeMuH5Path;
	std::string m_strShapeEvH5Path;
	std::string m_strShapePcH5Path;
	std::string m_strTexMuH5Path;
	std::string m_strTexEvH5Path;
	std::string m_strTexPcH5Path;
	std::string m_strExprMuH5Path;
	std::string m_strExprEvH5Path;
	std::string m_strExprPcH5Path;
	std::string m_strTriangleListH5Path;

	unsigned int m_nVertices;
	unsigned int m_nFaces;
	unsigned int m_nIdPcs;
	unsigned int m_nExprPcs;

	// Z1Y2X3
	// Roll rotates around z axis
	// Yaw rotates around y axis
    // Pitch rotates around x axis 
	Eigen::Matrix3d m_matR;
	Eigen::Vector3d m_vecT;
	double m_dSc = 0.0075;
	std::array<double, N_EXT_PARAMS> m_aExtParams = { }; /* roll yaw pitch tx ty tz (initialized as 0)*/
	std::array<double, N_INT_PARAMS> m_aIntParams = { }; /* fx fy cx cy (initialized as 0)*/

	double *m_aShapeCoef;
	Eigen::VectorXd m_vecShapeMu;
	Eigen::VectorXd m_vecShapeEv;
	Eigen::MatrixXd m_matShapePc;

	bool m_bIsTexStd = true;
	double *m_aTexCoef;
	Eigen::VectorXd m_vecTexMu;
	Eigen::VectorXd m_vecTexEv;
	Eigen::MatrixXd m_matTexPc;

	double *m_aExprCoef;
	Eigen::VectorXd m_vecExprMu;
	Eigen::VectorXd m_vecExprEv;
	Eigen::MatrixXd m_matExprPc;

	Eigen::Matrix<int, Eigen::Dynamic, 1> m_vecTriangleList;	

	Eigen::VectorXd m_vecCurrentShape;
	Eigen::VectorXd m_vecCurrentTex;
	Eigen::VectorXd m_vecCurrentExpr;
	Eigen::VectorXd m_vecCurrentBlendshape;

	// landmarks
	bool m_bUseLandmark;
	std::vector<std::pair<int, int>> m_mapLandmarkIndices; 
	Eigen::VectorXd m_vecLandmarkShapeMu;
	Eigen::MatrixXd m_matLandmarkShapePc;
	Eigen::VectorXd m_vecLandmarkExprMu;
	Eigen::MatrixXd m_matLandmarkExprPc;
	Eigen::VectorXd m_vecLandmarkCurrentShape;
	Eigen::VectorXd m_vecLandmarkCurrentExpr;
	Eigen::VectorXd m_vecLandmarkCurrentBlendshape;

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


#endif // BFM_MANAGER_H