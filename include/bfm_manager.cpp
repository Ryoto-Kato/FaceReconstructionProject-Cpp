#include "bfm_manager.h"
#include <tuple>
#include "Eigen.h"


BfmManager::BfmManager(
	const std::string& strModelPath,
	const std::array<double, 4>& aIntParams, 
	const std::string& strLandmarkIdxPath) :
	m_strModelPath(strModelPath),
	m_aIntParams(aIntParams),
	m_strLandmarkIdxPath(strLandmarkIdxPath)
{
	if(!fs::exists(strModelPath))
	{
		LOG(WARNING) << "Path of Basel Face Model does not exist. Unexpected path:\t" << strModelPath;
		m_strModelPath = "";
		return;
	}
	else
	{
		fs::path modelPath(strModelPath);
		std::string strFn, strExt;
		
		strExt = modelPath.extension().string();
		if(strExt != ".h5")
		{
			LOG(ERROR) << "Data type must be hdf5. Unexpected tyoe: " << strExt;
		}
		else
		{
			strFn = modelPath.stem().string();
			if(strFn == "model2009-publicmm1-bfm")
			{
				LOG(WARNING) << "BFM 2009 does not contain expression.";

				m_strVersion = "2009";
				m_nVertices = 53490,
				m_nFaces = 106333,
				m_nIdPcs = 199,
				m_nExprPcs = 0,
				m_strShapeMuH5Path = R"(shape/model/mean)";
				m_strShapeEvH5Path = R"(shape/model/pcaVariance)";
				m_strShapePcH5Path = R"(shape/model/pcaBasis)";
				m_strTexMuH5Path = R"(color/model/mean)";
				m_strTexEvH5Path = R"(color/model/pcaVariance)";
				m_strTexPcH5Path = R"(color/model/pcaBasis)";
				m_strExprMuH5Path = "";
				m_strExprEvH5Path = "";
				m_strExprPcH5Path = "";
				m_strTriangleListH5Path = R"(representer/cells)";
			}
			else if(strFn == "model2017-1_bfm_nomouth")
			{
				m_strVersion = "2017";
				m_nVertices = 53149,
				m_nFaces = 105694,
				m_nIdPcs = 199,
				m_nExprPcs = 100,
				m_strShapeMuH5Path = R"(/shape/model/mean)";
				m_strShapeEvH5Path = R"(/shape/model/pcaVariance)";
				m_strShapePcH5Path = R"(/shape/model/pcaBasis)";
				m_strTexMuH5Path = R"(/color/model/mean)";
				m_strTexEvH5Path = R"(/color/model/pcaVariance)";
				m_strTexPcH5Path = R"(/color/model/pcaBasis)";
				m_strExprMuH5Path = R"(/expression/model/mean)";
				m_strExprEvH5Path = R"(/expression/model/pcaVariance)";
				m_strExprPcH5Path = R"(/expression/model/pcaBasis)";
				m_strTriangleListH5Path = R"(/shape/representer/cells)";
			}
			else if(strFn == "model2017-1_face12_nomouth")
			{
				m_strVersion = "2017-face12";
				m_nVertices = 28588,
				m_nFaces = 56572,
				m_nIdPcs = 199,
				m_nExprPcs = 100,
				m_strShapeMuH5Path = R"(shape/model/mean)";
				m_strShapeEvH5Path = R"(shape/model/pcaVariance)";
				m_strShapePcH5Path = R"(shape/model/pcaBasis)";
				m_strTexMuH5Path = R"(color/model/mean)";
				m_strTexEvH5Path = R"(color/model/pcaVariance)";
				m_strTexPcH5Path = R"(color/model/pcaBasis)";
				m_strExprMuH5Path = R"(expression/model/mean)";
				m_strExprEvH5Path = R"(expression/model/pcaVariance)";
				m_strExprPcH5Path = R"(expression/model/pcaBasis)";
				m_strTriangleListH5Path = R"(representer/cells)";
			}
			else if(strFn == "model2019_bfm")
			{
				m_strVersion = "2019";
				m_nVertices = 47439,
				m_nFaces = 94464,
				m_nIdPcs = 199,
				m_nExprPcs = 100,
				m_strShapeMuH5Path = R"(shape/model/mean)";
				m_strShapeEvH5Path = R"(shape/model/pcaVariance)";
				m_strShapePcH5Path = R"(shape/model/pcaBasis)";
				m_strTexMuH5Path = R"(color/model/mean)";
				m_strTexEvH5Path = R"(color/model/pcaVariance)";
				m_strTexPcH5Path = R"(color/model/pcaBasis)";
				m_strExprMuH5Path = R"(expression/model/mean)";
				m_strExprEvH5Path = R"(expression/model/pcaVariance)";
				m_strExprPcH5Path = R"(expression/model/pcaBasis)";
				m_strTriangleListH5Path = R"(representer/cells)";
			}
			else
			{
				LOG(WARNING) << "Load an undefined BFM model.";

				// Custom (Update by yourself)
				m_strVersion = "Others";
				m_nVertices = 46990,
				m_nFaces = 93322,
				m_nIdPcs = 99,
				m_nExprPcs = 29,
				m_strShapeMuH5Path = "shapeMU";
				m_strShapeEvH5Path = "shapeEV";
				m_strShapePcH5Path = "shapePC";
				m_strTexMuH5Path = "texMU";
				m_strTexEvH5Path = "texEV";
				m_strTexPcH5Path = "texPC";
				m_strExprMuH5Path = "expMU";
				m_strExprEvH5Path = "expEV";
				m_strExprPcH5Path = "expPC";
				m_strTriangleListH5Path = "faces";
				// end of custom
			}
		}
	}
	

	for(unsigned int iParam = 0; iParam < 4; iParam++)
		m_aIntParams[iParam] = aIntParams[iParam];

	m_bUseLandmark = strLandmarkIdxPath == "" ? false : true;

	// if(m_bUseLandmark){
	// 	std::cout<<"Use Landmark"<<std::endl;
	// }else{
	// 	std::cout<<"Does not use Landmark"<<std::endl;
	// }

	if(m_bUseLandmark)
	{
		std::ifstream inFile;
		inFile.open(strLandmarkIdxPath, std::ios::in);
		assert(inFile.is_open());
		int dlibIdx, bfmIdx;
		while(inFile >> dlibIdx >> bfmIdx)
		{
			// dlibIdx--;
			// If we do not have correspondence, the bfm vertex id is -1
			if(bfmIdx>0){
			m_mapLandmarkIndices.push_back(std::make_pair(dlibIdx - 1, std::move(bfmIdx)));
			}
		}
		inFile.close();
	}
	
	// for(const auto& [dlibIdx, bfmIdx] : m_mapLandmarkIndices){
		
	// 	std::cout<<dlibIdx<<" "<<bfmIdx<<std::endl;	
	// }

	this->alloc();
	this->load();	
	this->extractLandmarks();

	// std::cout<<m_vecTexMu<<std::endl;

	// unsigned int iTex = 0;
	// while(m_bIsTexStd)
	// {
	// 	if(m_vecTexMu(iTex++) > 1.0)
	// 		m_bIsTexStd = false;
	// }

	LOG(INFO) << "Infomation load done.\n";
	cout<< "Infomation load done.\n";

	LOG(INFO) << "*******************************************";
	LOG(INFO) << "*********** Load Basel Face Model *********";
	LOG(INFO) << "*******************************************";
	LOG(INFO) << "Version:\t\t\t\t" << m_strVersion;
	LOG(INFO) << "Number of vertices:\t\t\t" << m_nVertices;
	LOG(INFO) << "Number of faces:\t\t\t" << m_nFaces;
	LOG(INFO) << "Number of shape PCs:\t\t\t" << m_nIdPcs;
	LOG(INFO) << "Number of texture PCs:\t\t\t" << m_nIdPcs;
	if(m_strVersion == "2009")
		LOG(INFO) << "Number of expression PCs:\t\tNone";
	else
		LOG(INFO) << "Number of expression PCs:\t\t" << m_nExprPcs;
	if(m_bIsTexStd)
		LOG(INFO) << "Texture range:\t\t\t\t0.0~1.0";
	else
		LOG(INFO) << "Texture range:\t\t\t\t0~255";
	LOG(INFO) << "Number of dlib landmarks:\t\t68";
	if(m_bUseLandmark)
	{
		LOG(INFO) << "Number of custom landmarks:\t\t" << m_mapLandmarkIndices.size();
		LOG(INFO) << "Corresponding between dlib and custom:\t" << m_strLandmarkIdxPath;
	}
	else
		LOG(INFO) << "Number of custom landmarks:\tNone";
	LOG(INFO) << "Camera intrinsic parameters (fx, fy, cx, cy):";
	LOG(INFO) << "\t" << m_aIntParams[0] << "\t" << m_aIntParams[1]
			  << "\t" << m_aIntParams[2] << "\t" << m_aIntParams[3];
	LOG(INFO) << "\n";

	this->genAvgFace();
	this->genLandmarkBlendshape();
}


void BfmManager::alloc() 
{
	LOG(INFO) << "Allocate memory for model.";
	std::cout<<"Shape Coeff Dim: "<<m_nIdPcs<<std::endl;
	m_aShapeCoef = new double[m_nIdPcs];
	std::cout<<"	m_aShapeCoef: "<<m_aExtParams[m_nIdPcs-1]<<std::endl;
	std::fill(m_aShapeCoef, m_aShapeCoef + m_nIdPcs, 0.0);
	m_vecShapeMu.resize(m_nVertices * 3);
	m_vecShapeEv.resize(m_nIdPcs);
	m_matShapePc.resize(m_nVertices * 3, m_nIdPcs);

	m_aTexCoef = new double[m_nIdPcs];
	std::fill(m_aTexCoef, m_aTexCoef + m_nIdPcs, 0.0);
	m_vecTexMu.resize(m_nVertices * 3);
	m_vecTexEv.resize(m_nIdPcs);
	m_matTexPc.resize(m_nVertices * 3, m_nIdPcs);

	m_aExprCoef = new double[m_nExprPcs];
	std::fill(m_aExprCoef, m_aExprCoef + m_nExprPcs, 0.0);
	m_vecExprMu.resize(m_nVertices * 3);
	m_vecExprEv.resize(m_nExprPcs);
	m_matExprPc.resize(m_nVertices * 3, m_nExprPcs);

	m_vecTriangleList.resize(m_nFaces * 3);

	m_vecCurrentShape.resize(m_nVertices * 3);
	m_vecCurrentTex.resize(m_nVertices * 3);
	m_vecCurrentExpr.resize(m_nVertices * 3);
	m_vecCurrentBlendshape.resize(m_nVertices * 3);

	auto nLandmarks = m_mapLandmarkIndices.size();
	std::cout<<"Number of the landmark: "<<nLandmarks<<std::endl;
	if (m_bUseLandmark) 
	{
		m_vecLandmarkShapeMu.resize(nLandmarks * 3);
		m_matLandmarkShapePc.resize(nLandmarks * 3, m_nIdPcs);
		m_vecLandmarkExprMu.resize(nLandmarks * 3);
		m_matLandmarkExprPc.resize(nLandmarks * 3, m_nExprPcs);
	}
}

// template<class T>
// void print_loaded_data(std::vector<vector<T>> & data){
// 	for(unsigned int i = 0; i<data.rows(); i++){
// 		for(unsigned int j = 0; j<data.cols(); j++){
// 			std::cout<<data[i][j]<<",";
// 		}
// 		std::cout<<std::endl;
// 	}
// }


bool BfmManager::load() 
{
	LOG(INFO) << "Load model from disk.";

	try
	{
		std::unique_ptr<float[]> vecShapeMu(new float[m_nVertices * 3]);
		std::unique_ptr<float[]> vecShapeEv(new float[m_nIdPcs]);
		std::unique_ptr<float[]> matShapePc(new float[m_nVertices * 3 * m_nIdPcs]);
		std::unique_ptr<float[]> vecTexMu(new float[m_nVertices * 3]);
		std::unique_ptr<float[]> vecTexEv(new float[m_nIdPcs]);
		std::unique_ptr<float[]> matTexPc(new float[m_nVertices * 3 * m_nIdPcs]);
		std::unique_ptr<float[]> vecExprMu(new float[m_nVertices * 3]);
		std::unique_ptr<float[]> vecExprEv(new float[m_nExprPcs]);
		std::unique_ptr<float[]> matExprPc(new float[m_nVertices * 3 * m_nExprPcs]);
		std::unique_ptr<int[]> vecTriangleList(new int[m_nFaces * 3]);

		hid_t file = H5Fopen(m_strModelPath.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
		std::cout<<"Open H5 file"<<std::endl;

		// std::cout<<m_strShapeMuH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strShapeMuH5Path, vecShapeMu, m_vecShapeMu, H5T_NATIVE_FLOAT);

		// std::cout<<m_strShapeEvH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strShapeEvH5Path, vecShapeEv, m_vecShapeEv, H5T_NATIVE_FLOAT);
		// std::cout<<m_strShapePcH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strShapePcH5Path, matShapePc, m_matShapePc, H5T_NATIVE_FLOAT);

		// std::cout<<m_strTexMuH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strTexMuH5Path, vecTexMu, m_vecTexMu, H5T_NATIVE_FLOAT);
		// std::cout<<m_strTexEvH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strTexEvH5Path, vecTexEv, m_vecTexEv, H5T_NATIVE_FLOAT);
		// std::cout<<m_strTexPcH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strTexPcH5Path, matTexPc, m_matTexPc, H5T_NATIVE_FLOAT);
		
		// std::cout<<m_strExprMuH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strExprMuH5Path, vecExprMu, m_vecExprMu, H5T_NATIVE_FLOAT);
		// std::cout<<m_strExprEvH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strExprEvH5Path, vecExprEv, m_vecExprEv, H5T_NATIVE_FLOAT);
		// std::cout<<m_strExprPcH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strExprPcH5Path, matExprPc, m_matExprPc, H5T_NATIVE_FLOAT);
		
		// std::cout<<m_strTriangleListH5Path<<std::endl;
		bfm_utils::LoadH5Model(file, m_strTriangleListH5Path, vecTriangleList, m_vecTriangleList, H5T_STD_I32LE);
		// std::cout<<"Read dataset from HDF5 (triangle list)"<<std::endl;
		// std::cout<<m_vecTriangleList<<std::endl; //shape 3xm_nFaces, 1

		//Rearrange the order for the verticies set which has triangle
		Eigen::MatrixXi _copy_m_vecTriangleList = Eigen::MatrixXi::Zero(3, m_nFaces);

		for(unsigned int i=0; i<3; i++){
			for(unsigned int _row = 0; _row<m_nFaces; _row++){
				_copy_m_vecTriangleList(i, _row) = m_vecTriangleList(m_nFaces*i+_row);
			}
		}

		// std::cout<<"Copy of the m_vecTriangleList"<<std::endl;
		// std::cout<<_copy_m_vecTriangleList<<std::endl;

		//Store vertex id for a triangle in order which is same with vertex coordinate
		
		for(unsigned int c = 0; c<_copy_m_vecTriangleList.cols(); c++){
			for(unsigned int r = 0; r<_copy_m_vecTriangleList.rows(); r++){
				m_vecTriangleList(3*c+r) = _copy_m_vecTriangleList(r, c);
			}
		}

		// std::cout<<"Final m_vecTriangleList"<<std::endl;
		// std::cout<<m_vecTriangleList<<std::endl;
	}
	catch(std::bad_alloc& ba)
	{
		LOG(ERROR) << "Failed to alloc";
		return false;
	}
	
	std::cout<<"Finish data"<<std::endl;
	return true;
}



void BfmManager::extractLandmarks() 
{
	unsigned int iLandmark = 0;
	for(const auto& [dlibIdx, bfmIdx] : m_mapLandmarkIndices) 
	{
		m_vecLandmarkShapeMu(iLandmark * 3) = m_vecShapeMu(bfmIdx * 3);
		m_vecLandmarkShapeMu(iLandmark * 3 + 1) = m_vecShapeMu(bfmIdx * 3 + 1);
		m_vecLandmarkShapeMu(iLandmark * 3 + 2) = m_vecShapeMu(bfmIdx * 3 + 2);
		m_vecLandmarkExprMu(iLandmark * 3) = m_vecExprMu(bfmIdx * 3);
		m_vecLandmarkExprMu(iLandmark * 3 + 1) = m_vecExprMu(bfmIdx * 3 + 1);
		m_vecLandmarkExprMu(iLandmark * 3 + 2) = m_vecExprMu(bfmIdx * 3 + 2);

		for(unsigned int iIdPc = 0; iIdPc < m_nIdPcs; iIdPc++) 
		{
			m_matLandmarkShapePc(iLandmark * 3, iIdPc) = m_matShapePc(bfmIdx * 3, iIdPc);
			m_matLandmarkShapePc(iLandmark * 3 + 1, iIdPc) = m_matShapePc(bfmIdx * 3 + 1, iIdPc);
			m_matLandmarkShapePc(iLandmark * 3 + 2, iIdPc) = m_matShapePc(bfmIdx * 3 + 2, iIdPc);	
		}

		for(unsigned int iExprPc = 0; iExprPc < m_nExprPcs; iExprPc++) 
		{
			m_matLandmarkExprPc(iLandmark * 3, iExprPc) = m_matExprPc(bfmIdx * 3, iExprPc);
			m_matLandmarkExprPc(iLandmark * 3 + 1, iExprPc) = m_matExprPc(bfmIdx * 3 + 1, iExprPc);
			m_matLandmarkExprPc(iLandmark * 3 + 2, iExprPc) = m_matExprPc(bfmIdx * 3 + 2, iExprPc);
		}

		++iLandmark;
		// std::cout<<"Counter: "<<iLandmark<<std::endl;
	}

	// std::cout<<m_vecLandmarkShapeMu<<std::endl;
	// std::cout<<m_vecLandmarkExprMu<<std::endl;
	// std::cout<<m_matLandmarkExprPc<<std::endl;
}




void BfmManager::genRndFace(double dScale) 
{
	if(dScale == 0.0)
		LOG(INFO) << "Generate average face";
	else
		LOG(INFO) << "Generate random face (using the same scale)";

	/*
	m_nIdPcs = Identity principal components = 199
	m_aShapeCoef(199) 
	m_aTexCoef(199)
	m_aExprCoef(100)

	bfm_utils::randn will provide you uniformly sampled coefficients
	genAvgFace -> genRndFace(0.0)
		In this case, coefficient equals to the mean (mu)
	*/
	
	m_aShapeCoef = bfm_utils::randn(m_nIdPcs, dScale); 
	m_aTexCoef   = bfm_utils::randn(m_nIdPcs, dScale);
	if(m_strVersion != "2009")
		m_aExprCoef  = bfm_utils::randn(m_nExprPcs, dScale);

	this->genFace();
}


void BfmManager::genRndFace(double dShapeScale, double dTexScale, double dExprScale) 
{
	LOG(INFO) << "Generate random face (using different scales)";
	m_aShapeCoef = bfm_utils::randn(m_nIdPcs, dShapeScale);
	m_aTexCoef   = bfm_utils::randn(m_nIdPcs, dTexScale);
	if(m_strVersion != "2009")
		m_aExprCoef  = bfm_utils::randn(m_nExprPcs, dExprScale);

	this->genFace();
}


// void Vector2Mat(const Eigen::Derived *const _coeff, MatrixXd & mat_3d, const unsigned int max_num){
// 	long unsigned int row_idx=0;
// 	while(row_idx<max_num){
// 		for(unsigned int col = 0; col<3; col++){
// 			mat_3d(row_idx, col) = _coeff(row_idx * 3 + col);
// 		}
// 		++row_idx;
// 	}

// 	std::cout<<"Vector->Mat"<<std::endl;
// 	std::cout<<mat_3d<<std::endl;
// }

int inline float2color(const float & _f_color){
	return ((int)(_f_color*255));
}


void BfmManager::genFace() 
{
	LOG(INFO) <<"Generate face with shape and expression coefficients";

	

	m_vecCurrentShape = this->coef2Object(m_aShapeCoef, m_vecShapeMu, m_matShapePc, m_vecShapeEv, m_nIdPcs);
	
	// std::cout<<"m_vecCurrentShape"<<std::endl;
	// std::cout<<m_vecCurrentShape<<std::endl;
	m_vecCurrentTex   = this->coef2Object(m_aTexCoef, m_vecTexMu, m_matTexPc, m_vecTexEv, m_nIdPcs);
	// std::cout<<"m_vecCurrentTex"<<std::endl;
	// std::cout<<m_vecCurrentTex<<std::endl;
	if(m_strVersion != "2009")
	{
		m_vecCurrentExpr  = this->coef2Object(m_aExprCoef, m_vecExprMu, m_matExprPc, m_vecExprEv, m_nExprPcs);
		m_vecCurrentBlendshape = m_vecCurrentShape + m_vecCurrentExpr;		
	}
	else
		m_vecCurrentBlendshape = m_vecCurrentShape;
}


void BfmManager::genLandmarkBlendshape()  
{
	LOG(INFO) <<"Generate landmarks with shape and expression coefficients";

	m_vecLandmarkCurrentShape = this->coef2Object(m_aShapeCoef, m_vecLandmarkShapeMu, m_matLandmarkShapePc, m_vecShapeEv, m_nIdPcs);
	if(m_strVersion != "2009")
	{
		m_vecLandmarkCurrentExpr = this->coef2Object(m_aExprCoef, m_vecLandmarkExprMu, m_matLandmarkExprPc, m_vecExprEv, m_nExprPcs);
		m_vecLandmarkCurrentBlendshape = m_vecLandmarkCurrentShape + m_vecLandmarkCurrentExpr;
	}
	else
		m_vecLandmarkCurrentBlendshape = m_vecLandmarkCurrentShape;
}


void BfmManager::genRMat() 
{
	LOG(INFO) <<"Generate rotation matrix.";

	const double &roll   = m_aExtParams[0];
	const double &yaw    = m_aExtParams[1];
	const double &pitch  = m_aExtParams[2];
	m_matR = bfm_utils::Euler2Mat(roll, yaw, pitch, false);
}


void BfmManager::genTVec()
{
	LOG(INFO) <<"Generate translation vector.";	

	const double &tx = m_aExtParams[3];
	const double &ty = m_aExtParams[4];
	const double &tz = m_aExtParams[5];
	m_vecT << tx, ty, tz;	
}


void BfmManager::genTransMat()
{
	this->genRMat();
	this->genTVec();
}


void BfmManager::genExtParams()
{
	LOG(INFO) <<"Generate external paramter.";

	if(!bfm_utils::IsRMat(m_matR))
	{
		LOG(WARNING) << "Detect current matrix does not satisfy constraints.";
		bfm_utils::SatisfyExtMat(m_matR, m_vecT);
		LOG(WARNING) << "Problem solved";
	}

	double sy = std::sqrt(m_matR(0,0) * m_matR(0,0) +  m_matR(1,0) * m_matR(1,0));
    bool bIsSingular = sy < 1e-6;

    if (!bIsSingular) 
	{
        m_aExtParams[2] = atan2(m_matR(2,1) , m_matR(2,2));
        m_aExtParams[1] = atan2(-m_matR(2,0), sy);
        m_aExtParams[0] = atan2(m_matR(1,0), m_matR(0,0));
    } 
	else 
	{
        m_aExtParams[2] = atan2(-m_matR(1,2), m_matR(1,1));
        m_aExtParams[1] = atan2(-m_matR(2,0), sy);
        m_aExtParams[0] = 0;
    }
	m_aExtParams[3] = m_vecT(0, 0);
	m_aExtParams[4] = m_vecT(1, 0);
	m_aExtParams[5] = m_vecT(2, 0);
	
	this->genTransMat();
}


void BfmManager::accExtParams(double *aExtParams) 
{
	/* in every iteration, P = R`(RP+t)+t`, 
	 * R_{new} = R`R_{old}
	 * t_{new} = R`t_{old} + t`
	 */

	Matrix3d matR;
	Vector3d vecT;	
	double dYaw   = aExtParams[0];
	double dPitch = aExtParams[1];
	double dRoll  = aExtParams[2];
	double dTx = aExtParams[3];
	double dTy = aExtParams[4];
	double dTz = aExtParams[5];

	/* accumulate rotation */
	matR = bfm_utils::Euler2Mat(dYaw, dPitch, dRoll, true);
	m_matR = matR * m_matR;

	/* accumulate translation */
	vecT << dTx, dTy, dTz;
	m_vecT = matR * m_vecT + vecT;
}

void BfmManager::writePly(std::string fn, long mode) const 
{
	std::ofstream out;
	/* Note: In Linux Cpp, we should use std::ios::out as flag, which is not necessary in Windows */
	out.open(fn, std::ios::out | std::ios::binary);
	if(!out.is_open()) 
	{
		std::string sErrMsg = "Creation of " + fn + " failed.";
		LOG(ERROR) << sErrMsg;
		throw std::runtime_error(sErrMsg);
		return;
	}

	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "comment Made from the 3D Morphable Face Model of the Univeristy of Basel, Switzerland.\n";
	out << "element vertex " << m_nVertices << "\n";
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "property uchar red\n";
	out << "property uchar green\n";
	out << "property uchar blue\n";
	out << "element face " << m_nFaces << "\n";
	out << "property list uchar int vertex_indices\n";
	out << "end_header\n";

	std::cout<<"Writer_test_output"<<std::endl;

	int cnt = 0;
	for (int iVertice = 0; iVertice < m_nVertices; iVertice++) 
	{
		float x, y, z;
		if(mode & ModelWriteMode_NoExpr) 
		{
			x = float(m_vecCurrentShape(iVertice * 3)) ;
			y = float(m_vecCurrentShape(iVertice * 3 + 1));
			z = float(m_vecCurrentShape(iVertice * 3 + 2));
		} 
		else 
		{
			x = float(m_vecCurrentBlendshape(iVertice * 3));
			y = float(m_vecCurrentBlendshape(iVertice * 3 + 1));
			z = float(m_vecCurrentBlendshape(iVertice * 3 + 2));
		}

		if(mode & ModelWriteMode_CameraCoord) 
		{
			x *= m_dSc;
			y *= m_dSc;
			z *= m_dSc;
			bfm_utils::Trans(m_aExtParams.data(), x, y, z);
			// y = -y; z = -z;
		}

		float r, g, b;
		if (mode & ModelWriteMode_PickLandmark)
		{
			bool bIsLandmark = false;
			for(const auto& [dlibIdx, bfmIdx] : m_mapLandmarkIndices)
			{
				if(bfmIdx == (int)iVertice)
				{
					bIsLandmark = true;
					break;
				}
			}
			if(bIsLandmark)
			{
				r = 255;
				g = 0;
				b = 0;
				cnt++;
			}else{
				r = m_vecCurrentTex(iVertice * 3);
				g = m_vecCurrentTex(iVertice * 3 + 1);
				b = m_vecCurrentTex(iVertice * 3 + 2);
			}
		} 
		else 
		{
			r = m_vecCurrentTex(iVertice * 3);
			g = m_vecCurrentTex(iVertice * 3 + 1);
			b = m_vecCurrentTex(iVertice * 3 + 2);
		}

		// std::cout<<iVertice<<"th vertex: (xyz)=(";
		// std::cout<<"x: "<<x<<",";
		// std::cout<<"y: "<<y<<",";
		// std::cout<<"z: "<<z<<")";


		// std::cout<<" (rgb)=(";
		// std::cout<<"r: "<<float2color(r)<<",";
		// std::cout<<"g: "<<float2color(g)<<",";
		// std::cout<<"b: "<<float2color(b)<<")"<<std::endl;

		// std::cout<<iVertice<<": ";
		// std::cout<<x<<" "<<y<<" "<<z<<" "<<float2color(r)<<" "<<float2color(g)<<" "<<float2color(b)<<"\n";
		out<<x<<" "<<y<<" "<<z<<" "<<float2color(r)<<" "<<float2color(g)<<" "<<float2color(b)<<"\n";

		// out.write((char *)&x, sizeof(x));
		// out.write((char *)&y, sizeof(y));
		// out.write((char *)&z, sizeof(z));
		// out.write((char *)&r, sizeof(r));
		// out.write((char *)&g, sizeof(g));
		// out.write((char *)&b, sizeof(b));
	}

	if ((mode & ModelWriteMode_PickLandmark) && cnt != m_mapLandmarkIndices.size()) 
	{
		LOG(ERROR) << "Pick too less landmarks.";
		LOG(ERROR) << "Number of picked points is " << cnt;
		throw std::runtime_error("Pick too less landmarks");
	}

	unsigned char N_VER_PER_FACE = 3;
	for (int iFace = 0; iFace < (int)m_nFaces; iFace++) 
	{
		// out.write((char *)&N_VER_PER_FACE, sizeof(N_VER_PER_FACE));
		int x = m_vecTriangleList(iFace * 3);
		int y = m_vecTriangleList(iFace * 3 + 1);
		int z = m_vecTriangleList(iFace * 3 + 2);

		// out.write((char *)&x, sizeof(x));
		// out.write((char *)&y, sizeof(y));
		// out.write((char *)&z, sizeof(z));
		// std::cout<<(int)N_VER_PER_FACE<<" "<<x<<" "<<y<<" "<<z<<"\n";
		out<<(int)N_VER_PER_FACE<<" "<<x<<" "<<y<<" "<<z<<"\n";
	}

	out.close();
}


void BfmManager::writeLandmarkPly(std::string fn) const {
	std::ofstream out;
	/* Note: In Linux Cpp, we should use std::ios::out as flag, which is not necessary in Windows */
	out.open(fn, std::ios::out | std::ios::binary);
	if(!out.is_open()) 
	{
		LOG(ERROR) << "Creation of " << fn << " failed.";
		return;
	}

	std::cout<<"Writing ply ...."<<std::endl;

	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "comment Made from the 3D Morphable Face Model of the Univeristy of Basel, Switzerland.\n";
	out << "element vertex " << m_mapLandmarkIndices.size() << "\n";
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "end_header\n";

	// unsigned long int cnt = 0;
	for (int i = 0; i < m_mapLandmarkIndices.size(); i++) 
	{
		float x, y, z;
		x = float(m_vecLandmarkCurrentBlendshape(i * 3));
		y = float(m_vecLandmarkCurrentBlendshape(i * 3 + 1));
		z = float(m_vecLandmarkCurrentBlendshape(i * 3 + 2));
		out<<x<<" "<<y<<" "<<z<<"\n";
		// out.write((char *)&x, sizeof(x));
		// out.write((char *)&y, sizeof(y));
		// out.write((char *)&z, sizeof(z));
	}

	out.close();
	std::cout<<"Finish write ply"<<std::endl;
}


void BfmManager::clrExtParams()
{
	m_aExtParams.fill(0.0);
	this->genTransMat();
	this->genFace();
}

std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Vector3i>> BfmManager::GetBFM(std::string f_name_ply, std::vector<double> & Coef_shape, std::vector<double> & Coef_Tex, std::vector<double> & Coef_exp,  bool average, bool withExp){
	
	if(average){
		double dScale[3]= {sqrt(m_vecShapeEv.minCoeff()), sqrt(m_vecTexEv.minCoeff()), sqrt(m_vecExprEv.minCoeff())};
		m_aShapeCoef = bfm_utils::randn(m_nIdPcs, 0.0);
		// std::cout<<"m_aShapeCoef"<<std::endl;
		// for(int i = 0; i<199; i++){
		// 	std::cout<<m_aShapeCoef[i]<<", ";
		// }
		// std::cout<<std::endl;
		m_aTexCoef   = bfm_utils::randn(m_nIdPcs, 0.0);
		// std::cout<<"m_aTexCoef"<<std::endl;
		// for(int i = 0; i<199; i++){
		// 	std::cout<<m_aTexCoef[i]<<", ";
		// }
		// std::cout<<std::endl;
		if(m_strVersion != "2009"){
			m_aExprCoef  = bfm_utils::randn(m_nExprPcs, 0.0);
			
			// std::cout<<"m_aExprCoef"<<std::endl;
			// for(int i = 0; i<100; i++){
			// 	std::cout<<m_aExprCoef[i]<<", ";
			// }
			// std::cout<<std::endl;
		}
		this->genFace();
	}else{
		double * p2shape = GetPointer2array(Coef_shape);
		double * p2tex = GetPointer2array(Coef_Tex);
		double * p2exp = GetPointer2array(Coef_exp);

		m_aShapeCoef = p2shape;
		m_aTexCoef   = p2tex;
		m_aExprCoef = p2exp;

		//obtain the each vertex positions after apply coefficents

		m_vecCurrentShape = this->coef2Object(m_aShapeCoef, m_vecShapeMu, m_matShapePc, m_vecShapeEv, m_nIdPcs);
		m_vecCurrentTex   = this->coef2Object(m_aTexCoef, m_vecTexMu, m_matTexPc, m_vecTexEv, m_nIdPcs);
		m_vecCurrentExpr  = this->coef2Object(m_aExprCoef, m_vecExprMu, m_matExprPc, m_vecExprEv, m_nExprPcs);
		m_vecCurrentBlendshape = m_vecCurrentShape + m_vecCurrentExpr;	
	}

		std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Vector3i>> _result_mesh_components;
		
		_result_mesh_components = this->rk_Meshwriter(f_name_ply, withExp);

		return _result_mesh_components;
}



std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Vector3i>> BfmManager::rk_Meshwriter(std::string f_name_ply, bool withExp){
	std::ofstream out;
	/* Note: In Linux Cpp, we should use std::ios::out as flag, which is not necessary in Windows */
	out.open(f_name_ply, std::ios::out | std::ios::binary);
	if(!out.is_open()) 
	{
		std::string sErrMsg = "Creation of " + f_name_ply + " failed.";
		LOG(ERROR) << sErrMsg;
		throw std::runtime_error(sErrMsg);
		// return;
	}

    std::cout<<"Writing face mesh ply ("<<f_name_ply<<")"<<"....";

	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "comment Made from the 3D Morphable Face Model of the Univeristy of Basel, Switzerland.\n";
	out << "element vertex " << m_nVertices << "\n";
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "property uchar red\n";
	out << "property uchar green\n";
	out << "property uchar blue\n";
	out << "element face " << m_nFaces << "\n";
	out << "property list uchar int vertex_indices\n";
	out << "end_header\n";

	std::vector<Eigen::Vector3f> result_vertexPos;
	std::vector<Eigen::Vector3f> result_vertexRGB;
	std::vector<Vector3i> result_triangleList;

	int cnt = 0;
	for (int iVertice = 0; iVertice < m_nVertices; iVertice++) 
	{
		float x, y, z;
		if(!withExp) 
		{
			x = float(m_vecCurrentShape(iVertice * 3)) ;
			y = float(m_vecCurrentShape(iVertice * 3 + 1));
			z = float(m_vecCurrentShape(iVertice * 3 + 2));
		} 
		else 
		{
			x = float(m_vecCurrentBlendshape(iVertice * 3));
			y = float(m_vecCurrentBlendshape(iVertice * 3 + 1));
			z = float(m_vecCurrentBlendshape(iVertice * 3 + 2));
		}

		Eigen::Vector3f _point_coords = {x, y, z};
		result_vertexPos.push_back(_point_coords);

		float r, g, b;
		r = m_vecCurrentTex(iVertice * 3);
		g = m_vecCurrentTex(iVertice * 3 + 1);
		b = m_vecCurrentTex(iVertice * 3 + 2);

		Eigen::Vector3f _point_rgb = {r, g, b};
		result_vertexRGB.push_back(_point_rgb);

		out<<x<<" "<<y<<" "<<z<<" "<<float2color(r)<<" "<<float2color(g)<<" "<<float2color(b)<<"\n";
	}

	unsigned char N_VER_PER_FACE = 3;
	for (int iFace = 0; iFace < (int)m_nFaces; iFace++) 
	{
		int x = m_vecTriangleList(iFace * 3);
		int y = m_vecTriangleList(iFace * 3 + 1);
		int z = m_vecTriangleList(iFace * 3 + 2);

		Vector3i _triangle_id_set = {x, y, z};
		result_triangleList.push_back(_triangle_id_set);

		out<<(int)N_VER_PER_FACE<<" "<<x<<" "<<y<<" "<<z<<"\n";
	}

	std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Vector3i>> result_blendshape_components = std::make_tuple(result_vertexPos, result_vertexRGB, result_triangleList);

	out.close();
    std::cout<<"Finish face mesh ply ("<<f_name_ply<<")"<<std::endl;

	return result_blendshape_components;

}

void BfmManager::GetShapeComponents(VectorXd & vec_Mu_Shape, VectorXd & vec_Ev_Shape, MatrixXd & mat_Pc_Shape){
	vec_Mu_Shape = BfmManager::get_ShapeMu();
	vec_Ev_Shape = BfmManager::get_ShapeEv();
	mat_Pc_Shape = BfmManager::get_ShapePc();
}

void BfmManager::GetTexComponents(VectorXd & vec_Mu_Tex, VectorXd & vec_Ev_Tex, MatrixXd & mat_Pc_Tex){
	vec_Mu_Tex = BfmManager::get_TexMu();
	vec_Ev_Tex = BfmManager::get_TexEv();
	mat_Pc_Tex = BfmManager::get_TexPc();
}

void BfmManager::GetExpComponents(VectorXd & vec_Mu_Exp, VectorXd & vec_Ev_Exp, MatrixXd & mat_Pc_Exp){
	vec_Mu_Exp = BfmManager::get_ExprMu();
	vec_Ev_Exp = BfmManager::get_ExprEv();
	mat_Pc_Exp = BfmManager::get_ExprPc();
}

