#pragma once

// The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <flann/flann.hpp>

#include "SimpleMesh.h"
#include "NearestNeighbor.h"
#include "PointCloud.h"
#include "ProcrustesAligner.h"


/**
 * Helper methods for writing Ceres cost functions.
 */
template <typename T>
static inline void fillVector(const Vector3f& input, T* output) {
	output[0] = T(input[0]);
	output[1] = T(input[1]);
	output[2] = T(input[2]);
}


/**
 * Pose increment is only an interface to the underlying array (in constructor, no copy
 * of the input array is made).
 * Important: Input array needs to have a size of at least 6.
 */
template <typename T>
class PoseIncrement {
public:
	explicit PoseIncrement(T* const array) : m_array{ array } { }
	
	void setZero() {
		for (int i = 0; i < 6; ++i)
			m_array[i] = T(0);
	}

	T* getData() const {
		return m_array;
	}

	/**
	 * Applies the pose increment onto the input point and produces transformed output point.
	 * Important: The memory for both 3D points (input and output) needs to be reserved (i.e. on the stack)
	 * beforehand).
	 */
	void apply(T* inputPoint, T* outputPoint) const {
		// pose[0,1,2] is angle-axis rotation.
		// pose[3,4,5] is translation.
		const T* rotation = m_array;
		const T* translation = m_array + 3;

		T temp[3];
		// return temp which rotated the inputPoint
		ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

		outputPoint[0] = temp[0] + translation[0];
		outputPoint[1] = temp[1] + translation[1];
		outputPoint[2] = temp[2] + translation[2];
	}

	/**
	 * Converts the pose increment with rotation in SO3 notation and translation as 3D vector into
	 * transformation 4x4 matrix.
	 */
	static Matrix4f convertToMatrix(const PoseIncrement<double>& poseIncrement) {
		// pose[0,1,2] is angle-axis rotation.
		// pose[3,4,5] is translation.
		double* pose = poseIncrement.getData();
		double* rotation = pose;
		double* translation = pose + 3;

		// Convert the rotation from SO3 to matrix notation (with column-major storage).
		double rotationMatrix[9];
		ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

		// Create the 4x4 transformation matrix.
		Matrix4f matrix;
		matrix.setIdentity();
		matrix(0, 0) = float(rotationMatrix[0]);	matrix(0, 1) = float(rotationMatrix[3]);	matrix(0, 2) = float(rotationMatrix[6]);	matrix(0, 3) = float(translation[0]);
		matrix(1, 0) = float(rotationMatrix[1]);	matrix(1, 1) = float(rotationMatrix[4]);	matrix(1, 2) = float(rotationMatrix[7]);	matrix(1, 3) = float(translation[1]);
		matrix(2, 0) = float(rotationMatrix[2]);	matrix(2, 1) = float(rotationMatrix[5]);	matrix(2, 2) = float(rotationMatrix[8]);	matrix(2, 3) = float(translation[2]);
		
		return matrix;
	}

private:
	T* m_array;
};


/**
 * Optimization constraints.
 */
class PointToPointConstraint {
public:
	PointToPointConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float weight) :
		m_sourcePoint{ sourcePoint },
		m_targetPoint{ targetPoint },
		m_weight{ weight }
	{ }

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const {

		//Given
		//POSE (to be optimized = M(SO(3)))
			// pose: lie algebra(6DOF)
			// pose[0,1,2] is angle-axis rotation.
			// pose[3,4,5] is translation.

		//Source point p_s (red bunny point clouds)
			// m_sourcePoint(Vector3f) in PointToPointConstraint (one vertex)
		
		//Target point p_t (green bunny point clouds)
			// m_targetPoint(Vector3f) in PointToPointConstraint (one vertex)
		
		//weight (additional parameters to be optimized)
			// m_weight (float) in PointToPointConstraint (one vertex)
		
		// poseIncrement.m_array(private) = pose
		PoseIncrement<T> poseIncrement = PoseIncrement<T>(const_cast<T* const>(pose));

		// TODO: Implement the point-to-point cost function.
		// The resulting 3D residual should be stored in the residuals array. To apply the pose 
		// increment (pose parameters) to the source point, you can use the PoseIncrement class.
		// Important: Ceres automatically squares the cost function.

		/*
		*1: we pose-increment.apply()/transformed (rotate and translate) the source point with pose (we approximate so far)
		*2: subtract the target vertex coordinate from the transformed corresponding vertex coordinate
		*3: register as the residual
		*/
		
		//poseIncrement.apply()
		T transformed_msP[3];
		
		// convert vector3f to T array
		T source_point[3];
		fillVector(m_sourcePoint, source_point);
		
		poseIncrement.apply(source_point, transformed_msP);
		//poseIncrement.apply()

		T subtract_st[3];

		T targetPoint[3];
		fillVector(m_targetPoint, targetPoint);

		subtract_st[0] = transformed_msP[0] - targetPoint[0];
		subtract_st[1] = transformed_msP[1] - targetPoint[1];
		subtract_st[2] = transformed_msP[2] - targetPoint[2];

		float _weight = std::sqrt(m_weight);
		float _LAMBDA = std::sqrt(LAMBDA);
		T coeff = T(_weight * _LAMBDA);

		// Three residual
		residuals[0] = subtract_st[0] * coeff;
		residuals[1] = subtract_st[1] * coeff;
		residuals[2] = subtract_st[2] * coeff;

		return true;
	}

	static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float weight) {
		return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 6>(
			new PointToPointConstraint(sourcePoint, targetPoint, weight)
		);
	}

protected:
	const Vector3f m_sourcePoint;
	const Vector3f m_targetPoint;
	const float m_weight;
	const float LAMBDA = 0.1f;
};

class PointToPlaneConstraint {
public:
	PointToPlaneConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) :
		m_sourcePoint{ sourcePoint },
		m_targetPoint{ targetPoint },
		m_targetNormal{ targetNormal },
		m_weight{ weight }
	{ }

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const {

		//Given
		//POSE (to be optimized = M(SO(3)))
			// pose: lie algebra(6DOF)
			// pose[0,1,2] is angle-axis rotation.
			// pose[3,4,5] is translation.

		//Source point p_s (red bunny point clouds)
			// m_sourcePoint(Vector3f) in PointToPointConstraint (one vertex)
		
		//Target point p_t (green bunny point clouds)
			// m_targetPoint(Vector3f) in PointToPointConstraint (one vertex)
		
		//Target Normal (normal of vertex in green bunny point clouds)
			// m_targetNormal(Vector3f) in PointToPlaneConstraint (one vertex)

		//weight (additional parameters to be optimized)
			// m_weight (float) in PointToPointConstraint (one vertex)


		PoseIncrement<T> poseIncrement = PoseIncrement<T>(const_cast<T* const>(pose));
		
		// TODO: Implement the point-to-plane cost function.
		// The resulting 1D residual should be stored in the residuals array. To apply the pose 
		// increment (pose parameters) to the source point, you can use the PoseIncrement class.
		// Important: Ceres automatically squares the cost function.

		/*
		*1: we pose-increment.apply()/transformed (rotate and translate) the source point with pose (we approximate so far)
		*2: subtract the target vertex coordinate from the transformed corresponding vertex coordinate
		*3: register as the residual
		*4: inner-product with subtracted vertex coordinates
		*/
		//poseIncrement.apply()
		T transformed_msP[3];

		//convert the Vector3f to T array
		T sourcePoint[3];
		fillVector(m_sourcePoint, sourcePoint);

		poseIncrement.apply(sourcePoint, transformed_msP);

		T subtract_st[3];

		T targetPoint[3];
		fillVector(m_targetPoint, targetPoint);
		
		subtract_st[0] = transformed_msP[0] - targetPoint[0];
		subtract_st[1] = transformed_msP[1] - targetPoint[1];
		subtract_st[2] = transformed_msP[2] - targetPoint[2];

		// residuals = m_targetNormal.dot(subtract_st);
		// auto dot_product = m_targetNormal[0]*subtract_st[0] + m_targetNormal[1]*subtract_st[1] + m_targetNormal[2]*subtract_st[2];

		float _weight = m_weight;
		float _LAMBDA = LAMBDA;
		T coeff = T(std::sqrt(_weight) * std::sqrt(_LAMBDA));

		T targetNormal[3];
		fillVector(m_targetNormal, targetNormal);

		//single residual
		residuals[0] = ceres::DotProduct(targetNormal, subtract_st) * coeff;
		return true;
	}

	static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) {
		return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, 6>(
			new PointToPlaneConstraint(sourcePoint, targetPoint, targetNormal, weight)
		);
	}

protected:
	const Vector3f m_sourcePoint;
	const Vector3f m_targetPoint;
	const Vector3f m_targetNormal;
	const float m_weight;
	const float LAMBDA = 1.0f;
};

/**
 * ICP optimizer - Abstract Base Class, using Ceres for optimization.
 */
class ICPOptimizer {
public:
	ICPOptimizer() : 
		m_bUsePointToPlaneConstraints{ false },
		m_nIterations{ 20 },
		m_nearestNeighborSearch{ std::make_unique<NearestNeighborSearchFlann>() }
	{ }

	void setMatchingMaxDistance(float maxDistance) {
		m_nearestNeighborSearch->setMatchingMaxDistance(maxDistance);
	}

	void usePointToPlaneConstraints(bool bUsePointToPlaneConstraints) {
		m_bUsePointToPlaneConstraints = bUsePointToPlaneConstraints;
	}

	void setNbOfIterations(unsigned nIterations) {
		m_nIterations = nIterations;
	}

	virtual Matrix4f estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f initialPose = Matrix4f::Identity()) = 0;

protected:
	bool m_bUsePointToPlaneConstraints;
	unsigned m_nIterations;
	std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch;

	std::vector<Vector3f> transformPoints(const std::vector<Vector3f>& sourcePoints, const Matrix4f& pose) {
		std::vector<Vector3f> transformedPoints;
		transformedPoints.reserve(sourcePoints.size());

		const auto rotation = pose.block(0, 0, 3, 3);
		const auto translation = pose.block(0, 3, 3, 1);

		for (const auto& point : sourcePoints) {
			transformedPoints.push_back(rotation * point + translation);
		}

		return transformedPoints;
	}

	std::vector<Vector3f> transformNormals(const std::vector<Vector3f>& sourceNormals, const Matrix4f& pose) {
		std::vector<Vector3f> transformedNormals;
		transformedNormals.reserve(sourceNormals.size());

		const auto rotation = pose.block(0, 0, 3, 3);

		for (const auto& normal : sourceNormals) {
			transformedNormals.push_back(rotation.inverse().transpose() * normal);
		}

		return transformedNormals;
	}

	void pruneCorrespondences(const std::vector<Vector3f>& sourceNormals, const std::vector<Vector3f>& targetNormals, std::vector<Match>& matches) {
		const unsigned nPoints = sourceNormals.size();

		for (unsigned i = 0; i < nPoints; i++) {
			Match& match = matches[i];
			if (match.idx >= 0) {
				const auto& sourceNormal = sourceNormals[i];
				const auto& targetNormal = targetNormals[match.idx];

				// TODO: Invalidate the match (set it to -1) if the angle between the normals is greater than 60
				// sourceNormal and targetNormal are normalized to length 1

				double angle = acosf64x(sourceNormal.dot(targetNormal));
				if(angle > (double)(60.0)){
					match.idx = -1;
				}
			}
		}
	}
};


/**
 * ICP optimizer - using Ceres for optimization.
 */
class CeresICPOptimizer : public ICPOptimizer {
public:
	CeresICPOptimizer() {}

	virtual Matrix4f estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f initialPose = Matrix4f::Identity()) override {
		// Build the index of the FLANN tree (for fast nearest neighbor lookup).
		m_nearestNeighborSearch->buildIndex(target.getPoints());

		// The initial estimate can be given as an argument.
		Matrix4f estimatedPose = initialPose;

		// We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its length presents
		// the rotation angle) and 3 parameters for the translation vector.
		double incrementArray[6];
		auto poseIncrement = PoseIncrement<double>(incrementArray);
		poseIncrement.setZero();

		//iterative optimization
		for (int i = 0; i < m_nIterations; ++i) {
			// Compute the matches.
			std::cout << "Matching points ..." << std::endl;
			clock_t begin = clock();

			auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
			auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

			//matches contains the correspondences
			auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
			pruneCorrespondences(transformedNormals, target.getNormals(), matches);

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			// Prepare point-to-point and point-to-plane constraints.
			ceres::Problem problem;
			prepareConstraints(transformedPoints, target.getPoints(), target.getNormals(), matches, poseIncrement, problem);

			// Configure options for the solver.
			ceres::Solver::Options options;
			configureSolver(options);

			// Run the solver (for one iteration).
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			// std::cout << summary.BriefReport() << std::endl;
			std::cout << summary.FullReport() << std::endl;

			// Update the current pose estimate (we always update the pose from the left, using left-increment notation).
			Matrix4f matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);
			estimatedPose = PoseIncrement<double>::convertToMatrix(poseIncrement) * estimatedPose;
			poseIncrement.setZero();

			std::cout << "Optimization iteration done." << std::endl;
		}

		return estimatedPose;
	}


private:
	void configureSolver(ceres::Solver::Options& options) {
		// Ceres options.
		options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
		options.use_nonmonotonic_steps = false;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = 1;
		options.max_num_iterations = 1;
		options.num_threads = 8;
	}

	void prepareConstraints(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints, const std::vector<Vector3f>& targetNormals, const std::vector<Match> matches, const PoseIncrement<double>& poseIncrement, ceres::Problem& problem) const {
		//poseIncrement (containing the lie algebra) is to be optimized
		/* poseIncrement will return the address of array (6 elements)
		double* pose = poseIncrement.getData();
		double* rotation = pose;
		double* translation = pose + 3;
		*/
		const unsigned nPoints = sourcePoints.size();

		for (unsigned i = 0; i < nPoints; ++i) {
			//matches[i] contains that corresponding vertex index of i
			/*
			*i: index of source point
			*match.index: index of corresponding target point 
			*match.weight: weight of the correspondence
			*/ 
			const auto match = matches[i];
			if (match.idx >= 0) {
				const auto& sourcePoint = sourcePoints[i];
				const auto& targetPoint = targetPoints[match.idx];
				const auto& weight = match.weight;

				if (!sourcePoint.allFinite() || !targetPoint.allFinite())
					continue;

				// TODO: Create a new point-to-point cost function and add it as constraint (i.e. residual block) 
				// to the Ceres problem

				// PointToPointConstraint ptpc = {sourcePoint, targetPoint, weight}; 
				ceres::CostFunction * cost_function = PointToPointConstraint::create(sourcePoint, targetPoint, weight);
				problem.AddResidualBlock(cost_function, nullptr, poseIncrement.getData());

				if (m_bUsePointToPlaneConstraints) {
					const auto& targetNormal = targetNormals[match.idx];

					if (!targetNormal.allFinite())
						continue;

					// TODO: Create a new point-to-plane cost function and add it as constraint (i.e. residual block) 
					// to the Ceres problem.

					// PointToPlaneConstraint n_ptpc = {sourcePoint, targetPoint, targetNormal, weight}; 
					ceres::CostFunction * cost_function = PointToPlaneConstraint::create(sourcePoint, targetPoint, targetNormal, weight);
					problem.AddResidualBlock(cost_function, nullptr, poseIncrement.getData());

				}
			}
		}
	}
};


/**
 * ICP optimizer - using linear least-squares for optimization.
 */
class LinearICPOptimizer : public ICPOptimizer {
public:
	LinearICPOptimizer(bool reg): regularizer(reg){}

	virtual Matrix4f estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f initialPose = Matrix4f::Identity()) override {
		// Build the index of the FLANN tree (for fast nearest neighbor lookup).
		m_nearestNeighborSearch->buildIndex(target.getPoints());

		// The initial estimate can be given as an argument.
		Matrix4f estimatedPose = initialPose;

		for (int i = 0; i < m_nIterations; ++i) {
			// Compute the matches.
			std::cout << "Matching points ..." << std::endl;
			clock_t begin = clock();

			auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
			auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

			auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
			pruneCorrespondences(transformedNormals, target.getNormals(), matches);

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			std::vector<Vector3f> sourcePoints;
			std::vector<Vector3f> targetPoints;
			std::vector<float> weights;

			// Add all matches to the sourcePoints and targetPoints vectors,
			// so that sourcePoints[i] matches targetPoints[i].
			for (int j = 0; j < transformedPoints.size(); j++) {
				const auto& match = matches[j];
				if (match.idx >= 0) {
					sourcePoints.push_back(transformedPoints[j]);
					targetPoints.push_back(target.getPoints()[match.idx]);
					weights.push_back(match.weight);
				}
			}

			// Estimate the new pose
			if (m_bUsePointToPlaneConstraints) {
				estimatedPose = estimatePosePointToPlane(sourcePoints, targetPoints, target.getNormals(), m_nIterations, weights) * estimatedPose;
			}
			else {
				estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
			}

			std::cout << "Optimization iteration done." << std::endl;
		}

		return estimatedPose;
	}

private:
	bool regularizer;

	Matrix4f estimatePosePointToPoint(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		//Point to point metric
		// we need to minimize a linear least-squares problem which can be solved with the Procrustes algorithm
		// The rotation matrix is defined by U*V.transpose()
		ProcrustesAligner procrustAligner;
		Matrix4f estimatedPose = procrustAligner.estimatePose(sourcePoints, targetPoints);

		return estimatedPose;
	}

	Matrix4f estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints, const std::vector<Vector3f>& targetNormals, const int iteration, const std::vector<float> & weights) {
		const unsigned nPoints = sourcePoints.size();
		float init_weight_pointToplane = std::sqrt(1.e-8f);
		float init_weight_pointTopoint = std::sqrt(1.e8);
		float Lamda=20;
		// Build the system
		// 4 equations(constraints) per point
		// 1: point to plane constraint
		// 2: x point to point constraint
		// 3: y point to point constraint
		// 4: z point to point constraint

		// parameters to be optimized (alpha, beta, gamma, 3Dvector(translation))
		MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
		VectorXf b = VectorXf::Zero(4 * nPoints);

		std::cout<<iteration<<std::endl;

		for (unsigned i = 0; i < nPoints; i++) {
			const auto& s = sourcePoints[i];
			const auto& d = targetPoints[i];
			const auto& n = targetNormals[i];
			// const auto& weight = weights[i];

			// TODO: Add the point-to-plane constraints to the system
			
			//outer product between the source point vector and target surface normal
			
			auto a_00 = n[2] * s[1] - n[1] * s[2];
			auto a_01 = n[0] * s[2] - n[2] * s[0];
			auto a_02 = n[1] * s[0] - n[0] * s[1];

			long double weight_pointToplane = 1.0f;
			weight_pointToplane = std::sqrt(weight_pointToplane);

			//create entries
			Eigen::VectorXf _row1(6);
			
			std::vector<float> vect_1 = {a_00, a_01, a_02, n[0], n[1], n[2]};

			for(unsigned int j = 0; j<6; j++){
				 _row1[j] = vect_1[j] * weight_pointToplane;
			}

			//insert row at corresponding index (point to plane)
			A.row(i) = _row1;
			
			b[i] = (n[0]*d[0] + n[1]*d[1] + n[2]*d[2] - n[0]*s[0] - n[1]*s[1] - n[2]*s[2]) * weight_pointToplane;

			// TODO: Add the point-to-point constraints to the system
			
			Eigen::VectorXf _row_x(6);
			std::vector<float> vect_x={0, s[2], -1*s[1], 1, 0, 0};

			Eigen::VectorXf _row_y(6);
			std::vector<float> vect_y={-1 * s[2], 0, s[0], 0, 1, 0};
			
			Eigen::VectorXf _row_z(6);
			std::vector<float> vect_z = {s[1], -1*s[0], 0, 0, 0, 1};

			long double weight_pointTopoint = 0.1f;
			weight_pointTopoint = std::sqrt(weight_pointTopoint);

			// weight_pointToplane = init_weight_pointToplane * std::exp(-1 * Lamda * iteration);
			// std::cout<<"weight point to point"<<std::endl;
			// std::cout<<weight_pointTopoProcrustesAlignerint<<std::endl;
			for(unsigned int t =0; t<6; t++){
				_row_x[t] = vect_x[t] * (weight_pointTopoint);
				_row_y[t] = vect_y[t] * (weight_pointTopoint);
				_row_z[t] = vect_z[t] * (weight_pointTopoint);
			}

			A.row(i+1) = _row_x;
			A.row(i+2) = _row_y;
			A.row(i+3) = _row_z;
			b[i+1] = (d[0] - s[0]) * (weight_pointTopoint);
			b[i+2] = (d[1] - s[1]) * (weight_pointTopoint);
			b[i+3] = (d[2] - s[2]) * (weight_pointTopoint);

            // TODO: Optionally, apply a higher weight to point-to-plane correspondence
		}

		// TODO: Solve the system
		VectorXf x(6);

		Eigen::MatrixXf System_mat = A.transpose()*A;
		Eigen::VectorXf rhs = A.transpose()*b;

		//regularizer
		//This regularizer will make the optimizer robust to the outliear
		//But in this case we do not have outlier (only the case we estimate shape pose)

		if(regularizer){
			double lambda = 0.01;
			double scaler = sqrt(7);
			lambda *= scaler;
			System_mat.diagonal() += lambda * lambda * VectorXf::Ones(6);
		}


		JacobiSVD<MatrixXf> svd(System_mat, ComputeFullU | ComputeFullV);
		svd.compute(System_mat);

		// The pseudo-inverse of A is defined as 
		// V(Sigma_inverse)U.T
		// Sigma_inverse is the inverse of the non-zero elements of the singular value matrix
		// To Obtbain the inverse of the singular value matrix -> leaving the zero elements unchanged
		// Replace the sub-matrix containing non-zero elements in diagonal by the its inverse

		auto U_trasposed = svd.matrixU().transpose(); //shape [4*nPoints, 4*nPoints]
		auto V = svd.matrixV(); //shape [6, 6]

		std::cout<<"U_transposed_dimension:	"<<U_trasposed.rows()<<" "<<U_trasposed.cols()<<std::endl;
		std::cout<<"V_dimension:	"<<V.rows()<<" "<<V.cols()<<std::endl;

		std::cout<<"Singular values length"<<std::endl;
		std::cout<<svd.singularValues().size()<<std::endl;
		
		//threshold
		double epsilon = 1.e-8;

		Eigen::MatrixXf Sigma_PseudoInvMat = Eigen::MatrixXf::Zero(6, 6);
		auto Singular_values = svd.singularValues();

		for(auto & sv : Singular_values){
			if(sv > epsilon){
				sv = 1.0/sv;
			}else{
				sv = 0;
			}
		}

		for(unsigned int i =0; i<6; i++){
			Sigma_PseudoInvMat(i, i) = Singular_values(i);
		}

		// std::cout<<Sigma_PseudoInvMat<<std::endl;

		// Sigma_PseudoInvMat.diagonal() = Singular_values;

		Eigen::MatrixXf A_pseudo_InvMat = Eigen::MatrixXf::Zero(6, 4*nPoints);

		std::cout<<"ryotok"<<std::endl;
		A_pseudo_InvMat = V * Sigma_PseudoInvMat * U_trasposed;
		x = A_pseudo_InvMat * rhs;

        float alpha = x(0), beta = x(1), gamma = x(2);

		// Build the pose matrix
		Matrix3f rotation = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
			                AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
			                AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();

		Vector3f translation = x.tail(3);

		// TODO: Build the pose matrix using the rotation and translation matrices
		Matrix4f estimatedPose;
		estimatedPose.block(0,0,3,3)=rotation;
		estimatedPose.block(0,3,3,1)=translation;
		

		return estimatedPose;
	}
};
