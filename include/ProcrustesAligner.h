#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT((sourcePoints.size() == targetPoints.size()) && ("The number of source and target points should be the same, since every source point is matched with corresponding target point."));


		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);
		
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements

		Matrix4f estimatedPose = Matrix4f::Identity(4,4);
		/*
		r11, r12, r13, t1
		r21, r22, r23, t2
		r31, r32, r33, t3,
		  0,   0,   0,  1,
		
		*/
		// std::cout<<rotation<<std::endl;
		// std::cout<<translation<<std::endl;
		
		estimatedPose.block(0,0,3,3) = rotation;
		// std::cout<<estimatedPose<<std::endl;
		estimatedPose.block(0,3,3,1) = translation;

		// std::cout<<estimatedPose<<std::endl;
		
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// points : list
		Vector3f sum{0.0, 0.0, 0.0};
		Vector3f mean{0.0, 0.0, 0.0};

		long unsigned int size = points.size();

		for (unsigned int i = 0; i<points.size(); i++){
			sum += points[i]; 
		}

		mean = sum/size;
        return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.
		Matrix3f rotation = Matrix3f::Identity(3,3);
		MatrixXf cov;
		MatrixXf cov_S;
		int num_corresp = targetPoints.size();
		MatrixXf T(num_corresp, 3); //shape=[3, #keypoints], centralized target point coordinates w.r.t targetMean
		MatrixXf S(num_corresp, 3); //shape=[3, #keypoints], centralized source point coordinates w.r.t sourceMean
		
		for(int i = 0; i<num_corresp; i++){
			Vector3f T_centralized_pos = targetPoints[i]-targetMean;
			T.row(i)=T_centralized_pos;
			Vector3f S_centralized_pos = sourcePoints[i]-sourceMean;
			S.row(i)=S_centralized_pos;
		}

		// std::cout<<T<<std::endl;
		// std::cout<<S<<std::endl;

		cov = T.transpose() * S; // shape=[3, 3]
		// cov_S = S * S.transpose();

		// std::cout<<cov<<std::endl;

        JacobiSVD<MatrixXf> svd(cov, ComputeThinU | ComputeThinV);
        svd.compute(cov);
		Matrix3f singular_matrix = Matrix3f::Zero(3, 3);
		singular_matrix.diagonal() = svd.singularValues();
		rotation = svd.matrixU() * svd.matrixV().transpose();
		if (rotation.determinant()<0){
			Matrix3f mirror = Matrix3f::Zero(3, 3);
			Vector3f one_entries = Vector3f::Ones(3);
			//flip one vector to mirror
			one_entries[2] *= -1;
			mirror.diagonal() = one_entries;
			// std::cout<<mirror<<std::endl;
			rotation = svd.matrixU() * mirror *svd.matrixV().transpose();
		}
		// std::cout<<rotation.determinant()<<std::endl;

        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.
		Vector3f translation;
		translation = -1 * rotation * sourceMean + targetMean;
        return translation;
	}
};