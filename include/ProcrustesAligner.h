#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		
		Matrix4f estimatedPose(4,4);
		estimatedPose.block<3,3>(0,0) = rotation;
		estimatedPose.block<3,1>(0,3) = translation;
		Vector4f filler_vec = Vector4f::Zero();
		filler_vec(3) = 1;
		estimatedPose.block<1,4>(3,0) = filler_vec;
		
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.

		Vector3f mean = Vector3f::Zero();
		for (int i = 0; i < points.size(); i++) {
			mean += points[i];
		}

		return mean/points.size();
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).

		Matrix3f rotation(3,3);
		
		MatrixXf src_points_matrix(sourcePoints.size(), 3);
		MatrixXf trgt_points_matrix(targetPoints.size(), 3);
		for (int i = 0; i < sourcePoints.size(); i++) {
			src_points_matrix.block<1,3>(i,0) = sourcePoints[i];
			trgt_points_matrix.block<1,3>(i,0) = targetPoints[i];
		}

		src_points_matrix = src_points_matrix.rowwise() - sourceMean.transpose();
		trgt_points_matrix = trgt_points_matrix.rowwise() - targetMean.transpose();

		JacobiSVD<MatrixXf> svd_cross_covariance(trgt_points_matrix.transpose() * src_points_matrix, ComputeFullU | ComputeFullV);
		
		rotation = svd_cross_covariance.matrixU() * svd_cross_covariance.matrixV().transpose();

		if (rotation.determinant() == -1) {
			Matrix3f mirror_mat(3,3);
			mirror_mat << 1, 0, 0,
						  0, 1, 0,
						  0, 0, -1;
			rotation = svd_cross_covariance.matrixU() * mirror_mat * svd_cross_covariance.matrixV().transpose();
		}

        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.

		Vector3f translation;
		translation = -rotation * sourceMean + targetMean;
        return translation;
	}
};
