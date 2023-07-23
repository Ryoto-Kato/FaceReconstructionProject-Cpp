#pragma once
#include <flann/flann.hpp>

#include "Eigen.h"

struct Match_ps {
	int idx;
	float weight;
};

class NearestNeighborSearch_poseEstimate {
public:
	virtual ~NearestNeighborSearch_poseEstimate() {}

	virtual void setMatchingMaxDistance(float maxDistance) {
		m_maxDistance = maxDistance;
	}

	virtual void buildIndex(const std::vector<Eigen::Vector3f>& targetPoints) = 0;
	virtual std::vector<Match_ps> queryMatches(const std::vector<Vector3f>& transformedPoints) = 0;
	virtual inline double get_aveDist() = 0;

protected:
	float m_maxDistance;

	NearestNeighborSearch_poseEstimate() : m_maxDistance{ 0.005f } {}
};


/**
 * Brute-force nearest neighbor search.
 */
class NearestNeighborSearch_poseEstimateBruteForce : public NearestNeighborSearch_poseEstimate {
public:
	NearestNeighborSearch_poseEstimateBruteForce() : NearestNeighborSearch_poseEstimate() {}

	void buildIndex(const std::vector<Eigen::Vector3f>& targetPoints) {
		m_points = targetPoints;
	}

	std::vector<Match_ps> queryMatches(const std::vector<Vector3f>& transformedPoints) {
		const unsigned nMatches = transformedPoints.size();
		std::vector<Match_ps> matches(nMatches);
		const unsigned nTargetPoints = m_points.size();
		std::cout << "nMatches: " << nMatches << std::endl;
		std::cout << "nTargetPoints: " << nTargetPoints << std::endl;

		#pragma omp parallel for
		for (int i = 0; i < nMatches; i++) {
			matches[i] = getClosestPoint(transformedPoints[i]);
		}

		return matches;
	}

private:
	std::vector<Eigen::Vector3f> m_points;

	Match_ps getClosestPoint(const Vector3f& p) {
		int idx = -1;

		float minDist = std::numeric_limits<float>::max();
		for (unsigned int i = 0; i < m_points.size(); ++i) {
			float dist = (p - m_points[i]).norm();
			if (minDist > dist) {
				idx = i;
				minDist = dist;
			}
		}

		if (minDist <= m_maxDistance)
			return Match_ps{ idx, 1.f };
		else
			return Match_ps{ -1, 0.f };
	}
};


/**
 * Nearest neighbor search using FLANN.
 */
class NearestNeighborSearch_poseEstimateFlann : public NearestNeighborSearch_poseEstimate {
public:
	NearestNeighborSearch_poseEstimateFlann() :
		NearestNeighborSearch_poseEstimate(),
		m_nTrees{ 1 },
		m_index{ nullptr },
		m_flatPoints{ nullptr }
	{ }

	~NearestNeighborSearch_poseEstimateFlann() {
		if (m_index) {
			delete m_flatPoints;
			delete m_index;
			m_flatPoints = nullptr;
			m_index = nullptr;
		}
	}

	void buildIndex(const std::vector<Eigen::Vector3f>& targetPoints) {
		std::cout << "Initializing FLANN index with " << targetPoints.size() << " points." << std::endl;

		// FLANN requires that all the points be flat. Therefore we copy the points to a separate flat array.
		m_flatPoints = new float[targetPoints.size() * 3];
		for (size_t pointIndex = 0; pointIndex < targetPoints.size(); pointIndex++) {
			for (size_t dim = 0; dim < 3; dim++) {
				m_flatPoints[pointIndex * 3 + dim] = targetPoints[pointIndex][dim];
			}
		}

		flann::Matrix<float> dataset(m_flatPoints, targetPoints.size(), 3);

		// Building the index takes some time.
		m_index = new flann::Index<flann::L2<float>>(dataset, flann::KDTreeIndexParams(m_nTrees));
		m_index->buildIndex();

		std::cout << "FLANN index created." << std::endl;
	}

	std::vector<Match_ps> queryMatches(const std::vector<Vector3f>& transformedPoints) {
		if (!m_index) {
			std::cout << "FLANN index needs to be build before querying any matches." << std::endl;
			return {};
		}

		// FLANN requires that all the points be flat. Therefore we copy the points to a separate flat array.
		float* queryPoints = new float[transformedPoints.size() * 3];
		for (size_t pointIndex = 0; pointIndex < transformedPoints.size(); pointIndex++) {
			for (size_t dim = 0; dim < 3; dim++) {
				queryPoints[pointIndex * 3 + dim] = transformedPoints[pointIndex][dim];
			}
		}

		flann::Matrix<float> query(queryPoints, transformedPoints.size(), 3);
		flann::Matrix<int> indices(new int[query.rows * 1], query.rows, 1);
		flann::Matrix<float> distances(new float[query.rows * 1], query.rows, 1);
		
		// Do a knn search, searching for 1 nearest point and using 16 checks.
		flann::SearchParams searchParams{ 16 };
		searchParams.cores = 0;
		m_index->knnSearch(query, indices, distances, 1, searchParams);

		// Filter the matches.
		const unsigned nMatches = transformedPoints.size();
		std::vector<Match_ps> matches;
		matches.reserve(nMatches);

		double average_dist = 0;
		int counter_valid_point = 0;

		for (int i = 0; i < nMatches; ++i) {
            auto dist = *distances[i];
			if (dist <= m_maxDistance){
				matches.push_back(Match_ps{ *indices[i], 1.f });
				average_dist+=dist;
				counter_valid_point++;
			}
			else{
				matches.push_back(Match_ps{ -1, 0.f });
			}
		}

		ave_dist = average_dist/counter_valid_point;

		// Release the memory.
		delete[] query.ptr();
		delete[] indices.ptr();
		delete[] distances.ptr();

		return matches;
	}

	inline double get_aveDist(){
		return ave_dist;
	}

private:
	int m_nTrees;
	flann::Index<flann::L2<float>>* m_index;
	float* m_flatPoints;
	double ave_dist;
};


