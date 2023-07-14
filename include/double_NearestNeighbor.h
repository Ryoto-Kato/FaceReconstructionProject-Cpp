#pragma once
#include <flann/flann.hpp>

#include "Eigen.h"

struct Match {
	int idx;
	double weight;
};

class NearestNeighborSearch {
public:
	virtual ~NearestNeighborSearch() {}

	virtual void setMatchingMaxDistance(double maxDistance) {
		m_maxDistance = maxDistance;
	}

	virtual void buildIndex(const std::vector<Eigen::Vector3d>& targetPoints) = 0;
	virtual std::vector<Match> queryMatches(const std::vector<Vector3d>& transformedPoints) = 0;

protected:
	double m_maxDistance;

	NearestNeighborSearch() : m_maxDistance{ double(1e8) } {}
};


/**
 * Brute-force nearest neighbor search.
 */
class NearestNeighborSearchBruteForce : public NearestNeighborSearch {
public:
	NearestNeighborSearchBruteForce() : NearestNeighborSearch() {}

	void buildIndex(const std::vector<Eigen::Vector3d>& targetPoints) {
		m_points = targetPoints;
	}

	std::vector<Match> queryMatches(const std::vector<Vector3d>& transformedPoints) {
		const unsigned nMatches = transformedPoints.size();
		std::vector<Match> matches(nMatches);
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
	std::vector<Eigen::Vector3d> m_points;

	Match getClosestPoint(const Vector3d& p) {
		int idx = -1;

		double minDist = std::numeric_limits<double>::max();
		for (unsigned int i = 0; i < m_points.size(); ++i) {
			double dist = (p - m_points[i]).norm();
			if (minDist > dist) {
				idx = i;
				minDist = dist;
			}
		}

		if (minDist <= m_maxDistance)
			return Match{ idx, double(1.0) };
		else
			return Match{ -1, double(0.0) };
	}
};


/**
 * Nearest neighbor search using FLANN.
 */
class NearestNeighborSearchFlann : public NearestNeighborSearch {
public:
	NearestNeighborSearchFlann() :
		NearestNeighborSearch(),
		m_nTrees{ 1 },
		m_index{ nullptr },
		m_flatPoints{ nullptr }
	{ }

	~NearestNeighborSearchFlann() {
		if (m_index) {
			delete m_flatPoints;
			delete m_index;
			m_flatPoints = nullptr;
			m_index = nullptr;
		}
	}

	void buildIndex(const std::vector<Eigen::Vector3d>& targetPoints) {
		std::cout << "Initializing FLANN index with " << targetPoints.size() << " points." << std::endl;

		// FLANN requires that all the points be flat. Therefore we copy the points to a separate flat array.
		m_flatPoints = new double[targetPoints.size() * 3];
		for (size_t pointIndex = 0; pointIndex < targetPoints.size(); pointIndex++) {
			for (size_t dim = 0; dim < 3; dim++) {
				m_flatPoints[pointIndex * 3 + dim] = targetPoints[pointIndex][dim];
			}
		}

		flann::Matrix<double> dataset(m_flatPoints, targetPoints.size(), 3);

		// Building the index takes some time.
		m_index = new flann::Index<flann::L2<double>>(dataset, flann::KDTreeIndexParams(m_nTrees));
		m_index->buildIndex();

		std::cout << "FLANN index created." << std::endl;
	}

	std::vector<Match> queryMatches(const std::vector<Vector3d>& transformedPoints) {
		if (!m_index) {
			std::cout << "FLANN index needs to be build before querying any matches." << std::endl;
			return {};
		}

		// FLANN requires that all the points be flat. Therefore we copy the points to a separate flat array.
		double* queryPoints = new double[transformedPoints.size() * 3];
		for (size_t pointIndex = 0; pointIndex < transformedPoints.size(); pointIndex++) {
			for (size_t dim = 0; dim < 3; dim++) {
				queryPoints[pointIndex * 3 + dim] = transformedPoints[pointIndex][dim];
			}
		}

		flann::Matrix<double> query(queryPoints, transformedPoints.size(), 3);
		flann::Matrix<int> indices(new int[query.rows * 1], query.rows, 1);
		flann::Matrix<double> distances(new double[query.rows * 1], query.rows, 1);
		
		// Do a knn search, searching for 1 nearest point and using 16 checks.
		flann::SearchParams searchParams{ 16 };
		searchParams.cores = 0;
		m_index->knnSearch(query, indices, distances, 1, searchParams);

		// Filter the matches.
		const unsigned nMatches = transformedPoints.size();
		std::vector<Match> matches;
		matches.reserve(nMatches);

		std::cout<<"m_maxDistance: "<<m_maxDistance<<std::endl;

		for (int i = 0; i < nMatches; ++i) {
			double dist = (*distances[i]);
			std::cout<<i<<"th match distance: "<<dist<<std::endl;
			if (*distances[i] <= m_maxDistance)
				matches.push_back(Match{ *indices[i], double(1.0) });
			else
				matches.push_back(Match{ -1, double(0.0) });
		}


		// Release the memory.
		delete[] query.ptr();
		delete[] indices.ptr();
		delete[] distances.ptr();

		return matches;
	}

private:
	int m_nTrees;
	flann::Index<flann::L2<double>>* m_index;
	double* m_flatPoints;
};


