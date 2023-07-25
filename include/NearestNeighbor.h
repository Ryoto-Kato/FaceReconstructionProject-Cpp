#pragma once
#include <flann/flann.hpp>

#include "Eigen.h"
#include <queue>

struct Match {
	int idx;
	float weight;
};

class NearestNeighborSearch {
public:
	virtual ~NearestNeighborSearch() {}

	virtual void setMatchingMaxDistance(float maxDistance) {
		m_maxDistance = maxDistance;
	}

	virtual void buildIndex(const std::vector<Eigen::Vector3f>& targetPoints) = 0;
	virtual std::vector<Match> queryMatches(const std::vector<Vector3f>& transformedPoints) = 0;
	virtual inline float get_aveDist() = 0;

	virtual inline float get_measuredMaxDist() = 0;
	virtual inline float get_measuredMinDist() = 0;
	virtual inline std::vector<float> get_measuredDists() = 0;

protected:
	float m_maxDistance;

	NearestNeighborSearch() : m_maxDistance{ 0.005f } {}
};


/**
 * Brute-force nearest neighbor search.
 */
class NearestNeighborSearchBruteForce : public NearestNeighborSearch {
public:
	NearestNeighborSearchBruteForce() : NearestNeighborSearch() {}

	void buildIndex(const std::vector<Eigen::Vector3f>& targetPoints) {
		m_points = targetPoints;
	}

	std::vector<Match> queryMatches(const std::vector<Vector3f>& transformedPoints) {
		const unsigned nMatches = transformedPoints.size();
		std::vector<Match> matches(nMatches);
		const unsigned nTargetPoints = m_points.size();
		std::cout << "nMatches: " << nMatches << std::endl;
		std::cout << "nTargetPoints: " << nTargetPoints << std::endl;
		std::cout<<"BruteForce Matching"<<std::endl;

		measuredDists.clear();
		measuredDists.reserve(nMatches);

		// #pragma omp parallel for
		for (int i = 0; i < nMatches; i++) {
			matches[i] = getClosestPoint(transformedPoints[i]);
		}
		ave_dist/=counter_valid_point;

		return matches;
	}

	inline float get_aveDist(){
		return ave_dist;
	}

	inline float get_measuredMaxDist(){
		return measuredMax;
	}

	inline float get_measuredMinDist(){
		return measuredMin;
	}

	inline std::vector<float> get_measuredDists(){
		return measuredDists;
	}

private:
	std::vector<Eigen::Vector3f> m_points;
	float ave_dist = 0;
	int counter_valid_point = 0;
	float measuredMax = -1e23;
	float measuredMin = 1e23;
	std::vector<float> measuredDists;
	float current_dist = -1;

	Match getClosestPoint(const Vector3f& p) {
		int idx = -1;

		float minDist = std::numeric_limits<float>::max();
		for (unsigned int i = 0; i < m_points.size(); ++i) {
			float dist = (p - m_points[i]).norm();
			if (minDist > dist) {
				idx = i;
				minDist = dist;
			}
		}

		float _copy_dist = minDist;
		measuredDists.push_back(_copy_dist);
		// float exp_negative_dist = exp(_copy_dist*(-1));

		if (_copy_dist <= m_maxDistance){
			ave_dist+=_copy_dist;
			counter_valid_point++;
			if(_copy_dist <= measuredMin){
				measuredMin = _copy_dist;
			}
			if(_copy_dist >= measuredMax){
				measuredMax = _copy_dist;
			}

			return Match{ idx,  1.f};
		}
		else
			return Match{ -1, 0.f };
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

	std::vector<Match> queryMatches(const std::vector<Vector3f>& transformedPoints) {
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
		std::vector<Match> matches;
		matches.reserve(nMatches);

		float average_dist = 0;
		int counter_valid_point = 0;

		float min_dist = 1e10;
		float max_dist = (-1)*1e10;
		measuredDists.clear();
		measuredDists.reserve(nMatches);

		// std::cout<<"m_maxDistance: "<<m_maxDistance<<std::endl;
		for (int i = 0; i < nMatches; ++i) {
            auto dist = *distances[i];
			// std::cout<<i<<"th match distance: "<<dist<<std::endl;
			if (dist <= m_maxDistance){
				float exp_nagative_dist = exp(-1*dist);
				matches.push_back(Match{ *indices[i], exp_nagative_dist });
				average_dist+=dist;
				counter_valid_point++;
				if(dist <= min_dist){
					min_dist = dist;
				}
				
				if(dist >= max_dist){
					max_dist = dist;
				}
			}
			else{
				matches.push_back(Match{ -1, 0.f });
			}
			measuredDists.push_back(float(dist));
		}
		
		measuredMin = min_dist;			
		measuredMax = max_dist;
		ave_dist = average_dist/counter_valid_point;

		// Release the memory.
		delete[] query.ptr();
		delete[] indices.ptr();
		delete[] distances.ptr();

		return matches;
	}

	inline float get_aveDist(){
		return ave_dist;
	}

	inline float get_measuredMaxDist(){
		return measuredMax;
	}

	inline float get_measuredMinDist(){
		return measuredMin;
	}

	inline std::vector<float> get_measuredDists(){
		return measuredDists;
	}

private:
	int m_nTrees;
	flann::Index<flann::L2<float>>* m_index;
	float* m_flatPoints;
	float ave_dist;
	float measuredMax;
	float measuredMin;
	std::vector<float> measuredDists;
};