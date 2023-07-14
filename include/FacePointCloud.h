#pragma once
#include "Eigen.h"

class FacePointCloud {
public:
	FacePointCloud() {}

	FacePointCloud(std::vector<Vector3f> & _vertices, std::vector<Vector3i> & _triangle_lists) {
		const auto& vertices = _vertices;
		const auto& triangles = _triangle_lists;
		m_triangle_lists = _triangle_lists;
		const unsigned nVertices = vertices.size();
		const unsigned nTriangles = triangles.size();

		// Copy vertices.
		m_points.reserve(nVertices);
		for (const auto& vertex : vertices) {
			m_points.push_back(vertex);
		}
		// Compute normals (as an average of triangle normals).
		m_normals = std::vector<Vector3f>(nVertices, Vector3f::Zero());
		for (size_t i = 0; i < nTriangles; i++) {
			const auto& triangle = triangles[i];
			int idx0 = triangle.x();
			int idx1 = triangle.y();
			int idx2 = triangle.z();
			// Vector3f faceNormal = (m_points[triangle.idx1] - m_points[triangle.idx0]).cross(m_points[triangle.idx2] - m_points[triangle.idx0]);
			Vector3f faceNormal = (m_points[idx1] - m_points[idx0]).cross(m_points[idx2] - m_points[idx0]);

			m_normals[idx0] += faceNormal;
			m_normals[idx1] += faceNormal;
			m_normals[idx2] += faceNormal;
		}
		for (size_t i = 0; i < nVertices; i++) {
			m_normals[i].normalize();
		}

		std::cout<<"finish face point cloud configration"<<std::endl;
	}

	FacePointCloud(MatrixRGB & colorMap, MatrixXf & depthMap, const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const unsigned width, const unsigned height, unsigned downsampleFactor, float maxDistance, float maxDepth, float minDepth) {
		// Get depth intrinsics.
		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);
		//max distance with adjacent neighbor
		const float maxDistanceHalved = maxDistance / 2.f;

		// Compute inverse depth extrinsics.
		Matrix4f depthExtrinsicsInv = depthExtrinsics.inverse();
		Matrix3f rotationInv = depthExtrinsicsInv.block(0, 0, 3, 3);
		Vector3f translationInv = depthExtrinsicsInv.block(0, 3, 3, 1);

		// Back-project the pixel depths into the camera space.
		std::vector<Vector3f> pointsTmp(width * height);
		std::vector<Vector3uc> colorsTmp(width * height);

		// For every pixel row.
		#pragma omp parallel for
		for (int v = 0; v < height; ++v) {
			// For every pixel in a column.
			for (int u = 0; u < width; ++u) {
				float depth = depthMap(u, v);
				unsigned int idx = v*width + u; // linearized index
				if (depth > maxDepth || depth < minDepth) {
					pointsTmp[idx] = Vector3f(MINF, MINF, MINF);
				}
				else {
					// Back-projection to camera space.
					pointsTmp[idx] = rotationInv * Vector3f((u - cX) / fovX * depth, (v - cY) / fovY * depth, depth) + translationInv;
				}
				colorsTmp[idx] = colorMap(u, v);
			}
		}

		// We need to compute derivatives and then the normalized normal vector (for valid pixels).
		std::vector<Vector3f> normalsTmp(width * height);

		#pragma omp parallel for
		for (int v = 1; v < height - 1; ++v) {
			for (int u = 1; u < width - 1; ++u) {
				unsigned int idx = v*width + u; // linearized index
				float du = 0.5f * (depthMap(u+1, v) - depthMap(u-1, v));
				float dv = 0.5f * (depthMap(u, v+1) - depthMap(u, v-1));
				if (!std::isfinite(du) || !std::isfinite(dv) || abs(du) > maxDistanceHalved || abs(dv) > maxDistanceHalved) {
					normalsTmp[idx] = Vector3f(MINF, MINF, MINF);
					continue;
				}

				// TODO: Compute the normals using central differences. 
				// back projected points coordinate
					// pointsTmp [idx] (Vector3f (Xc, Yc, Depth = depthMap[idx]))
					// Cross product
					/*
						Vector3f diff_u = pointsTmp[u+1][v] - pointsTmp[u-1][v];
						Vector3f diff_v = pointsTmp[u][v+1] - pointsTmp[u][v-1];
						normalsTmp = diff_u.cross(diff_v);
					*/
				// normal by cross product
				unsigned int index_ut = v*width + (u+1);
				unsigned int index_uo = v*width + (u-1);
				unsigned int index_vt = (v+1)*width + u;
				unsigned int index_vo = (v-1)*width + u;
				Vector3f diff_u = 0.5 * (pointsTmp[index_ut] - pointsTmp[index_uo]);
				Vector3f diff_v = 0.5 * (pointsTmp[index_vt] - pointsTmp[index_vo]);
				if(diff_u.allFinite() && diff_v.allFinite()){
					normalsTmp[idx] = diff_u.cross(diff_v);
					normalsTmp[idx].normalize();
				}else{
					normalsTmp[idx] = Vector3f(MINF, MINF, MINF);
				}
			}
		}

		// We set invalid normals for border regions.
		for (int u = 0; u < width; ++u) {
			normalsTmp[u] = Vector3f(MINF, MINF, MINF);
			normalsTmp[u + (height - 1) * width] = Vector3f(MINF, MINF, MINF);
		}
		for (int v = 0; v < height; ++v) {
			normalsTmp[v * width] = Vector3f(MINF, MINF, MINF);
			normalsTmp[(width - 1) + v * width] = Vector3f(MINF, MINF, MINF);
		}

		// set the normal to MatrixNormalMap

		MatrixNormalMap _normalMap(height, width);

		for(unsigned int v = 0; v < height; v++){
			for(unsigned int u = 0; u < width; u++){
				_normalMap(u, v) = normalsTmp[v*width+u];
			}
		}

		setNormalMap(_normalMap);

		// We filter out measurements where either point or normal is invalid.
		const unsigned nPoints = pointsTmp.size();
		m_points.reserve(std::floor(float(nPoints) / downsampleFactor));
		m_normals.reserve(std::floor(float(nPoints) / downsampleFactor));
		m_points_color.reserve(std::floor(float(nPoints) / downsampleFactor));

		for (int i = 0; i < nPoints; i = i + downsampleFactor) {
			const auto& point = pointsTmp[i];
			const auto& normal = normalsTmp[i];
			const auto& color = colorsTmp[i];

			if (point.allFinite() && normal.allFinite() && color.allFinite()) {
				m_points.push_back(point);
				m_normals.push_back(normal);
				m_points_color.push_back(color);
			}
		}
	}

	bool readFromFile(const std::string& filename) {
		std::ifstream is(filename, std::ios::in | std::ios::binary);
		if (!is.is_open()) {
			std::cout << "ERROR: unable to read input file!" << std::endl;
			return false;
		}

		char nBytes;
		is.read(&nBytes, sizeof(char));

		unsigned int n;
		is.read((char*)&n, sizeof(unsigned int));

		if (nBytes == sizeof(float)) {
			float* ps = new float[3 * n];

			is.read((char*)ps, 3 * sizeof(float) * n);

			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
				m_points.push_back(p);
			}

			is.read((char*)ps, 3 * sizeof(float) * n);
			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
				m_normals.push_back(p);
			}

			delete ps;
		}
		else {
			double* ps = new double[3 * n];

			is.read((char*)ps, 3 * sizeof(double) * n);

			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
				m_points.push_back(p);
			}

			is.read((char*)ps, 3 * sizeof(double) * n);

			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
				m_normals.push_back(p);
			}

			delete ps;
		}


		//std::ofstream file("pointcloud.off");
		//file << "OFF" << std::endl;
		//file << m_points.size() << " 0 0" << std::endl;
		//for(unsigned int i=0; i<m_points.size(); ++i)
		//	file << m_points[i].x() << " " << m_points[i].y() << " " << m_points[i].z() << std::endl;
		//file.close();

		return true;
	}

	std::vector<Vector3d> convert_float2double(std::vector<Vector3f> & float_vector){
		int length = float_vector.size();
		std::vector<Vector3d> double_vector;
		for(int i = 0; i<length; i++){
			Vector3d _temp = float_vector[i].cast<double>();
			double_vector.push_back(_temp);
		}
		return double_vector;
	}

	std::vector<Vector3f>& getPoints() {
		return m_points;
	}

	std::vector<Vector3d>& getPoints_double() {
		std::vector<Vector3d> _double_m_points = convert_float2double(m_points);
		return _double_m_points;
	}

	const std::vector<Vector3f>& getPoints() const {
		return m_points;
	}

	std::vector<Vector3f>& getNormals() {
		return m_normals;
	}

	std::vector<Vector3d>& getNormals_double() {
		std::vector<Vector3d> _double_m_normals = convert_float2double(m_normals);
		return _double_m_normals;
	}

	const std::vector<Vector3f>& getNormals() const {
		return m_normals;
	}

	std::vector<Vector3i>& getTriangleLists(){
		return m_triangle_lists;
	}

	const std::vector<Vector3i>& getTriangleLists() const{
		return m_triangle_lists;
	}

	unsigned int getClosestPoint(Vector3f& p) {
		unsigned int idx = 0;

		float min_dist = std::numeric_limits<float>::max();
		for (unsigned int i = 0; i < m_points.size(); ++i) {
			float dist = (p - m_points[i]).norm();
			if (min_dist > dist) {
				idx = i;
				min_dist = dist;
			}
		}

		return idx;
	}

	void print_points_and_normals(){
		std::cout<<"Number of points: "<<m_points.size()<<std::endl;
		std::cout<<"Number of normals at points: "<<m_normals.size()<<std::endl;
		unsigned int num_point = m_points.size();
		std::cout<<"point coordinate, \tnormal vector"<<std::endl;
		for(unsigned int i = 0; i < num_point; i++){
			std::cout<<i<<": "<<m_points[i].transpose()<<",\t"<<m_normals[i].transpose()<<std::endl;
		}

	}

	MatrixNormalMap getNormalMap(){
		return NormalMap;
	}

	void setNormalMap(MatrixNormalMap & _normalMap){
		NormalMap = _normalMap;
	}

	std::vector<Vector3f>& get_selectedVertexPos(std::vector<int> & bfm_landmarks_index_list){
		const auto & blil = bfm_landmarks_index_list;
		const unsigned length = blil.size();
		m_Landmark_points.reserve(length);
		for(int item : blil){
			m_Landmark_points.push_back(m_points[item]);
		}
		return m_Landmark_points;
	}

	std::vector<Vector3f>& get_selectedNormals(std::vector<int> & bfm_landmarks_index_list){
		const auto & blil = bfm_landmarks_index_list;
		const unsigned length = blil.size();
		m_Landmark_normals.reserve(length);
		for(int item : blil){
			m_Landmark_normals.push_back(m_normals[item]);
		}
		return m_Landmark_normals;
	}


    static void writeFacePointCloudPly(std::string fn, std::vector<Vector3f> & point_clouds){
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

        out.close();
        
        std::cout<<"Finish face mesh ply ("<<fn<<")"<<std::endl;
    }

 	void writeDepthMapPly(std::string fn, double scale){
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
        out << "element vertex " << m_points.size() << "\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
		out << "property uchar red\n";
		out << "property uchar green\n";
		out << "property uchar blue\n";
        out << "end_header\n";

        unsigned int cnt = 0;

        for (Vector3f point : m_points)
        {
			if(point.allFinite()){
            	out<<point.x()<<" "<<point.y()<<" "<<point.z()<<" "<<(int)m_points_color[cnt].x()<<" "<<(int)m_points_color[cnt].y()<<" "<<(int)m_points_color[cnt].z()<<"\n";
			}else{
            	out<<"0.0 0.0 0.0 0 0 0"<<"\n";
			}

			cnt++;
        }

        out.close();
        
        std::cout<<"Finish face mesh ply ("<<fn<<")"<<std::endl;
    }


private:
	std::vector<Vector3f> m_points;
	std::vector<Vector3uc> m_points_color;
	std::vector<Vector3f> m_normals;
	std::vector<Vector3f> m_Landmark_points;
	std::vector<Vector3f> m_Landmark_normals;
	std::vector<Vector3i> m_triangle_lists;
	MatrixNormalMap NormalMap;
};
