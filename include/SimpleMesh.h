#include <iostream>
#include <fstream>

#include "Eigen.h"

struct Vertex {
	Vector3f position;
    Vector3uc color;
};

struct Triangle {
	unsigned int idx0;
	unsigned int idx1;
	unsigned int idx2;

	Triangle() : idx0{ 0 }, idx1{ 0 }, idx2{ 0 } {}

	Triangle(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2) :
		idx0(_idx0), idx1(_idx1), idx2(_idx2) {}
};

class SimpleMesh {
public:
    std::vector<Vertex>& GetVertices() {
		return m_vertices;
	}

	std::vector<Triangle>& GetTriangles() {
		return m_triangles;
	}

	bool loadMesh(const std::string& filename) {
		// Read off file (Important: Only .off files are supported).
		m_vertices.clear();
		m_triangles.clear();

		std::ifstream file(filename);
		if (!file.is_open()) {
			std::cout << "Mesh file wasn't read successfully." << std::endl;
			return false;
		}

		// First line should say 'COFF'.
		char string1[5];
		file >> string1;

		// Read header.
		unsigned int numV = 0;
		unsigned int numP = 0;
		unsigned int numE = 0;
		file >> numV >> numP >> numE;

		m_vertices.reserve(numV);
		m_triangles.reserve(numP);

		// Read vertices.
		if (std::string(string1).compare("COFF") == 0) {
			// We have color information.
			for (unsigned int i = 0; i < numV; i++) {
				Vertex v;
				file >> v.position.x() >> v.position.y() >> v.position.z();
				// Colors are stored as integers. We need to convert them.
				Vector4i colorInt;
				file >> colorInt.x() >> colorInt.y() >> colorInt.z() >> colorInt.w();
				v.color = Vector3uc((unsigned char)colorInt.x(), (unsigned char)colorInt.y(), (unsigned char)colorInt.z());
				m_vertices.push_back(v);
			}
		}
		else if (std::string(string1).compare("OFF") == 0) {
			// We only have vertex information.
			for (unsigned int i = 0; i < numV; i++) {
				Vertex v;
				file >> v.position.x() >> v.position.y() >> v.position.z();
				v.color.x() = 0;
				v.color.y() = 0;
				v.color.z() = 0;
				m_vertices.push_back(v);
			}
		}
		else {
			std::cout << "Incorrect mesh file type." << std::endl;
			return false;
		}

		// Read faces (i.e. triangles).
		for (unsigned int i = 0; i < numP; i++) {
			unsigned int num_vs;
			file >> num_vs;
			ASSERT(num_vs == 3 && "We can only read triangular mesh.");
			
			Triangle t;
			file >> t.idx0 >> t.idx1 >> t.idx2;
			m_triangles.push_back(t);
		}

		return true;
	}
	
	bool writeMesh(const std::string& filename) {
		// Write off file.
		std::ofstream outFile(filename);
		if (!outFile.is_open()) return false;

		// Write header.
		outFile << "COFF" << std::endl;
		outFile << m_vertices.size() << " " << m_triangles.size() << " 0" << std::endl;

		// Save vertices.
		for (unsigned int i = 0; i < m_vertices.size(); i++) {
			const auto& vertex = m_vertices[i];
			if (vertex.position.allFinite())
				outFile << vertex.position.x() << " " << vertex.position.y() << " " << vertex.position.z() << " "
				<< int(vertex.color.x()) << " " << int(vertex.color.y()) << " " << int(vertex.color.z()) << std::endl;
			else
				outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
		}

		// Save faces.
		for (unsigned int i = 0; i < m_triangles.size(); i++) {
			outFile << "3 " << m_triangles[i].idx0 << " " << m_triangles[i].idx1 << " " << m_triangles[i].idx2 << std::endl;
		}

		// Close file.
		outFile.close();

		return true;
	}

private:
	std::vector<Vertex> m_vertices;
	std::vector<Triangle> m_triangles;
};

class PointCloud {
public:
	std::vector<Eigen::Vector3f>& GetPoints() {
		return m_points;
	}

	std::vector<Eigen::Vector3f>& GetNormals() {
		return m_normals;
	}

private:
	std::vector<Eigen::Vector3f> m_points;
	std::vector<Eigen::Vector3f> m_normals;
};