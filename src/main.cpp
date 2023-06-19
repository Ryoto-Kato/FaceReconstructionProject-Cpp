#include <iostream>
#include <fstream>

#include "../include/Eigen.h"
#include "../include/SimpleMesh.h"

int main() { 
	const std::string filename = std::string("../Data/bunny.off");

	SimpleMesh sourceMesh;
	if (!sourceMesh.loadMesh(filename)) {
		std::cout << "Mesh file wasn't read successfully at location: " << filename << std::endl;
		return -1;
	}

	SimpleMesh resultingMesh = sourceMesh;
	resultingMesh.writeMesh(std::string("result.off"));
	std::cout << "Resulting mesh written." << std::endl;

    return 0;
}