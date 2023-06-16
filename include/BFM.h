#include "Eigen.h"
#include "SimpleMesh.h"

class BFM{
public:
    bool readBFM(const std::string& filename) {
        return true;
    }

private:
    SimpleMesh averageFace;
    Eigen::MatrixXf principalShapeComponents;
    std::vector<float> shapeVariance;
    Eigen::MatrixXf principalTextureComponents;
    std::vector<float> textureVariance;
};