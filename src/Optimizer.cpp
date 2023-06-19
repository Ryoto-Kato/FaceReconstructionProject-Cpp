#include <ceres/ceres.h>

#include "../include/Eigen.h"
#include "../include/RGBImage.h"
#include "../include/SimpleMesh.h"

class GeometricConstraint {
public:
    // use RGBDImage class to save RGBD input?
    GeometricConstraint(const Matrix3f& intrinsic_, const Matrix4f& extrinsic_, const RGBDImage& image_, const float weightPoint_, const float weightPlane_) :
        intrinsic{ intrinsic_ },
        extrinsic{ extrinsic_ },
        image{ image_ },
        weightPoint{ weightPoint_ },
        weightPlane{ weightPlane_ }
    { }

    template <typename T>
    bool operator()(const T* parameters, T* residuals) const {
        // generateMesh(parameters) function to generate mesh from given paramters
        SimpleMesh templateMesh = generateMesh(parameters);
        // extract vertices from mesh
        std::vector<Vertex> templateVertices = templateMesh.GetVertices();
        // compute normals of mesh
        std::vector<Vector3f> templateNormals = templateMesh.getNormals();
        
        // implementation "reprojection" error
        int vertexIndex = 0;
        for(const auto& vertex : templateVertices) {
            Vector3f vertexPosWorld = vertex.position;
            Vector4f vertexPosWorldHom(4);
            vertexPosWorldHom << vertexPosWorld, 1.0f;
            
            // transform vertex coordinates from world to camera frame
            Vector4f vertexPosCameraHom = extrinsic * vertexPosWorldHom;
            Vector3f vertexPosCamera = vertexPosCameraHom.head(3);
            
            // project vertex into image
            Vector3f vertexPosImage = intrinsic * vertexPosCamera;
            float u = vertexPosImage[0] / vertexPosImage[2];
            float v = vertexPosImage[1] / vertexPosImage[2];

            // round computed pixel coordinate to integer and extract depth of pixel
            float depth = image.getDepth(round(u), round(v));
            // Reverse rounding or not?
            Vector3f vertexPosPixel(u, v, 1.0f);
            // backproject pixel back to world coordinates
            vertexPosPixel *= depth; 
            Vector3f pixelPosCamera = intrinsic.inverse() * vertexPosPixel;
            Vector4f pixelPosCameraHom(4);
            pixelPosCameraHom << pixelPosCamera, 1.0f;
            Vector4f pixelPosWorldHom = extrinsic.inverse() * pixelPosCameraHom;
            Vector3f pixelPosWorld = pixelPosWorldHom.head(3);

            Vector3f differences = vertexPosWorld - pixelPosWorld;
            // add point to point distances between template vertex and backprojected pixel to residuals
            residuals[4*vertexIndex] = T(std::sqrt(weightPoint)) * T(differences[0]);
            residuals[4*vertexIndex + 1] = T(std::sqrt(weightPoint)) * T(differences[1]);
            residuals[4*vertexIndex + 2] = T(std::sqrt(weightPoint)) * T(differences[2]);

            // add point to plane distances between template vertex and backprojected pixel to residuals
            Vector3f normal = templateNormals[vertexIndex];
            residuals[4*vertexIndex + 3] = T(std::sqrt(weightPlane)) * T(differences.dot(normal));

            vertexIndex++;
            return true;
        }
    }

protected:
    const Matrix3f intrinsic;
    const Matrix4f extrinsic;
    const RGBDImage image;
    const float weightPoint;
    const float weightPlane;
};

// TODO: write function to compute normals for each vertex of template mesh for point to plane cost function
//       (integrate in mesh class)
// TODO: outlier rejection (threshold distances/depth, normals)
// TODO: write generateMesh() function