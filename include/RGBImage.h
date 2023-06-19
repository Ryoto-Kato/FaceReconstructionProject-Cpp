#include "Eigen.h"

class RGBImage {
public:
    RGBImage(int width, int height) : width_(width), height_(height) {
        rgbData_.resize(width * height);
    }

    int getWidth() const {
        return width_;
    }

    int getHeight() const {
        return height_;
    }

    void setPixel(int x, int y, const Vector3uc& rgb) {
        int index = y * width_ + x;
        rgbData_[index] = rgb;
    }

    const Vector3uc& getPixel(int x, int y) const {
        int index = y * width_ + x;
        return rgbData_[index];
    }

private:
    int width_;
    int height_;
    std::vector<Vector3uc> rgbData_;
};

class RGBDImage : public RGBImage {
public:
    RGBDImage(int width, int height) : RGBImage(width, height) {
        depthData_.resize(width * height);
    }

    void setDepth(int x, int y, float depth) {
        int index = y * getWidth() + x;
        depthData_[index] = depth;
    }

    float getDepth(int x, int y) const {
        int index = y * getWidth() + x;
        return depthData_[index];
    }

private:
    std::vector<float> depthData_;
};
