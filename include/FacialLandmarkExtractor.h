
#ifndef FACIALLANDMARKEXTRACTOR
#define FACIALLANDMARKEXTRACTOR

#include <Eigen.h>

#include <dlib/opencv.h>
// #include <opencv2/highgui.hpp> original
// since from openCV4 we need to read header directory under opencv2
#include <opencv2/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>


// You can get this file from http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
const std::string MODELL_PATH = "../models/dlib/shape_predictor_68_face_landmarks.dat";

class FacialLandmarkExtractor
{
public:
    FacialLandmarkExtractor()
    {
        // Load face detection and pose estimation models.
        detector = dlib::get_frontal_face_detector();
        dlib::deserialize(MODELL_PATH) >> pose_model;
    };

    bool process_image(const std::string& file)
    {
        image = cv::imread(file);

        // Check if the image was successfully loaded
        if (!image.empty()) {

            dlib::cv_image<dlib::bgr_pixel> cimg(image);

            // Detect faces
            std::vector<dlib::rectangle> faces = detector(cimg);

            if (faces.size() == 0){
                std::cout << "Failed to find face in image"  << file << std::endl;
                return false;
            }

            if (faces.size() > 1){
                std::cout << "Multiple faces found, only the one with index zero will be used"  << file << std::endl;
            }

            // Get landmarks for face
            dlib::full_object_detection shape = pose_model(cimg, faces[0]);

            for (size_t i = 0; i < shape.num_parts(); ++i)
            {
                const auto& point = shape.part(i);
                landmarks.push_back(Eigen::Vector2i(point.x(), point.y()));
            }

            return true;

        } else {
            std::cout << "Failed to load the image with filename: "  << file << std::endl;
            return false;
        }
    }

    std::vector<Eigen::Vector2i> get_landmarks()
    {
        return landmarks;
    };

private:
    cv::Mat image;
    dlib::frontal_face_detector detector;
    dlib::shape_predictor pose_model;
    std::vector<Eigen::Vector2i> landmarks;
};

#endif //FACIALLANDMARKEXTRACTOR
