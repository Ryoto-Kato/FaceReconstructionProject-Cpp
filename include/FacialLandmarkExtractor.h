
#ifndef FACIALLANDMARKEXTRACTOR
#define FACIALLANDMARKEXTRACTOR

#include <Eigen.h>
#include <map>

#include <dlib/opencv.h>
// #include <opencv2/highgui.hpp> original
// since from openCV4 we need to read header directory under opencv2
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <opencv2/imgproc.hpp>


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

            original_num_landmarks = landmarks.size();

            return true;

        } else {
            std::cout << "Failed to load the image with filename: "  << file << std::endl;
            return false;
        }
    }

    std::vector<Eigen::Vector2i> get_landmarks()
    {
        return landmarks;
    }

    inline cv::Mat getRGB_cvMat(){
        return image;
    }

    inline void set_landmark_img(cv::Mat & landmark_im){
        landmark_image = landmark_im.clone();
    }

    cv::Mat get_landmarkImage(std::vector<Vector2i> & detected_pix_landmarks, bool _override_onSourceImage){
        int rows = image.rows;
        int cols = image.cols;
        cv::Mat _landmark_image = image.clone();
        // _landmark_image = cv::Scalar(255, 255, 255);
        int red = 0;
        int green = 255;
        int blue = 0;

        truncated_num_landmarks = detected_pix_landmarks.size();

        unsigned int counter = 0;
        for(auto & pix_landmark : detected_pix_landmarks){
            //point.x() = col
            //potin.y() = row
            _landmark_image.at<cv::Vec3b>(pix_landmark.y(), pix_landmark.x()) = cv::Vec3b(blue, green, red);
            counter++;
        }

        set_landmark_img(_landmark_image);

        return landmark_image;
    }

    void set_map_landmarkId2Vector2i(std::vector<Vector2i> & detected_pix_landmarks){
        unsigned int counter = 0;
        for(auto & im_pix_cds : detected_pix_landmarks){
            map_landmarkId2Vector2i[counter] = im_pix_cds;
            counter++;
        }
    }

    int get_key_byValue(Vector2i & value){
        int key = 0;
        for(auto & pair : map_landmarkId2Vector2i){
            if(pair.second == value){
                key = pair.first;
                break;
            }
        }
        return key;
    }

    static void draw_point(cv::Mat& img, cv::Point2f fp, cv::Scalar color )
    {
        cv::circle( img, fp, 2, color, cv::FILLED, cv::LINE_AA, 0 );
    }
    
    // Draw delaunay triangles
    static void draw_delaunay(cv::Mat& img, cv::Subdiv2D& subdiv)
    {
        cv::Scalar delaunay_color(255,255,255);
        std::vector<cv::Vec6f> triangleList;
        subdiv.getTriangleList(triangleList);
        std::vector<cv::Point> pt(3);
        cv::Size size = img.size();
        cv::Rect rect(0,0, size.width, size.height);
    
        for( size_t i = 0; i < triangleList.size(); i++ )
        {
            cv::Vec6f t = triangleList[i];
            pt[0] = cv::Point(std::round(t[0]), std::round(t[1]));
            pt[1] = cv::Point(std::round(t[2]), std::round(t[3]));
            pt[2] = cv::Point(std::round(t[4]), std::round(t[5]));
    
            // Draw rectangles completely inside the image.
            if ( rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
            {
                cv::line(img, pt[0], pt[1], delaunay_color, 1, cv::LINE_AA, 0);
                cv::line(img, pt[1], pt[2], delaunay_color, 1, cv::LINE_AA, 0);
                cv::line(img, pt[2], pt[0], delaunay_color, 1, cv::LINE_AA, 0);
            }
        }
    }


    std::vector<Vector3i> get_triList_landmarks(cv::Mat & landmark_cvMat, std::vector<Vector2i> & detected_pix_landmarks, bool _debug){
        set_map_landmarkId2Vector2i(detected_pix_landmarks);
        // for(unsigned int c = 0; c<truncated_num_landmarks; c++){
        //     std::cout<<"---------"<<c<<"---------"<<std::endl;
        //     std::cout<<map_landmarkId2Vector2i[c]<<std::endl;
        // }
        cv::Size size = landmark_cvMat.size();
        cv::Rect rect(0, 0, size.width, size.height);

        cv::Subdiv2D subdiv(rect);

        for(unsigned int it= 0; it<detected_pix_landmarks.size(); it++){
            cv::Point2f _pix_coords(detected_pix_landmarks[it].x(), detected_pix_landmarks[it].y());
            subdiv.insert(_pix_coords);
        }


        std::vector<cv::Vec6f> triangleList;
        subdiv.getTriangleList(triangleList);

        std::vector<Vector3i> triangle_id_List;

        for(auto & t : triangleList){
            Vector2i _p1;
            _p1.x() = t[0];
            _p1.y() = t[1];
            int _id_p1 = get_key_byValue(_p1); 
            Vector2i _p2;
            _p2.x() = t[2];
            _p2.y() = t[3];
            int _id_p2 = get_key_byValue(_p2); 
            Vector2i _p3;
            _p3.x() = t[4];
            _p3.y() = t[5];
            int _id_p3 = get_key_byValue(_p3); 
            Vector3i _tri_idSet = {_id_p1, _id_p2, _id_p3};
            triangle_id_List.push_back(_tri_idSet);
        }
        
        if(_debug){
            cv::Mat dup = landmark_cvMat.clone();
            draw_delaunay(dup, subdiv);
            std::cout<<"Write landmark image with delaunay triangulation"<<std::endl;
            cv::imwrite("Delaunay_tri_detected_landmark.png", dup);
            std::cout<<"Triangle id set"<<std::endl;
            for(auto & set : triangle_id_List){
                std::cout<<set[0]<<","<<set[1]<<","<<set[2]<<std::endl;
            }
        }
        return triangle_id_List;
    }


    MatrixRGB getRGB_EigenMat(){
        int rows = image.rows;
        int cols = image.cols;

        MatrixRGB mat_rgb(rows, cols);

        for(unsigned int r = 0; r < rows; r++){
            for(unsigned int c = 0; c<cols; c++){
                cv::Vec3b intensity = image.at<cv::Vec3b>(r, c);
                unsigned char blue = intensity.val[0];
                unsigned char green = intensity.val[1];
                unsigned char red = intensity.val[2];
                Vector3uc _pix;
                _pix.x() = red;
                _pix.y() = green;
                _pix.z() = blue;

                // c = u, r = v
                mat_rgb(c, r) = _pix;
            }
        }

        return mat_rgb;

    }

    MatrixRGBA getRGBA_EigenMat(){
        int rows = image.rows;
        int cols = image.cols;

        MatrixRGBA mat_rgba(rows, cols);

        for(unsigned int r = 0; r < rows; r++){
            for(unsigned int c = 0; c<cols; c++){
                cv::Vec3b intensity = image.at<cv::Vec3b>(r, c);
                unsigned char blue = intensity.val[0];
                unsigned char green = intensity.val[1];
                unsigned char red = intensity.val[2];
                Vector4uc _pix;
                unsigned int maximum = 255;
                _pix.x() = red;
                _pix.y() = green;
                _pix.z() = blue;
                _pix.w() = (unsigned char)maximum;
                // c = u, r = v
                mat_rgba(c, r) = _pix;
            }
        }

        return mat_rgba;

    }

private:
    cv::Mat image;
    cv::Mat landmark_image;
    unsigned int original_num_landmarks;
    unsigned int truncated_num_landmarks;
    dlib::frontal_face_detector detector;
    dlib::shape_predictor pose_model;
    std::vector<Eigen::Vector2i> landmarks;
    std::map<int, Vector2i> map_landmarkId2Vector2i;
};

#endif //FACIALLANDMARKEXTRACTOR
