# Real-time facial landmark detection demo

- Requirements (test environment)
  - Linux (manjaro)
  - C++ gcc compiler
    - g++ (GCC) 13.1.1
  - OpenCV 4.7.0 (Visualization)
    - build source, install it
    - with opencv_contrib

            $ git clone https://github.com/opencv/opencv.git
            $ git clone https://github.com/opencv/opencv_contrib.git


  - [Dlib](https://github.com/davisking/dlib) (Landmark detector)
    - build source, install it
  - Web camera (Input)
    - Default camera test
            
            #check the device list
            v4l2-ctl -d /dev/video0 --list-ctrls

            #config brightness
            v4l2-ctl -c brightness=128

            #local webcamera test
            ffplay /dev/video0 

- check out **CMakeLists.txt**

- example 

        mkdir build
        cd build
        cmake ..
        cmake --build . --config Release
        ./realtime_landmark_detection shape_predictor_68_face_landmarks.dat


 