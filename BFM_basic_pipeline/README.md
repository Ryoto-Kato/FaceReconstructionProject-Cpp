# BFM_basic_pipeline

A basic pipeline to create Basel Face Model (BFM) mesh given coefficients in C++. 

## Depend on:

* Basel Face Model (version=2017);
  * Download 2017 .h5 from the Basel Face Model [[here]](https://faces.dmi.unibas.ch/bfm/bfm2017.html)
  * location to save,
    *  ./BFM_basic_pipeline/Data/model2017-1_bfm_nomouth.h5
* Eigen
* glog
* [HDF5](https://www.hdfgroup.org/downloads/hdf5) (version=1.12);
* [Dlib](http://dlib.net/) (version=19.24.99);
* [OpenCV4](https://github.com/opencv/opencv) with [opencv_contrib](https://github.com/opencv/opencv_contrib)


## How to use:
glog will output log of the process

            mkdir build
            cd build
            cmake ..
            cmake --build .
            ./gen_faces > log.txt


