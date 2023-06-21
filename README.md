# Organization
- .gitignore
  - If you want to ignore your own personal folder or files within this root directory, you can ignore them by rename it with "**personal**"

# Team
## Members [[team_list](https://docs.google.com/spreadsheets/d/1KcEIfJHHQVqG12gHMNQo_0s5yTAw82hNx3K4T5NtPMY/edit#gid=0)]

    - Ben Robert Sturgis	03776460	
    - Kevin	Qu	          03730587	
    - Marco	Busch	        03779574	
    - Ryoto	Kato	        03767467

# Abstract
Faithful 3D face reconstruction and the synthesis of reliable facial expressions are an essential foundation for the generation of photorealistic virtual avatars. The entertainment industry has addressed this task by having experienced artists manually create these avatars. While they are able to deliver impressive results, it is rather impractical because of the time- and labor-intensive nature of their work. In our project, we present an approach for 3D face reconstruction which does not require labor-intensive manual work. Based on a parametric face model, we perform an analysis-by-synthesis loop to reconstruct the 3D face from a single RGB-D input image. Furthermore, we apply an expression transfer between a source person and a target person.

# Tasks
- Week 1 (10.06 - 16.06)
  - [ ] Decided on final dataset, got access and familiarized ourselves with it
  - [ ] Completed implementation of input/output functions as well as data transformation and prepro-
  cessing
  - [ ] weekly report
- Week 2 (17.06 - 23.06)
  - [ ] Finished creation of a code template for orientation and better distribution of tasks, which we keep running during the entire development process
  - [ ] weekly report
- Week 3 (24.06 - 30.06)
  - [ ] Integrated suitable off-the-shelf facial landmark detector
  - [ ] Implemented Procrustes algorithm to get an initial coarse alignment of the face model and the input RGB-D image using the obtained landmarks
  - [ ] weekly report
- Week 4 (01.07 - 07.07)
  - [ ] Defined the energy function using the Ceres library
  - [ ] weekly report
- Week 5 (08.07 - 14.07)
  - [ ] Implemented the transfer of facial expressions of a source to a target person
  - [ ] weekly report
- Week 6 (15.07 - 21.07)
  - [ ] Fixed remaining bugs and code cleanup
  - [ ] Performed experiments to evaluate the developed face reconstruction
  - [ ] Optional: Integrated rendering pipeline using OpenGL
  - [ ] weekly report
- Week 7 (22.07 - 28.07)
  - [ ] Completion of the final report and presentation video
  - [ ] weekly report
- Week 8 (29.07 - 31.07)
  - [ ] Prepared for presentation
  - [ ] weekly report

# Requirement
- Datasets
  - RGB-D scans of humans faces
    - FaceWarehouse (Kinect RGB-D camera base)
      - 150 individuals
      - 7-80 from various ethnic backgrounds
      - 19 expressions + 1 neutral
      - Contents
        - raw data from the Kinect RGBD camera
        - reconstructed face mesh
        - labelled 2D landmarks for the first frame of raw data
        - Blendshapes
        - **NO Dataloader**
      - How to get data
        - (1) your name, title, affiliation (**if you are a student, please ask your advisor to contact us**)
        (2) your intended use of the data
        (3) a statement saying that you accept the following terms of licensing:
        The rights to copy, distribute, and use the data (including the RGBD images and 3D models) you are being given access to are under the control of Kun Zhou, head of the Graphics and Parallel Systems Lab, Zhejiang University. You are hereby given permission to copy this data in electronic or hardcopy form for your own scientific use and to distribute it for scientific use to colleagues within your research group. Inclusion of rendered images or video made from this data in a scholarly publication (printed or electronic) is also permitted. In this case, credit must be given to the publication: FaceWarehouse: a 3D Facial Expression Database for Visual Computing. However, the data may not be included in the electronic version of a publication, nor placed on the Internet. These restrictions apply to any representations (other than images or video) derived from the data, including but not limited to simplifications, remeshing, and the fitting of smooth surfaces. The making of physical replicas this data is prohibited, and the data may not be distributed to students in connection with a class. For any other use, including distribution outside your research group, written permission is required from Kun Zhou. Any commercial use of the data is prohibited. Commercial use includes but is not limited to sale of the data, derivatives, replicas, images, or video, inclusion in a product for sale, or inclusion in advertisements (printed or electronic), on commercially-oriented web sites, or in trade shows.
    - [IAS-LAB RGB-D Face Dataset](http://robotics.dei.unipd.it/reid/index.php/downloads)
        - Kinect RGB-D V2
        - Contents
          - Data Loader
          - 26 people in train and 19 people in test set
          - **No facial expression??**
          G. Pitteri, M. Munaro and E. Menegatti.
          "Depth-based frontal view generation for pose invariant face recognition with consumer RGB-D sensors".
          In Proceedings of the 14th International Conference on Intelligent Autonomous Systems (IAS-14), Shanghai, 2016.


  - Face model
    - Basel Face model 2017
      - provided
        - The average shape
        - The principal shape components
        - The shape variance
        - The mesh topology
        - The average texture
        - The principal texture components
        - The texture variance
      - 
- Libraries for facial landmark detection
  - CSIRO Face Analysis SDK 
    - Too old, 10 years ago
  - [**Dlib facial landmark**](http://dlib.net/compile.html)
    - Can be integrated in C++
    - Pre-trained facial landmark detection model
    - There is an sample program [here](http://dlib.net/face_landmark_detection_ex.cpp.html)
    - DEMO: Real-time facial landmark detection demo
      - check out /realtime_facial_landmark_detection_demo
  - Openpose
    - Not yet
  - MediaPipe
    - Python
    - We need to create an data streamer from python to our C++ (server.client)
  - OpenCV with a pre-trained model
    - Not yet

# Proposals (Deadline: June 9)
- Overleaf (CVPR)
- Time line
      1. Individually write respective section by Thursday 23:59
      2. Check them individually and note comments
      3. Meeting on Friday
      4. Finalize the proposal

# Weekly report (Every Friday)
- Format: Google Docs
- Content
- about 1 page
    - What you have accomplished in the
    week
    - Which problems did you encounter
    - If possible, show some intermediate
    results
    - Outline plan for the next week

# Final Report (Deadline: July 28)
- Content
  - 4 pages including figures and tables, excluding references
  - Should be structured like a paper
    - Introduction (Motivation)
    - Related Work (What has been done before)
    - Method (What we did)
    - Results (Quantitative & Qualitative)
    - Conclusion (What was achieved, what can be achieved in future)
- Deadline
  - Friday July 28 23:59
