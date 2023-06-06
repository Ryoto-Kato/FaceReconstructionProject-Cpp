
# Team
## Members [[team_list](https://docs.google.com/spreadsheets/d/1KcEIfJHHQVqG12gHMNQo_0s5yTAw82hNx3K4T5NtPMY/edit#gid=0)]

    - Ben Robert Sturgis	03776460	
    - Kevin	Qu	          03730587	
    - Marco	Busch	        03779574	
    - Ryoto	Kato	        03767467



# Tasks
- June 08
  - Complete drafts for proposal (individually each part of it)
- June 09
  - [x] Project topic
  - [x] Apply as a team via Google Form 
  - [ ] **Check drafts before meeting !**
  - [ ] Meeting 14:00-
  - [ ] Create proposal (Overleaf Latex)
  - [ ] Submit proposal
- June 16
  - [ ] weekly report
- June 23
  - [ ] weekly report
- June 30
  - [ ] weekly report
- July 7
  - [ ] weekly report
- July 14
  - [ ] weekly report
- July 21
  - [ ] weekly report
- **Friday July 28 23:59**
  - [ ] Final report
- **August 1**
  - [ ] Presentation
  - [ ] 5 mins videos

# Proposals (Deadline: June 9)
- Overleaf (CVPR)
- Time line

      1. Individually write respective section by Thursday 23:59
      2. Check them individually and note comments
      3. Meeting on Friday
      4. Finalize the proposal

## Abstract (Ryoto)
- Some abstraction of our proposal

## Technical Approach (Kevin)
  - How do you propose to solve it
  - We parameterize face model by $\mathbf{P}=(\Phi, \alpha, \beta, \delta, \gamma)$
  - Approach 1 (K-means clustering)
    - Apply K-means clustering on dataset
      - Obtain centroids (means of face vertices positions)
      - Obtain averaged $\mathbf{Eexp}$ per cluster
    - In preprocessing, we estimate $\Phi$ (Shape identity) and $\beta$ (Material/Albedo) as Face2Face does.
    - By using estimated parameters, find a closest cluster and obtain its centroid and $\mathbf{Eexp}$
    - Use them as the principal component of the facial expressions
    - Construct energy function
    - Parameter Estimation as energy minimization
    - perform **Analysis-by-Synthesis**

  - Approach 2 (Face skeletal structure descriptor)
    - By using detected facial landmarks, we obtain sparse and a low-dimensional representatives of target's face skeletal structure (facial mesh)
    - It can be used as input to get a descriptor of the identity.
    - Since we have vertex correspondences between space face meshes within database, we can use some metric which are used for energy minimization in Non-rigid 3D reconstruction (Mesh deformation)
    - As we measure deviation from rigidity by summing up deviation with respect to a fan, we can obtain "Face skeletal structure" difference between a target person and one from dataset
    
    $$\mathbf{E_{fss} = \sum_i\sum_{j\in\mathcal{N}(i)} ||(vi-vj)-(vi'-vj')||^2}$$
    $$\text{for ith vertex fan}$$

    - Matching strategy
      1. Target vs one from parametric model and try to find a face model which has the highest similarity
      2. Target vs average face descriptor from clusters and find a cluster which has the highest similarity
    - By matching, we will obtain the 
    - In preprocessing, we estimate $\Phi$ (Shape identity) and $\beta$ (Material/Albedo) as Face2Face does.
    - We derive a certain structure $\mathbf{Eexp}$ and its standard deviation by referring nearest neighboring face model $\mathbf{Eexp}$ and its standard deviation
  - ,
## Requirements (Marco)
  - Which datasets & libraries will you need ?
  - [Facial Landmark detection]
    - off-the-shelf
      - OpenPose
      - Media pipe
      - OpenCV
      - https://github.com/ci2cv/face-analysis-sdk
      - Dlib
  - [Dataset] 
    - https://github.com/cleardusk/3DDFA/tree/master
    - https://vcai.mpi-inf.mpg.de/projects/MonFaceCap/#dataset
    - Face warehouse(Face model (Scanned data), email required)
    - IASLAB-RGB D dataset(c++ library for visualization)
  - [Landmark](https://github.com/anilbas/BFMLandmarks)
  - Optimizer (ceres)
  - Eigen
  - OpenCV
  - OpenGL

## Milestones (Ben)
  - Time estimate, tasks
    - Decide dataset
    - Get data set
    - Get familiar with dataset
    - Data-loader function
    - Landmark detector
    - Check the landmark corresponding
    - Landmark-based Pose estimation (Procrustes w.r.t landmarks)
      - Backproject the landmark
      - Exercise1
      - Exercise3
      - Get pose of model
    - check the Camera intrinsic in face models (if we need)
    - Dense matching
    - Experiment
      - Dense, Sparse, and Albedo
        - RGB-D example input
        - Identity estimation
        - Expression estimation

    - PCA (if we do not have parametric model data base and only scanning data)
      - Align template to the every scanned face
      - PCA of the aligned face mesh
      - Obtain the parametric model
    - Face Landmark detection and data stream
    - Face template alignment
    - Face expression transfer
      - Second Proposal
        - Find personalized Facial expression deformation components
        - Personalized deformation coefficients
          - Goal: approximate plausible face deformation principal components for target face by referring neighboring face model in a certain feature space (Face identity itself or face skeletal structure)
    - Precomputing process: identify target RGB input Identity (Albedo and Identity) and Pose
    - Texture/Lighting
    - Rendering
  - 
- Team-Members
  - Group of 4
  


# Useful information 
- Basel Face model viewer (2019)
- Basel Face model 
  - https://faces.dmi.unibas.ch/bfm/bfm2017.html
- Key landmark detection (Dlib library) 

# Pipeline details
- Input :RGB-D data
  - Pre-computation
    - Pose estimate
  - Runtime
    - Other parameters estimation
- Output:
  - Reconstructed face model and estimate parameters
- Comparison:
  - Visualization of energy-term (vertex space distance)
  - Visualization of results
  - Visualization of expression transfer

# Reference
- @inproceedings{Mori2017TheUV,
  title={The Uncanny Valley: The Original Essay by Masahiro Mori-IEEE Spectrum},
  author={Masahiro Mori and Karl F. Macdorman},
  year={2017}
}

# Questions
- How can we obtain the dataset which is also used for the Face2Face
- We know that Face2Face used Facial Landmark detector but how can we obtain correspondence between face mesh template vertices and the detected landmark
- 

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

