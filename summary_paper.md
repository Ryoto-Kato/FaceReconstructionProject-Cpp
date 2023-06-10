# [Real-time Expression Transfer for Facial Reenactment](http://www.graphics.stanford.edu/~niessner/papers/2015/10face/thies2015realtime.pdf)
# Goal
- Real-time facial expressions transfer
# Contributions
  - Real-time transfer facial expression approach
  - Photo realistic re-rendering of facial deformations
# Requirement
- RGB-D input
  - A general environment with largely Lambertian surfaces and smoothly varying illumination
# Parametric Face model illumination (as prior for facial performance capture)
- A parametric face model that spans a PCA space (Linear in these three attributes)
  - Facial identities
  - Face poses
  - Corresponding Albedo (reflectance)
- Custom parametric face model by combining a morpharable model for identity and skin albedo
  - BLANZ, V., AND VETTER, T. 1999. A morphable model for the synthesis of 3d faces. In Proc. SIGGRAPH, ACM Press/Addison-Wesley Publishing Co., 187–194.
- With expression space of a blend shape model ()
  - Alexander et al . 2009; Cao et al . 2014b] 

- Lighting model with a Lambertian surface reflectance assumption to jointly estimate the environment lighting,

- Parametric face model
  - Mean: $\mathbf{\alpha}_{alb}$, Standard deviation: $\mathbf{\sigma}_{alb}$
  - Mean: $\mathbf{\alpha}_{id}$, Standard deviation: $\mathbf{\sigma}_{id}$
  - First 160 principal directions are used
  - Parameterized facial expression with 76 blendshapes
  - **Note that the identity is parameterized in PCA space with linearly independent components, while the expressions are represented by blendshapes that may be overcomplete**

# Image Formation model (all parameters to be estimated)
### Parameter $\mathbf{P}$
- 429 parameters in total
$$
\mathcal{P} = (\mathbf{\alpha}, \mathbf{\beta}, \mathbf{\delta}, \mathbf{\gamma}, \mathcal{R}, \mathbf{t})
$$
- for ith vertex 
$$
  \mathcal{M}_{geo}(\mathbf{\alpha},{\delta}) \in \mathbf{\real}^3 = \mathbf{\alpha_{id}} + E_{id} \mathbf{\alpha} + E_{exp} \mathbf{\delta} 
$$
$$

  \mathcal{M}_{alb}(\beta) \in \mathbf{\real}^3 = \mathbf{\alpha}_{alb} + E_{alb}\mathbf{\beta}
  
$$

- Average face
  - $\mathbf{\alpha}_{\mathbf{id}}$

- Identity
  - Basis 
    - $\mathbf{E}_\mathbf{id}$
  - Weights
    - $\mathbf{\alpha}$
    - $|\mathbf{\alpha}|$ = 160

- Albedo
  - Basis
    - $\mathbf{E}_\mathbf{alb}$
  - Weights
    - $\mathbf{\beta}$
    - $|\mathbf{\beta}|$ = 160

- Expression
  - Basis
    - $\mathbf{E}_\mathbf{exp}$
  - Weights
    - $\mathbf{\delta}$
    - $|\mathbf{\delta}|$ = 76

- Illumination
  - basis (kth Sherical Harmonics (SH) basis)
    - $\mathbf{y}_\mathbf{k}$
  - Weights
    - $\mathbf{\gamma}$
    - $|\mathbf{\gamma}|$ = 23

- Rigid transformation
  - 3DOF linear algebra (convert to SE(3) by Rodriguez formula)
  - 3DOF translation vector

- With this, we can define an image formation $\mathcal{S}(\mathcal{P})$

$$
\mathcal{P} = (\mathbf{\alpha}, \mathbf{\beta}, \mathbf{\delta}, \mathbf{\gamma}, \mathcal{R}, \mathbf{t})
$$
 
- by defining following camera projection model

## 6DOF model to RGB-D camera space rigid transformation
- We need to represent head pose and the camera projection onto the virtual image plane
- World coordinate frame is equivalent with RGB-D sensor and assume the camera to be calibrated
- The model-to-world transformation (model to the RGB-D camera space transformation) is give by

$$
\Phi(v) = \mathbf{R}v+\mathbf{t}
$$

- where $\mathbf{R}$ is parameterized using Euler angles and together with $\mathbf{t}$

## RGB-D camera intrinsic parameter
- This is calibrated beforehand
- Define a full perspective projection $\Pi$ that transforms the world coordinate (RGB-D camera frame) tot the image coordinates.

# Illumination model
- Underlying assumption such that the lighting is distant and that the surfaces in the scene are predominantly Lambertian,
- Use Spehrical Harmonics (SH) basis
  - a low dimensional representation of the incident illumination
  - The irradiance in a vertex with normal n and scalar albedo c is represented using b=3 bands of SHs for the incident illumination
$$
\mathcal{L}_(\gamma, n, c) = c \dot \sum_{k=1}^{b^2} \gamma_k y_k(n)
$$
- $y_k(n)$: is the k-th SH basis function
- $\gamma = (\gamma_1, .., \gamma_{b^2})$ (the SH coefficients)
- Since we assume that it is the distant light source, the irradiance is independent of the vertex position and only depends on the vertex normal and the albedo
- RGB channels are considered separately.
- Reference
  - Irradiance and albedo coefficient are RGB triples
  - RAMAMOORTHI, R., AND HANRAHAN, P. 2001. A signal-processing framework for inverse rendering. In Proc. SIGGRAPH,ACM, 117–128
  - M ¨ULLER, C. 1966. Spherical harmonics. Springer.



# Energy Formulation
- Linear least squares energy
  - unconstrained energy minimization problem in the unknowns $\mathcal{P}$
$$
 \mathbf{E}(\mathcal{P}) = \mathbf{E}_{\mathbf{emb}}(\mathcal{P}) + \omega_{col}\mathbf{E}_{\mathbf{col}}(\mathcal{P}) + \omega_{lan}\mathbf{E}_{\mathbf{lan}}(\mathcal{P}) + \omega_{reg}\mathbf{E}_{\mathbf{reg}}(\mathcal{P})
$$

- Geometric error (in 3D coordinate): $\mathbf{E}_{\mathbf{emb}}(\mathcal{P})$
- Photometric error (on 2d image plane): $\mathbf{E}_{\mathbf{col}}(\mathcal{P})$
- Landmark error (Landmark space): $\mathbf{E}_{\mathbf{lan}}(\mathcal{P})$
- Regularization term: $\mathbf{E}_{\mathbf{reg}}(\mathcal{P})$

## Geometric Error (Geometry Consistency Metric)
- **Goal**: measure errors between the reconstructed geometry of virtual face and RGB-D input

$$
\mathbf{E}_{\mathbf{emb}}(\mathcal{P}) = \omega_{point}\mathbf{E}_{\mathbf{point}}(\mathcal{P}) + \omega_{plane}\mathbf{E}_{\mathbf{plane}}(\mathcal{P})
$$

- $\omega_{plane} > \omega_{point}$ (e.g, 1.0 > 0.1), give weight to the point-to-plan error to minimize parameters strictly for the point-to-plane term

### Point to point error (Sum of squared 2d residuals)

$$
\mathbf{E}_{\mathbf{point}}(\mathcal{P}) = \sum_{\mathbf{p}\in\mathcal{V}} ||d_{point}(\mathbf{p})||_2^2

$$
$$
d_{point}(\mathbf{p}) = X_\mathcal{s}(\mathbf{p}) - X_\mathcal{\mathcal{I}}(\mathbf{p})

$$

X_\mathcal{s}(\mathbf{p}) : \text{observed 3D point vertex position}

$$
X_\mathcal{I}(\mathbf{p}) : \text{observed 3D point vertex position}

$$

### Point to plane error

$$
E_{plane}(\mathcal{P}) = \sum_{\mathbf{p}\in\mathcal{V}} [d^2_{plane}(N_s(\mathbf{p}), \mathbf{p})+d^2_{plane}(N_{\mathcal{I}}(\mathbf{p}), \mathbf{p})]
$$
$$
N_s(\mathbf{p}):\text{normal vector of p}
$$
$$
d_{plane}(N_s(\mathbf{p}), \mathbf{p}) = N_s(\mathbf{p})^Td_{point}(\mathbf{p})
$$

## Image space Error (Color Consistency Metric)
$$
\mathbf{E}_{col}(\mathcal{P})=\sum_{\mathcal{p} \in \mathcal{V}} ||C_s(\mathbf{p}) - C_\mathcal{I}(\mathbf{p})||_2^2

$$
C_s(\mathbf{p}): \text{reconstructed image}
$$
C_\mathcal{I}(\mathbf{p}): \text{ground truth}
$$

## Landmark space Error (Feature Similarity Metric)
- Each detected feature $\mathcal{f}_j = (u_j, v_j)$ is a 2D location in the image domain that corresponds to a consistent 3D vertex v_j in our geometric face model (only 38 landmarksManual process)
- manually selected 38 landmarks are concentrated in the mouth, eye, and nose regions of the face,
- Prune landmarks with low **Visibility** and assign a confidence $\omega_{conf}$ based on its trustworthiness
      
- Where $\mathcal{F}$ is the set of detected features in each RGB input frame, we can define a metric that enforces facial features in the synthesized views to be close to the detected features

$$
E_{lan}(\mathcal{P}) = \sum_{\mathbf{f}_j \in \mathcal{F}} \omega_{conf,j} ||\mathcal{f_j} - \Pi(\Phi(v_j))||_2^2
$$

## Regularization Constraints
- Under the assumption of Gaussian distributed parameters, the interval [-3$\sigma_{params, i}$,3$\sigma_{params, i}$]
- params: model parameters
  - $\alpha, \beta, \text{and},   \delta$

$$
E_{reg}(\mathcal{P}) = \sum_{i=1}^{160}[(\frac{\mathbf{\alpha}_i}{\sigma_{id, i}})^2+(\frac{\mathbf{\beta}_i}{\sigma_{alb, i}})^2] + \sum_{i=1}^{76}(\frac{\mathbf{\delta}_i}{\sigma_{exp, i}})^2$$

- standard deviation
  - $\sigma_{exp, i}$ was fixed to one
  - $\sigma_{params, i}$:other standard deviation is derived by the PCA of the scanned data
    - Custom parametric face model by combining a morpharable model for identity and skin albedo
      - BLANZ, V., AND VETTER, T. 1999. A morphable model for the synthesis of 3d faces. In Proc. SIGGRAPH, ACM Press/Addison-Wesley Publishing Co., 187–194.
    - With expression space of a blend shape model ()
      - Alexander et al . 2009; Cao et al . 2014b] 

# System procedure
1. After the recording commences (with in only a few seconds long), the process to find good initialization for the optimization,
        
        - Identity
        - Head pose
        - Expression
        - Skin albedo
        - Incident illumination
        
!! facial expression is not necessary but we can easily create the personalized expression basis from this calibration

- Almost all parameters are estimate by minimizing the energy function on given RGB-D frame to determine the parameter $\mathbf{P}$ that faithfully reproduce the observed face in each RGB-D input frame
-  
1. Run time
  - Fix Albedo and Identity (modify the above parameters from the good initialization)
  - Estimate head, pose, facial expression and incident 
  - Re-render the face back into the underlying input video stream
  - By modifying the different model parameters on the fly, a variety of video modification application become feasible
  - lighting for all subsequent frames at real-time rates


# Parameter optimization
- Goal: obtain the best fitting parameters $\mathcal{P}$ to the given RGB-D camera recording. The best optimized parameters can describe the target person's facial expression precisely

## Analysis through synthesis approach
- Render the result given potentially non-optimal parameters and compare with ground truth every iteration (RGB-D input)

- 

## Expression Transfer
- Just substitut $\delta_{s}$ by the optimized $\delta_{s}$ for the source target.

$$
\mathcal{M}_{geo}(\alpha_t, \delta_s) = \mathbf{\alpha}_{id} + E_{id}\mathbf{\alpha_{t}} + E_{exp}\mathbf{\delta_{s}}
$$

        # idea (optional)
          - Relatively easily we can perform some experiment with another ideas for facial expression transfer.
          - We can present a personalized E_exp for the more reliable expression
          - now we just reconstruct face based on the average facial expression basis and source actor coefficient.
          - Usually we have different way to make facial expression.
          - Amount of change or direction of change for a facial expression is not identical at all
 
# Input data (RGB-D ground truth)
- RGB-D input
  - camera sequence $C_\mathcal{I}$
  - depth sequence $X_\mathcal{I}$
  - Underlying assumption
    - Depth data and color value are aligned in image space
    - They have correspondence by index (u, v) of pixel coordinate
- Normal field of input depth geometry
  - Obtain by the cross product of the partial derivatives of $X_{\mathcal{I}}$

# Tracking
- CSIRO Face Analysis SDK
  - https://github.com/ci2cv/face-analysis-sdk/blob/master/doc/documentation.org
  - SDK in C++ is provided

        ## recommended OpenCV build config
        cmake -DCMAKE_BUILD_TYPE=Release \
        -DENABLE_AVX=ON \
        -DENABLE_FAST_MATH=ON \
        -DENABLE_SSE=ON \
        -DENABLE_SSE2=ON \
        -DENABLE_SSE3=ON \
        -DENABLE_SSE41=ON \
        -DENABLE_SSE42=ON \
        -DENABLE_SSSE3=ON \
        /path/to/opencv/

- Input: RGB image
- Output: 66 2D image landmarks,
![](\images/CSIRO_landmark.png)

# Technical Approach
- Jointly fit a parametric face model for
  - Identity
  - Expression
  - Skin reflectance
  - Reconstruct scene lighting
- Pre-computation
  - Obtain **Identity** and **Albedo**
  - Create personalized model of the actor
  - 
- In real-time
  - Capturing both source and target face
  - Derive facial expression parameters of source
  - Map the expressions from the source to the target actor
  - New Analysis-through-synthesis approach
    - Minimizing a energy function which is jointly optimized in the unknown head-pose, face identity, facial expression, face albedo, and incident illumination

<!-- 
## 4.1 Parametric Face model
- $\mathcal{M}_{geo}(\mathbf{\alpha},{\delta})$ parameterized the face geometry by means of a set of dimensions encoding the identity with weights $\mathbf{\alpha}$ and encoding the facial expression with weights $\mathbf{\delta}$.
- $\mathcal{M}_{alb}(\mathbf{\beta})$ parameterized the skin albedo
- for ith vertex 
  $$
  \mathcal{M}_{geo}(\mathbf{\alpha},{\delta}) \in \mathbf{\real}^3 = \mathbf{\alpha_{id}} + E_{id} \mathbf{\alpha} + E_{exp} \mathbf{\delta} 

  \\

  \mathcal{M}_{alb}(\beta) \in \mathbf{\real}^3 = \mathbf{\alpha}_{alb} + E_{alb}\mathbf{\beta}
  
  \\

  **E_{id}, E_{exp}, E_{alb}: \text{basis vectors of the linear subspaces}**

  \\
  E_{id}= [{\mathbf{e_{id1}} \in \real^{3}, \mathbf{e_{id2}}, ..., \mathbf{e_{id-D}}}]\in \mathbf{\real}^{3D}

  \\

  **\mathbf{\alpha}, \mathbf{\beta}, \mathbf{\delta}: \text{coefficient (weights) of each noise [0, std]}
  **
  \\
  \mathbf{\alpha} \in \real^D

  \\

  \mathbf{\alpha_{id}}: \text{average identity face}

  \\

  \mathbf{\alpha_{alb}}: \text{average skin albedo}

  $$


# 5. Parametric model Fitting
- For the simultaneous estimation of the identity, facial expression, skin albedo, and scene lighting and head pose, we fit our image formulation model $\mathcal{S}_(\mathcal{P})$ to the input of a commodity RGB-D camera recording an actor's performance. -->
