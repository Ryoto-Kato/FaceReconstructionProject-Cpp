# Facial Expression Transfer Design

## Face Reconstruction Data Flow
```mermaid
    flowchart TB
        subgraph "Dataset"
            Ref[(Reference Mesh*)]
            Depth[(Depth Range)]
            RGB[(RGB Image)]
        end
        subgraph FR["Face Reconstruction"]
            subgraph Fit["Face Model Parameter Estimation"]
                subgraph Loss["Loss Function"]
                    Point("Point to Point")
                    Plane("Point to Plane")
                    Color("Color")
                    Sparse("Sparse")
                    Reg("Regularization")
                end
                Depth-->Loss
                RGB-->Color
                Loss --> AbS("Analysis by Synthesis")
                AbS --"n times"--> Loss
            end
            RGB
            --> LD("Landmark Detector")
            --> BP("Back-projection")
            --> Procrustes("Procrustes")
            Depth --> BP
            BP --> Sparse
            subgraph PE["Pose Estimation"]
                    Procrustes
                    --> rICP("Rigid ICP")
                    --"n times"--> rICP
                    --> Loss
            end
            subgraph BFM["Basel Face Model"]
                PC("Principal Components")
                --> Loss
                Sigma("Standard Deviation \n of Parameters")
                --> Reg
                AF("Average Face Mesh")
                --> LE("Landmark Extractor")
                --> Procrustes
                AF --> Loss
            end
        end
        AbS --> MeshGen("Mesh Generation")
        PC --> MeshGen
        AF --> MeshGen
        MeshGen --> GEO
        MeshGen --> Rendering
        subgraph Visualization
            GEO("Geometric Error Heatmap")
            Ref --> GEO
            Rendering("Mesh Rendering")
            --> Overlay("RGB Image Overlay")
            RGB --> Overlay
        end
```
*If not available this can be substituted by a mesh created form the depth point cloud

## Extension for Expression Transfer

```mermaid
    flowchart TD
        subgraph "Source"
            Depth0[(Depth Range)]
            RGB0[(RGB Image)]
        end
        subgraph "Target"
            Depth1[(Depth Range)]
            RGB1[(RGB Image)]
        end
        Depth0 --> FR0
        RGB0 --> FR0
        FR0("Source Face Reconstruction**") --"Source face model"--> ET
        Depth1 --> FR1
        RGB1 --> FR1
        FR1("Target Face Reconstruction**") --"Target face model"--> ET
        ET("Expression Transfer")
        MeshGen("Mesh Generation")
        ET --"Target face model with expression parameters form source"--> MeshGen
        MeshGen --> Vis("Visualization / Rendering")
        RGB1 --> Vis
```
**Target/Source Face Reconstruction both refer to the Face Reconstruction process shown above
