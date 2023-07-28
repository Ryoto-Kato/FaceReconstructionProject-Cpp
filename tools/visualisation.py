import numpy as np
import sys
import trimesh
import pyrender
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from PIL import Image

# read args
try:
    person_id = sys.argv[1]
except:
    person_id = 42

try:
    expression = sys.argv[2]
except:
    expression = "Smile"

try:
    save_figure = sys.argv[3]
except:
    save_figure = False

uf = 2 # upscale factor (rendering_res = image_res * upscale factor)
img_dim = (256, 256)*uf

# Load data
# bfm_trimesh = trimesh.load('../output/after_ColorparamEst.ply')
# face reenactment result
bfm_trimesh = trimesh.load('../output/after_paramEst_FACEREANACMENT.ply')
bfm_errormap_trimesh = trimesh = trimesh.load('../output/after_paramEst_errorMap.ply')
image_path = '../data/EURECOM_Kinect_Face_Dataset/' + str(person_id).zfill(4) + '/s1/RGB/rgb_' + str(person_id).zfill(4) + '_s1_' + expression + '.bmp'
img = np.asarray(Image.open(image_path).resize(img_dim, resample=Image.BOX))

# Define camera model
camera_pose = np.array([[1,  0,  0,  0],
                        [0, -1,  0,  0],
                        [0,  0, -1,  0],
                        [0,  0,  0,  1],])
camera = pyrender.IntrinsicsCamera(fx = 525*uf, fy = 525*uf, cx = 127.5*uf, cy = 165.5*uf, zfar=2000*uf)

# Render bfm mesh (We are no shading the face model here, just take color ov vertex)
mesh = pyrender.Mesh.from_trimesh(bfm_trimesh)
scene = pyrender.Scene(ambient_light=np.zeros(3), bg_color=[1.0, 1.0, 1,0])
scene.add(mesh)
scene.add(camera, pose=camera_pose)
r = pyrender.OffscreenRenderer(img_dim[0], img_dim[1])
color, _ = r.render(scene, flags = pyrender.RenderFlags.RGBA | pyrender.RenderFlags.FLAT)
# make white background pixel transparent
bfm_img = color.copy()
for i in range(img_dim[0]):
    for j in range(img_dim[1]):
        if np.array_equal(bfm_img[i,j], np.array([255, 255, 255, 255])):
            bfm_img[i,j,3] = 0

# generate image for geometric error map
mesh = pyrender.Mesh.from_trimesh(bfm_errormap_trimesh)
scene = pyrender.Scene()
scene.add(mesh)
scene.add(camera, pose=camera_pose)
r = pyrender.OffscreenRenderer(img_dim[0], img_dim[1])
geometric_error_map_img , _ = r.render(scene, flags = pyrender.RenderFlags.FLAT)

# Generate Plot
plt.figure(figsize=(24, 8), dpi=32)
plt.rcParams.update({'font.size': 30})

# Subplot 1: RGB image
plt.subplot(1,3,1)
plt.axis('off')
plt.imshow(img)

# Subplot 2: Geometric error map
plt.subplot(1,3,2)
plt.axis('off')
# Define the custom colormap from red to blue
custom_cmap = mcolors.LinearSegmentedColormap.from_list('blue_to_red', ['blue', 'red'], N=100)
plt.imshow([[0, 4], [0, 4]], cmap = custom_cmap)
plt.colorbar(label = 'Geometric Error [mm]', ticks=[0,4], orientation = 'horizontal', pad= 0, fraction = 0.19, shrink=0.7)
plt.imshow(geometric_error_map_img[30:-50, 40:-40])

# Subplot 3: Reconstructed face model as overly on RGB image
plt.subplot(1,3,3)
plt.axis('off')
plt.imshow(img)
plt.imshow(bfm_img, interpolation='none')
plt.tight_layout(pad=0.0)

if save_figure:
    plt.savefig((str(person_id).zfill(4) + '_s1_' + expression + '_vis.png'), bbox_inches='tight', pad_inches=0.0)
else:
    plt.show()
