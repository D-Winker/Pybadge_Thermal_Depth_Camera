# Visualize
#
# Reads the files in the Data folder and projects them as thermal-colored points in space.
#
# Daniel Winker, May 19, 2024

import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize


# Read in data
thermalData = pd.read_csv("Data/MLX90640/frm00003.csv", header=None)
thermalData = thermalData.iloc[:, :-1]  # Each line of the file ends with a comma, so drop the last column (all NaNs)

depthData = pd.read_csv("Data/VL53L5CX/frm00003.csv", header=None)
depthData = depthData.iloc[:, :-1]  # Each line of the file ends with a comma, so drop the last column (all NaNs)
depthData = depthData.values  # Convert to numpy 
depthData = depthData.reshape((8,8,3))  # Reshape to be like the physical sensor
deepestDepth = np.max(depthData) / 1000 
largestSquare = 0

plt.imshow(depthData[:,:,0], cmap='Greys')
plt.show()

cubes = []
scene = o3d.t.geometry.RaycastingScene()
# Project squares in space at each measured depth position
# For 'create_box()' Front left, bottom corner placed at (0, 0, 0)
# Translate coords are (x, y, z) -> (side-to-side, up and down, depth)
for row in range(depthData.shape[0]):
    for col in range(depthData.shape[1]):
        for depth in range(depthData.shape[2]):
            if depth != 0:#DEBUG
                continue
            depth = depthData[col, row, depth] / 1000  # mm to m
            if depth == 0:
                continue

            # Calculate the dimensions (side length) of each square at the measured depth
            angle = 45 / 2
            sideLen = (depth * np.tan(angle * np.pi / 180)) / 4
            if sideLen > largestSquare:
                largestSquare = sideLen

            # Calculate (x, y) position of the corner of the zone
            # The distances returned are horizontal from the sensor, as opposed to following
            # the angled ray for a given zone (which would be longer)
            x = (row - 4) * sideLen
            y = (col - 4) * sideLen

            # Project the square in space
            cube = o3d.geometry.TriangleMesh.create_box(width=sideLen, height=sideLen, depth=0.01).translate([x, y, depth])
            cube = o3d.t.geometry.TriangleMesh.from_legacy(cube)
            _ = scene.add_triangles(cube)
            cubes.append(cube)

            #print(sideLen, x, y, depth)
        #print()

# Add in a background canvas. This is my easy, kludgey way of handling the thermal-data raycasts going beyond
# the borders of the projected depth scene. Without this, not all rays hit the scene, and I haven't perfectly
# aligned the thermal raycast with the depth projections (cubes), so I can't easily predict which points will be valid.
backgroundSize = largestSquare*20
cube = o3d.geometry.TriangleMesh.create_box(width=backgroundSize, height=backgroundSize, depth=0.01).translate([-backgroundSize/2, -backgroundSize/2, deepestDepth])
cube = o3d.t.geometry.TriangleMesh.from_legacy(cube)
_ = scene.add_triangles(cube)


# Depth camera raycasts. This isn't actually used anywhere...
rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
    fov_deg=45,
    center=[0, 0, 1],  # Where the camera is looking 
    eye=[0, 0, 0],  # Position of the camera
    up=[0, 1, 0],  # Up vector
    width_px=64,
    height_px=64,
)
# We can directly pass the rays tensor to the cast_rays function.
ans = scene.cast_rays(rays)

hit = ans['t_hit'].isfinite()
points = rays[hit][:,:3] + rays[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))
pcd = o3d.t.geometry.PointCloud(points)
pcd.paint_uniform_color((1.0, 0.0, 0.0))
# Press Ctrl/Cmd-C in the visualization window to copy the current viewpoint


rays2 = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
    fov_deg=38,
    center=[0, 0, 1],  # Where the camera is looking 
    eye=[-0.0, -0.0185, 0.05],  # Position of the thermal camera.
    # The thermal camera is 18.5 mm below the depth camera, and a bit in front of it. I messed with these values until things looked aligned...
    # I have written down that the camera is 32x24 pixels and 55x35 degrees, which gives a different angular spread, per pixel, 
    # vertically and horizontally.
    up=[0, 1, 0],  # Up vector
    width_px=32,
    height_px=32,
)
# We can directly pass the rays tensor to the cast_rays function.
ans2 = scene.cast_rays(rays2)

hit2 = ans2['t_hit'].isfinite()
points2 = rays2[hit2][:,:3] + rays2[hit2][:,3:]*ans2['t_hit'][hit2].reshape((-1,1))
pcd2 = o3d.t.geometry.PointCloud(points2)

# TODO: Make the points bigger
# TODO: Figure out the thermal-depth alignment issue
### Convert the temperature data into an RGB colors for the point cloud
colors = np.zeros((32*32, 3))
# Normalize the thermal data to the range [0, 1]
norm = Normalize(vmin=thermalData.min().min(), vmax=thermalData.max().max())
normalized_thermal_data = norm(thermalData.values)

cmap = plt.cm.get_cmap('inferno')  # Get the inferno colormap

rgb_data = cmap(normalized_thermal_data)[:, :, :3]  # Apply the colormap to get RGB values. Drop the alpha channel if present.
rgb_data = np.flip(rgb_data, axis=(0))  # Mirror the image vertically

plt.imshow(rgb_data)
plt.show()

# Assign colors to points. Remember that the 'camera' is 24x32 and the raycast is 32x32
# Rays appear to be cast from left to right, top to bottom (so that's also the point order)
for row in range(24):
    for col in range(32):
        colors[col+(row+4)*32] = rgb_data[row, col]


pcd2.point['colors'] = o3d.core.Tensor(colors, dtype=o3d.core.Dtype.Float32)

# Extract points and colors from pcd2
points_array = np.asarray(pcd2.point.positions.cpu().numpy())
colors_array = np.asarray(pcd2.point['colors'].cpu().numpy())

# Trim the first and last 4*32 points
trimmed_points = points_array[4*32:-4*32]
trimmed_colors = colors_array[4*32:-4*32]

# Create a new point cloud with the trimmed data
pcd2_trimmed = o3d.t.geometry.PointCloud(o3d.core.Tensor(trimmed_points, dtype=o3d.core.Dtype.Float32))
pcd2_trimmed.point['colors'] = o3d.core.Tensor(trimmed_colors, dtype=o3d.core.Dtype.Float32)

# Visualize the trimmed points
cubes = [pcd2_trimmed.to_legacy()]
o3d.visualization.draw_geometries(cubes,
                                  front=[0, 0, 1],
                                  lookat=[0, 0, 0],
                                  up=[0, 1, 0],
                                  zoom=1.0)
