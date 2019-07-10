import numpy as np
from open3d import *

# open3D do not include basic 3d transformations
def scaMat(x, y, z):
    return np.array([[x, 0.0, 0.0, 0.0], [0.0, y, 0.0, 0.0], [0.0, 0.0, z, 0.0], [0.0, 0.0, 0.0, 1.0]])

def traMat(x, y, z):
    return np.array([[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]])

def rotxMat(thetha):
    c = np.cos(thetha)
    s = np.sin(thetha)
    return np.array([[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]])

def rotzMat(thetha):
    c = np.cos(thetha)
    s = np.sin(thetha)
    return np.array([[c, 0.0, s, 0.0], [0.0, 1.0, 0.0, 0.0], [-s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0]])

def rotyMat(thetha):
    c = np.cos(thetha)
    s = np.sin(thetha)
    return np.array([[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

mesh_car = create_mesh_box(width = 2.71, height = 1.6, depth = 0.2)
mesh_car.paint_uniform_color([0.0, 0.0, 1.0])
mesh_car.transform(traMat(-2.71, -1.60, 0.3))

mesh_imu = create_mesh_coordinate_frame(size = 0.5, origin = [-2.76, -0.48, 0.93])

mesh_velo = create_mesh_coordinate_frame(size = 0.3, origin = [-2.76, -0.48, 0.93])
mesh_velo.transform(traMat(0.81, -0.32, 0.8))

mesh_cam0 = create_mesh_coordinate_frame(size = 0.05)
mesh_cam0.transform(rotxMat(np.radians(-90.0)))
mesh_cam0.transform(rotyMat(np.radians(-90.0)))
mesh_cam0.transform(traMat(-1.68, -0.80, 1.65))

mesh_cam1 = create_mesh_coordinate_frame(size = 0.05)
mesh_cam1.transform(rotxMat(np.radians(-90.0)))
mesh_cam1.transform(rotyMat(np.radians(-90.0)))
mesh_cam1.transform(traMat(-1.68, -0.80, 1.65))
mesh_cam1.transform(traMat(0.0, -0.54, 0.0))

mesh_cam2 = create_mesh_coordinate_frame(size = 0.05)
mesh_cam2.transform(rotxMat(np.radians(-90.0)))
mesh_cam2.transform(rotyMat(np.radians(-90.0)))
mesh_cam2.transform(traMat(-1.68, -0.80, 1.65))
mesh_cam2.transform(traMat(0.0, 0.06, 0.0))

mesh_cam3 = create_mesh_coordinate_frame(size = 0.05)
mesh_cam3.transform(rotxMat(np.radians(-90.0)))
mesh_cam3.transform(rotyMat(np.radians(-90.0)))
mesh_cam3.transform(traMat(-1.68, -0.80, 1.65))
mesh_cam3.transform(traMat(0.0, -0.48, 0.0))

pcd = read_point_cloud("BufPC.xyzrgb")
pcd.transform(traMat(-1.68, -0.80, 1.65))

# Uncomment following line to hide the lidar point cloud
# pcd.clear()

draw_geometries([mesh_car, mesh_imu, mesh_velo, mesh_cam0, mesh_cam1, mesh_cam2, mesh_cam3, pcd])