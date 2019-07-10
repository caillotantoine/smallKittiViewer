import numpy as np
from open3d import *
from utils import *

frameIDX = 156

# Load kitti's dataset
basedir = '../Datasets'
date = '2011_09_26'
drive = '0009'
dataset = pykitti.raw(basedir, date, drive)

pc_lidar = dataset.get_velo(frameIDX)

img = np.array(dataset.get_rgb(frameIDX)[0])

P_rect = dataset.calib.P_rect_20
R_rect = dataset.calib.R_rect_20
T_velo = dataset.calib.T_cam2_velo

T_mat = np.dot(np.dot(P_rect, R_rect), T_velo)

print("Start writting buffer file")
f = open("BufPC_2.xyzrgb", "w")
for pts in pc_lidar:
    p = np.array([[pts[0]], [pts[1]], [pts[2]], [1]])
    P_proj = np.dot(T_mat, p)
    P_proj = P_proj / P_proj[2]
    P_proj = P_proj.astype(int)
    u = P_proj[0]
    v = P_proj[1]
    if P_proj[1] >= 0 and P_proj[1] < img.shape[0] and P_proj[0] >= 0 and P_proj[0] < img.shape[1] and pts[0] > 0:
        r = img[v, u, 0][0]/255.0
        g = img[v, u, 1][0]/255.0
        b = img[v, u, 2][0]/255.0
        f.write("{}\t {}\t {}\t {}\t {}\t {}\n".format(pts[0], pts[1], pts[2], r, g, b))
        # f.write("{}\t {}\t {}\t {}\t {}\t {}\n".format(pts[0], pts[1], pts[2], img[v, u, 0]/255.0, img[v, u, 1]/255.0, img[v, u, 2]/255.0))
    else:
        f.write("{}\t {}\t {}\t {}\t {}\t {}\n".format(pts[0], pts[1], pts[2], pts[3], pts[3], pts[3]))
f.close()
print("Writting done")

pcd = read_point_cloud("BufPC_2.xyzrgb")
pcd.transform(traMat(-1.68, -0.80, 1.65))

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

draw_geometries([mesh_car, mesh_imu, mesh_velo, mesh_cam0, mesh_cam1, mesh_cam2, mesh_cam3, pcd])
