import numpy as np
from open3d import *
from utils import *

frameIDX = 100

# Load kitti's dataset
basedir = '../Datasets'
date = '2011_09_26'
drive = '0009'
dataset = pykitti.raw(basedir, date, drive)

# Load lidar data
pc_lidar = dataset.get_velo(frameIDX) 

# Create a file since open3D seems to be able to get points cloud from file only
print("Start writting buffer file")
f = open("BufPC.xyzrgb", "w")
for i in pc_lidar:
    f.write("{}\t {}\t {}\t {}\t {}\t {}\n".format(i[0], i[1], i[2], i[3], i[3], i[3]))
f.close()
print("Writting done")

# Display sensors and the car 
# The car is represented by a blue cuboid
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

# Load and display the lidar pointcloud
pcd = read_point_cloud("BufPC.xyzrgb")
pcd.transform(traMat(-1.68, -0.80, 1.65))

# Get GNSS data
gpsCood = getLatLong(dataset)
ENabs = LatLong2ENabs(cropToNpts(gpsCood, frameIDX, 300))  # Translate GPS coordinate in metric coordinate with the UTM WGS84 algorithm 
    # 300 because 10 aquisition per seconds multiplied 30 secondes
ENrel = ENabs2rel(ENabs)    # Get relative motions
# dist = ENrel2dist(ENrel)  # Get distance between each points
cumRel = sumENrel(ENrel)    # Get cumulative vectors

# Create a set of lines to draw the GNSS path
line_set = geometry.LineSet()
line_set.points = Vector3dVector(two2threeD(cumRel, 0.93))
line_set.lines = Vector2iVector(ptLink(cumRel))
colors = [[0, 1, 0] for i in range(len(ptLink(cumRel)))]
line_set.colors = Vector3dVector(colors)

# Adapt the bearing to correct the GNSS trajectory
bearing = dataset.oxts[frameIDX].packet.yaw
line_set.transform(rotyMat(-bearing))
line_set.transform(traMat(-2.76, -0.48, 0.0))

# Draw everything
draw_geometries([mesh_car, mesh_imu, mesh_velo, mesh_cam0, mesh_cam1, mesh_cam2, mesh_cam3, pcd, line_set])