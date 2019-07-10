import numpy as np
from open3d import * # Library to display poitclouds and other 3D objects
import pykitti # Library developped by Lee Clement, found at https://github.com/utiasSTARS/pykitti (2019/05/22)

# Read the dataset
basedir = '../Datasets'
date = '2011_09_26'
drive = '0009'
dataset = pykitti.raw(basedir, date, drive, frames=range(0, 20, 5))

# Get lidar pointcloud for the frame 0
pc_lidar = dataset.get_velo(0) 

# Create a file since open3D seems to be able to get points cloud from file only
print("Start writting!!")
f = open("BufPC.xyz", "w")
for i in pc_lidar:
    f.write("{}\t {}\t {}\n".format(i[0], i[1], i[2]))
f.close()
print("Writting done")

# Read points clouds with open3d function
pcd = read_point_cloud("BufPC.xyz")
print(pcd)

# create a coordinate frame
mesh_frame = create_mesh_coordinate_frame(size = 0.6, origin = [0, 0, 0])

# display all
draw_geometries([pcd, mesh_frame])
