# smallKittiViewer

Here is the source code of some exercise. They are based on [KITTI](http://www.cvlibs.net/datasets/kitti/index.php) datasets.

## Requirements 

The following packets are needed: 
- [utm](https://pypi.org/project/utm/) (Convert GPS coordinates in mercator coordinates)
- [pykitti](https://pypi.org/project/pykitti/) (Read KITTI's datasets)
- [open3D](https://pypi.org/project/open3d-python/) (Display 3D objects and point cloud)
- [opencv](https://pypi.org/project/opencv-python/) (Image processing)
- numpy
- matplotlib


## Exercises:

List of exercises

1. Display coordinate frames for cameras, IMU and velodyne
2. Display the GNSS trajectory for the next 30 seconds in a Velodyne pointcloud.
3. Display the GNSS tragectory for the next 30 seconds in the image of one of the front camera

## Run the files

Download rectified raw dataset and calibrations set on the RAW Data page of [KITTI](http://www.cvlibs.net/datasets/kitti/index.php).
First exercice can be run with reperes.py, the second with lidarGNSS.py and the last with camGNSS.py. A bonus one, camToLidar.py, display the lidar pointcloud with the color from the camera image. 
