import numpy as np
from open3d import *
from utils import *
import cv2
import matplotlib.pyplot as plt

frameIDX = 150

basedir = '../Datasets'
date = '2011_09_26'
drive = '0009'
dataset = pykitti.raw(basedir, date, drive)

# print(dataset.calib)
# x = np.array( [[2,3], [3, 5]] )
# y = np.array( [[1,2], [5,-1]] )
# print(np.dot([[2,3], [3, 5]], [[1,2], [5,-1]]))

# Get GNSS data
gpsCood = getLatLong(dataset)
ENabs = LatLong2ENabs(cropToNpts(gpsCood, frameIDX, 300))  # Translate GPS coordinate in metric coordinate with the UTM WGS84 algorithm 
    # 300 because 10 aquisition per seconds multiplied 30 secondes
ENrel = ENabs2rel(ENabs)    # Get relative motions
# dist = ENrel2dist(ENrel)  # Get distance between each points
cumRel = sumENrel(ENrel)    # Get cumulative vectors

GNSS_points = two2threeD(cumRel, -0.93) #Projection au sol pour un meilleur rendu

bearing = dataset.oxts[frameIDX].packet.yaw
correctinBearingMat = rotyMat(-bearing)

GNSS_points_corr = []
for pts in GNSS_points:
    p = np.array([[pts[0]], [pts[1]], [pts[2]], [1]])
    P_proj = np.dot(correctinBearingMat, p)       # apply transformation
    # P_proj = P_proj / P_proj[2] # Normalizing
    GNSS_points_corr.append(P_proj)


P_rect = dataset.calib.P_rect_00
R_rect = dataset.calib.R_rect_00
T_imu = dataset.calib.T_cam0_imu


T_mat = np.dot(np.dot(P_rect, R_rect), T_imu)


imgPoints = []
for pts in GNSS_points_corr:
    p = np.array([[pts[0]], [pts[1]], [pts[2]], [1]])
    P_proj = np.dot(T_mat, p)       # apply transformation
    P_proj = P_proj / P_proj[2] # Normalizing
    imgPoints.append(P_proj)

# print(imgPoints)

#load the image
img = np.array(dataset.get_gray(frameIDX)[0])
# print(img.shape)
imgClr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# cv2.circle(imgClr,(447,63), 63, (0,0,255), -1)

for pts in imgPoints:
    cv2.circle(imgClr, (pts[0], pts[1]), 2, (0, 255, 0), -1)

plt.imshow(imgClr)
plt.show()
