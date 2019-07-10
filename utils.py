import numpy as np
import pykitti
import utm # use UTM-WGS84 to get GPS coordinates in meters 

# print(utm.from_latlon(lat, lon))

def scaMat(x, y, z):
    return np.array([[x, 0.0, 0.0, 0.0], [0.0, y, 0.0, 0.0], [0.0, 0.0, z, 0.0], [0.0, 0.0, 0.0, 1.0]])

def traMat(x, y, z):
    return np.array([[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]])

def rotxMat(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]])

def rotzMat(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, 0.0, s, 0.0], [0.0, 1.0, 0.0, 0.0], [-s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0]])

def rotyMat(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

def getLatLong(dataset):
    latlong = []
    for i in dataset.oxts:
        latlong.append([i.packet.lat, i.packet.lon])
    return latlong

def LatLong2ENabs(latlong):
    ENabs = []
    for i in latlong:
        j = utm.from_latlon(i[0], i[1])
        ENabs.append([j[0], j[1]])
    return ENabs

def ENabs2rel(ENabs):
    rangeEN = range(0, len(ENabs)-1, 1)
    ENrel = []
    for i in rangeEN:
        Erel = ENabs[i+1][0] - ENabs[i][0]
        Nrel = ENabs[i+1][1] - ENabs[i][1]
        ENrel.append([Erel, Nrel])
    return ENrel

def ENrel2dist(ENrel):
    dist = []
    for i in ENrel:
        dist.append(np.sqrt((i[0]*i[0])+(i[1]*i[1])))
    return dist

def sumENrel(ENrel):
    cumENrel = []
    cumENrel.append([0, 0])
    r = range(0, len(ENrel)-1)
    for i in r:
        p = cumENrel[i]
        cumENrel.append([p[0]+ENrel[i][0], p[1]+ENrel[i][1]])
    return cumENrel

def two2threeD(pt2d, heigth):
    pt3d = []
    for i in pt2d:
        pt3d.append([i[0], i[1], heigth])
    return pt3d

def ptLink(pts):
    links = []
    for i in range(0, len(pts)-2):
        links.append([i, i+1])
    return links

def cropToNpts(pts, start, N):
    if (start + N) >= len(pts):
        endIdx = len(pts) - 1
    else:
        endIdx = (start + N)
    return pts[start:endIdx]

