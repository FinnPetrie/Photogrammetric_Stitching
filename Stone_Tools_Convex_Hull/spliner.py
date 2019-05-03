from plyfile import PlyData, PlyElement
import os
import numpy as np
import random
import math
from Matching import Matching
from scipy.spatial import ConvexHull
from scipy import interpolate


# Parameters
distanceThreshold = 0.01
numRansacTrials = 100
numRuns = 0

def distance(point, plane):
    return abs(np.dot(plane[1], plane[0] - point))


def checkInliers(points, plane):
    nPnts = points.shape[0]
    inliers = []
    for i in range(nPnts):
        if distance(points[i], plane) < distanceThreshold:
            inliers.append(i)
    return inliers


def estimatePlane(p1, p2, p3):
    n = np.cross(p3 - p1, p2 - p1)
    n = n / np.linalg.norm(n)
    return (p1, n)


def estimatePlaneRansac(points):
    # RANSAC trials
    nPnts = points.shape[0]
    bestInliers = []
    bestPlane = (points[0], points[1])
    for t in range(numRansacTrials):
        ix = random.sample(range(nPnts), 3)
        p1 = points[ix[0]]
        p2 = points[ix[1]]
        p3 = points[ix[2]]
        plane = estimatePlane(p1, p2, p3)
        inliers = checkInliers(points, plane)
        if len(inliers) > len(bestInliers):
            bestInliers = inliers
            bestPlane = plane
            print(len(bestInliers))
    return bestPlane


def project(point, plane):
    norm = plane[1]
    orig = plane[0]
    scale = np.dot(orig, norm) / np.dot(point, norm)
    return scale * point


def write(vertices, colours, filename):
    f = open(filename + ".ply", "w+")
    f.write("ply\n" + "format ascii 1.0\n" + "element vertex " + str(vertices.size/3) + "\n" + "property float x\n" + "property float y\n" + "property float z\n" + "property uchar red\n" + "property uchar green\n" + "property uchar blue\n" + "end_header\n")
    domain = int(vertices.size)/3
    for i in range(int(domain)):
        for j in range(0, 3):
            f.write(str(vertices[i][j]) + " ")
        for p in range(0, 3):
            if p < 2:
                f.write(str(int(255)) + " ")
            else:
                f.write(str(int(0)))
        f.write("\n")
    f.close()



def curvature():
    return

def polar_coordinates(values):
    #given a vector of values
    print("Size of values: " + str(values.size))
    size = int(values.size/3)
    polar = np.zeros((size, 2))
    for i in range(size):
        r = math.sqrt(pow(values[i][1], 2) + pow(values[i][2], 2))
        theta = math.atan(values[i][2]/values[i][1])
        polar[i][0] = r
        polar[i][1] = theta
    return polar

def spline(src, tgt):
    # Read in source ply data
    print(' - Reading data')
    ply = PlyData.read(src)
    numVertices = ply['vertex'].count
    srcPts = np.zeros((numVertices, 3))
    #get the convex hull
    srcColours = np.zeros((numVertices, 3))

    x = []
    y = []
    for i in range(numVertices):
        srcPts[i][0] = ply['vertex']['x'][i]
        srcPts[i][1] = ply['vertex']['y'][i]
        srcPts[i][2] = ply['vertex']['z'][i]
        srcColours[i][0] = ply['vertex']['red'][i]
        srcColours[i][1] = ply['vertex']['green'][i]
        srcColours[i][1] = ply['vertex']['blue'][i]

        x.append(srcPts[i][1])
        y.append(srcPts[i][2])
    ##compute the spline.
   # tck, u = interpolate.splprep([x, y], s = 0, per=true)
    ##get xi, yi
  #  print(x)

    x1 = np.array(x)
    x2 = np.array(y)
    x1 = np.r_[x1, x1[0]]
    x2 = np.r_[x2, x2[0]]

    print(x1)
    print(x2)

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x1, x2], s=0, per=True)


    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, 1000), tck)

    print(xi.size)

    # plot the result


    vertices = np.zeros((1000, 3))
    tgtPts = np.zeros((1000, 3))
    for i in range(1000):
        vertices[i][1] = xi[i]
        vertices[i][2] = yi[i]
        vertices[i][0] = 0
   # for i in range(numVertices):
       # tgtPts[i] = srcPts[i]
       # print(srcPts[i])
    # Update the Ply file data
    p = polar_coordinates(vertices)
    print(p)
    print(' - Writing data')
    for i in range(numVertices):
        ply['vertex']['x'][i] = tgtPts[i][0]
        ply['vertex']['y'][i] = tgtPts[i][1]
        ply['vertex']['z'][i] = tgtPts[i][2]
    # Write the result
    write(vertices, srcColours, "spline" + str(numRuns))

   # ply.write(vertices)



def matching(dynamic, static):
    # Read in source ply data
    print(' - Reading data')
    staticPly = PlyData.read(static)
    dynamicPly = PlyData.read(dynamic)
    numVertices = staticPly['vertex'].count
    staticSpline = np.zeros((numVertices, 3))
    dynamicSpline = np.zeros((numVertices, 3))

    for i in range(numVertices):
        staticSpline[i][0] = staticPly['vertex']['x'][0]
        staticSpline[i][1] = staticPly['vertex']['y'][1]
        staticSpline[i][2] = staticPly['vertex']['z'][2]
        dynamicSpline[i][0] = dynamicPly['vertex']['x'][0]
        dynamicSpline[i][1] = dynamicPly['vertex']['y'][1]
        dynamicSpline[i][2] = dynamicPly['vertex']['z'][2]


    polarStatic = polar_coordinates(staticSpline)
    polarDynamic = polar_coordinates(dynamicSpline)

    match = Matching(polarStatic, polarDynamic )




# ply.write(vertices)

def approximate_spline():
    for f in os.listdir('output'):
     if os.path.isdir('output/' + f):
        print(f)
        src = 'output/' + f + '/' + f + '_static_point_cloud.ply'
        tgt = 'output/' + f + '/' + f + '_point_cloud_planified.ply'
        spline(src, tgt)
        numRuns += 1


def match_hulls():
    for f in os.listdir('output'):
        if os.path.isdir('output/' + f):
            srcOne = 'output/' + f + '/' + f + '_static_point_cloud.ply'
            srcTwo = 'output/' + f + '/' + f + '_dynamic_point_cloud.ply'
            matching(srcOne, srcTwo)

match_hulls()