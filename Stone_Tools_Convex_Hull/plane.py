from plyfile import PlyData, PlyElement
import os
import numpy as np
import random

# Parameters
distanceThreshold = 0.01
numRansacTrials = 100

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
    n = np.cross(p3-p1, p2-p1)
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
    scale = np.dot(orig,norm) / np.dot(point, norm)
    return scale* point

def planify(src, tgt):
    # Read in source ply data
    print(' - Reading data')
    ply = PlyData.read(src)
    numVertices = ply['vertex'].count
    srcPts = np.zeros((numVertices, 3))
    for i in range(numVertices):
        srcPts[i][0] = ply['vertex']['x'][i]
        srcPts[i][1] = ply['vertex']['y'][i]
        srcPts[i][2] = ply['vertex']['z'][i]
    tgtPts = np.zeros((numVertices, 3))
    # Just a copy for now
    print(' - Planification')
    # Find the best plane and project all points onto it
    plane = estimatePlaneRansac(srcPts)
    for i in range(numVertices):
        tgtPts[i] = project(srcPts[i], plane)
    # Update the Ply file data
    print(' - Writing data')
    for i in range(numVertices):
        ply['vertex']['x'][i] = tgtPts[i][0]
        ply['vertex']['y'][i] = tgtPts[i][1]
        ply['vertex']['z'][i] = tgtPts[i][2]
    # Write the result
    ply.write(tgt)


for f in os.listdir('output'):
    if os.path.isdir('output/' + f):
        print(f)
        src = 'output/' + f + '/' + f + '_point_cloud.ply'
        tgt = 'output/' + f + '/' + f + '_point_cloud_planified.ply'
        planify(src, tgt)

