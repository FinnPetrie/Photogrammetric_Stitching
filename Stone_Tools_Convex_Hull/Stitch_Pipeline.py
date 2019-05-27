from scipy.spatial import ConvexHull
from plyfile import PlyData, PlyElement
import numpy as np
from Matching import Matching
from pyquaternion import Quaternion
class StitchPipeline:

    def __init__(self, srcPly, dynPly):
        self.srcPly = PlyData.read(srcPly)
        self.dynPly = PlyData.read(dynPly)

    def changeOrientation(self, points, cbMatrix):
        for i in range(len(points)):
            points[i] = np.matmul(cbMatrix, points[i])

    def setup(self):
        srcSize = self.srcPly['vertex'].count
        dynSize = self.dynPly['vertex'].count

        self.srcPts = np.zeros((srcSize, 3))
        self.srcColours = np.zeros((srcSize, 3))
        # self.srcNormals = np.zeros((srcSize, 3))
        self.dynPts = np.zeros((dynSize, 3))
        self.dynColours = np.zeros((dynSize, 3))
        # self.dynNormals = np.zeros((dynSize, 3))

        ##setup the vertices vectors for the respective .plys
        for i in range(srcSize):
            self.srcPts[i][0] = self.srcPly['vertex'][i][0]
            self.srcPts[i][1] = self.srcPly['vertex'][i][1]
            self.srcPts[i][2] = self.srcPly['vertex'][i][2]

            self.srcColours[i][0] = self.srcPly['vertex'][i]['red']
            self.srcColours[i][1] = self.srcPly['vertex'][i]['green']
            self.srcColours[i][2] = self.srcPly['vertex'][i]['blue']

        for i in range(dynSize):
            self.dynPts[i][0] = self.dynPly['vertex'][i][0]
            self.dynPts[i][1] = self.dynPly['vertex'][i][1]
            self.dynPts[i][2] = self.dynPly['vertex'][i][2]

            self.dynColours[i][0] = self.dynPly['vertex'][i]['red']
            self.dynColours[i][1] = self.dynPly['vertex'][i]['green']
            self.dynColours[i][2] = self.dynPly['vertex'][i]['blue']

        print(self.dynColours)


        #compute covariances matrices
        dynCov = self.covarianceMatrix(self.dynPts)
        srcCov = self.covarianceMatrix(self.srcPts)

        srcEigVal, srcEigVec = np.linalg.eig(srcCov)
        dynEigVal, dynEigVec = np.linalg.eig(dynCov)

        ##we now compute the change of basis matrix, aligning the clouds' principle components with the right handed basis xyz.

        dynChange = self.c_o_bMatrix(dynEigVec, self.standardBasis())
        srcChange = self.c_o_bMatrix(srcEigVec, self.standardBasis())
        #rexpress the pointclouds w.r.t the standard basis.
        self.changeOrientation(self.dynPts, dynChange)
        self.changeOrientation(self.srcPts, srcChange)

        #remove translation from each pointcloud
        self.removeTranslation(self.dynPts)
        self.removeTranslation(self.srcPts)
        #now we rotate the dynamicSurface to be beneath the other surface.
        #i.e., a rotation by 180 degrees around it's largest principle component.

        dynEigVal, dynEigVec = np.linalg.eig(dynCov)

        self.rotateAroundAxis(dynEigVec, 2, np.pi, self.dynPts)
        
        #retrieve the convex hull w.r.t our dynamic and static vertices
        self.write("dynamicOrientation", self.dynPts, True)
        self.write("srcOrientation", self.srcPts, True)

        #flatten, identifying the axis to remove, i.e., 0 = the x axis.
        flattenDyn = self.flattenSurface(self.dynPts, 0)
        flattenSrc = self.flattenSurface(self.srcPts, 0)
        self.write("flattenDyn", flattenDyn, False)
        self.write("flattenSrc", flattenSrc, False)
        self.convexHull("dynamic", flattenDyn)
        self.convexHull("static", flattenSrc)


        self.dynConvexHullPts = np.zeros((self.dynConvexHull.vertices.size, 2))
        for i in range(self.dynConvexHull.vertices.size):
            self.dynConvexHullPts[i][0] = flattenDyn[self.dynConvexHull.vertices[i]][0]
            self.dynConvexHullPts[i][1] = flattenDyn[self.dynConvexHull.vertices[i]][1]

        self.stcConvexHullPts = np.zeros((self.staticConvexHull.vertices.size, 2))

        for i in range(self.staticConvexHull.vertices.size):
            self.stcConvexHullPts[i][0] = flattenSrc[self.staticConvexHull.vertices[i]][0]
            self.stcConvexHullPts[i][1] = flattenSrc[self.staticConvexHull.vertices[i]][1]






    def translateToHull(self):
        return 0

    def rotateAroundAxis(self, covMat, index, angle, pts):
        quat = Quaternion(axis = covMat[index], angle = angle)
        print(quat)
        for i in range(len(pts)):
            pts[i] = quat.rotate(pts[i])

    def match(self):
        ##retrieve the convex hulls
        print(self.dynConvexHullPts)
        m = Matching(self.stcConvexHullPts, self.dynConvexHullPts, staticSurface = self.srcPts, dynamicSurface = self.dynPts, stitching = True)
        m()
        self.write("dynConvex", self.dynConvexHullPts, False)
        self.write("stcConvexHull", self.stcConvexHullPts, False)
        self.write("staticSurface", self.srcPts, True, colours=self.srcColours)
        self.write("dynSurface", self.dynPts, True, colours = self.dynColours)

    #projects the surcace onto a plane by removing the axis specified.
    def flattenSurface(self, surface, axis):
        flattenD = np.zeros((len(surface), 2))
        for i in range(len(surface)):
            flattenD[i][0] = surface[i][(axis+1)%3]
            flattenD[i][1] = surface[i][(axis+2)%3]

        return flattenD


    #helper method, returns the rhs standard basis
    def standardBasis(self):
        x = np.array([1, 0, 0])
        y = np.array([0, 1, 0])
        z = np.array([0, 0, 1])
        return (x, y, z)


    def c_o_bMatrix(self, fromBasis, toBasis):
        print('our basis:\n ', fromBasis)
        print('to basis:\n ', toBasis)
        fromBasisMat = np.asmatrix([fromBasis[0], fromBasis[1], fromBasis[2]], dtype=np.float64)
        # fromBasisTranspose = fromBasisMat.transpose()
        fromBasisInverse = np.linalg.inv(fromBasisMat)
        print(fromBasisInverse)
        toBasisMat = np.asmatrix([toBasis[0], toBasis[1], toBasis[2]], dtype=np.float64)

        # print(fromBasisTranspose)
        print(toBasisMat)

        return np.matmul(toBasisMat, fromBasisInverse)


    def covarianceMatrix(self, pts):
        cov_Comp = np.transpose(pts)
        # print(cov_Comp)
        cov_mat = np.cov([cov_Comp[0, :], cov_Comp[1, :], cov_Comp[2, :]])
        return cov_mat

    def centroid(self, surface):
        mass = len(surface)
        centroid = np.zeros((3, 1))

        for i in range(mass):
            centroid[0] += surface[i][0]
            centroid[1] += surface[i][1]
            centroid[2] += surface[i][2]

        centroid[0] /= mass
        centroid[1] /= mass
        centroid[2] /= mass

        return centroid




    def removeTranslation(self, surface):
        srCentroid = self.centroid(surface)
        print(srCentroid)
        for i in range(len(surface)):
            surface[i][0] -= srCentroid[0]
            surface[i][1] -= srCentroid[1]
            surface[i][2] -= srCentroid[2]


    def convexHull(self, surface, points):
        if(surface == "dynamic"):
            self.dynConvexHull = ConvexHull(points)
        elif(surface == "static"):
            self.staticConvexHull = ConvexHull(points)


    def write(self, filename, points, threeD, colours = None):
        f = open(filename + ".ply", "w+")
        f.write("ply\n" + "format ascii 1.0\n" + "element vertex " + str(len(points)
              ) + "\n" + "property float x\n" + "property float y\n" + "property float z\n" +
            "property uchar red\n" + "property uchar green\n" + "property uchar blue\n" + "end_header\n")

        if(threeD):
           axis = 3
        else:
           axis = 2
        domain = int(len(points))
        for i in range(int(domain)):
            for j in range(0, axis):
                f.write(str(points[i][j]) + " ")

            if (not threeD):
                f.write(str("0") + " ")


            for p in range(0, 3):
                if(colours is None):
                     if p < 2:
                         f.write(str(int(255)) + " ")
                     else:
                         f.write(str(int(0)))
                else:
                    if(p < 2):
                        f.write(str(int(colours[i][p])) + " ")
                    else:
                        f.write(str(int(colours[i][p])))


            f.write("\n")
        f.close()

