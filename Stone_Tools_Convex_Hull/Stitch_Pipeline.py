from scipy.spatial import ConvexHull
from plyfile import PlyData, PlyElement
import numpy as np

class StitchPipeline:

    def __init__(self, srcPly, dynPly):
        self.srcPly = PlyData.read(srcPly)
        self.dynPly = PlyData.read(dynPly)


    def setup(self):
        srcSize = self.srcPly['vertex'].count
        dynSize = self.dynPly['vertex'].count

        self.srcPts = np.zeros((srcSize, 3))
        self.dynPts = np.zeros((dynSize, 3))

        ##setup the vertices vectors for the respective .plys
        for i in range(srcSize):
            self.srcPts[i][0] = self.srcPly['vertex'][i][0]
            self.srcPts[i][1] = self.srcPly['vertex'][i][1]
            self.srcPts[i][2] = self.srcPly['vertex'][i][2]

        for i in range(dynSize):
            self.dynPts[i][0] = self.dynPly['vertex'][i][0]
            self.dynPts[i][1] = self.dynPly['vertex'][i][1]
            self.dynPts[i][2] = self.dynPly['vertex'][i][2]

        #compute covariances matrices
        dynCov = self.covarianceMatrix(self.dynPts)
        srcCov = self.covarianceMatrix(self.srcPts)

        srcEigVal, srcEigVec = np.linalg.eig(srcCov)
        dynEigVal, dynEigVec = np.linalg.eig(dynCov)

        ##we now compute the change of basis matrix, aligning the clouds' principle components with the right handed basis xyz.

        dynChange = self.c_o_bMatrix(dynEigVec, self.standardBasis())
        srcChange = self.c_o_bMatrix(srcEigVec, self.standardBasis())
        self.changeOrientation(self.dynPts, dynChange)
        self.changeOrientation(self.srcPts, srcChange)
        #retrieve the convex hull w.r.t our dynamic and static vertices
        self.convexHull()

    
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
        mass = 1
        centroid = np.zeros((3, 1))
        if(surface == "static"):
            mass = len(srcPts)
            for i in range(mass):
                centroid += srcPts[i]
        elif(surface == "dynamic"):
            mass = len(dynPts)
            for i in range(mass):
                centroid += dynPts[i]

        centroid[0] /= mass
        centroid[1] /= mass
        centroid[2] /= mass

        return centroid




    def removeTranslation(self, surface):
        return None

    def convexHull(self):
        self.srcConvex = ConvexHull(self.srcPts)
        self.dynConvex = ConvexHull(self.dynPts)

    def write(self, filename):
        f = open(filename + ".ply", "w+")
        f.write("ply\n" + "format ascii 1.0\n" + "element vertex " + str(len(self.dynPts)
                                                                         ) + "\n" + "property float x\n" + "property float y\n" + "property float z\n" + "property uchar red\n" + "property uchar green\n" + "property uchar blue\n" + "end_header\n")
        domain = int(len(self.dynPts))
        for i in range(int(domain)):
            for j in range(0, 3):
                f.write(str(self.dynPts[i][j]) + " ")
            for p in range(0, 3):
                if p < 2:
                    f.write(str(int(255)) + " ")
                else:
                    f.write(str(int(0)))
            f.write("\n")
        f.close()