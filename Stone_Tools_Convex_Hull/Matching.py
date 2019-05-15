import math
from scipy import interpolate
import numpy as np
class Matching:



    def __init__(self, staticPoints, dynamicPoints, dynColours):
        self.staticPoints = staticPoints
        self.dynColours = dynColours
        self.dynamicPoints = dynamicPoints

    def setup_splines(self):
        staticX = self.staticPoints[:, 1]
        staticY = self.staticPoints[:, 2]
        #rint(staticX)
       #print(self.staticPoints)
        dynamicX = self.dynamicPoints[:, 1]
        dynamicY = self.dynamicPoints[:, 2]
      # print(dynamicX)
      # print(dynamicY)
        staticX = np.array(staticX)
        staticY = np.array(staticY)

        dynamicX = np.array(dynamicX)
        dynamicY = np.array(dynamicY)

        staticX = np.r_[staticX, staticX[0]]
        staticY = np.r_[staticY, staticY[0]]
        dynamicX = np.r_[dynamicX, dynamicX[0]]
        dynamicY = np.r_[dynamicY, dynamicY[0]]
        #rint(dynamicX)
      # print(dynamicY)
        self.tck_static, self.u_static = interpolate.splprep([staticX, staticY], s=0, per=True)
        self.tck_dynamic, self.u_dynanic = interpolate.splprep([dynamicX, dynamicY], s=0, per=True)

     #  print(staticX)

    def print(self):
        print("dynamic spline co-efficients: " +  str(self.dSpline[1]))
        print("static spline co-efficients: " + str(self.sSpline[1]))

    def evaluate(self, theta):

            v = np.zeros(2)
            v[0] = np.cos(theta)
            v[1] = np.sin(theta)
            print(np.linalg.norm(v))
            #x1 = interpolate.splev(theta, self.dSpline)
           # x2 = interpolate.splev(theta, self.sSpline)
            x1 = interpolate.splev(v, self.dSpline)

            print("Our x1 following evaluation: " + str(x1))

            return x1
           # x1 = interpolate.splev(theta, self.dSpline)
           # print(str(x1) + " = our x1")
           # print(str(x2) + " = our x2")
          #  d = np.subtract(x1, x2)
           # print(d)
           # print(np.linalg.norm(d))
           # x1 = 0

            #t1 = sqrt(pow(x1, 2)+ pow(y1, 2))
           # t2 = sqrt(pow(x2, 2) + pow(y2, 2))

            #subtract t2*v2 - t1*v1
            #cal magnitude - add to total error
            #solve for t

    def spline(self):
        self.tck_static, self.u_static = interpolate.splprep([self.staticX, self.staticY], s=0, per=True)
        self.tck_dynamic, self.u_dynamic = interpolate.splprep([self.dynamicX, self.dynamicY], s=0, per=True)



    def rotate(self, angle):
        theta = np.radians(angle)
        c, s = np.cos(theta), np.sin(theta)
      # print("Our sin: " + str(s))
       #print("Our cosine: " + str(c))
        R = np.zeros((2, 2))
        R[0][0] = c
        R[0][1] = -s
        R[1][0] = s
        R[1][1] = c
#       R = np.array((c, -s), (s, c))
        #rint(R)
        for i in range(0, len(self.dynamicPoints)):
           #newX = R*self.dynamicPoints[i][1]
           #newY = R*self.dynamicPoints[i][2]

            omitZ = np.zeros((2, 1))
            omitZ[0] = self.dynamicPoints[i][1]
            omitZ[1] = self.dynamicPoints[i][2]
            rot_D = np.matmul(R, omitZ)
          # print("This is R :"  + str(R[0]) + "\n" + str(R[1]))
           #print(rot_D[0])         #rint(newX)
          # print(rot_D[1])
           #print(newY)
            self.dynamicPoints[i][1] = rot_D[0]
            self.dynamicPoints[i][2] = rot_D[1]


    def error(self):
        i = 0
        errorDistance = 0
        while i <= 1:
            dynamicI = (interpolate.splev(i, self.tck_dynamic))
            staticI = (interpolate.splev(i, self.tck_static))
            # print("Our dynamic value: " + str(dynamicI) + " our static value : " + str(staticI))
            # distanceX = dynamicI[0] - staticI[0]
            # distanceY = dynamicI[1] - staticI[1]
            #   d = np.(distanceX, distanceY)
            dynamicNP = np.array(dynamicI)
            staticNP = np.array(staticI)
          # print("Our dynamic: " + str(dynamicNP))
           #print("Our static: " + str(staticNP))
            d = dynamicNP - staticNP
            errorDistance += np.linalg.norm(d)
            i += 0.01
        return errorDistance

    def run(self):
        self.write("BeforeMatched")
        theta = 0
        minTheta = 0
        minError = math.inf
        while(theta <= 360):
            self.setup_splines()
            self.rotate(theta)
            e = self.error()
            if(e < minError):
                minError = e
                minTheta = theta
            theta += 1
        self.rotate(minTheta)
        self.write("Matched")

    def write(self, filename):
        f = open(filename + ".ply", "w+")
        f.write("ply\n" + "format ascii 1.0\n" + "element vertex " + str(len(self.dynamicPoints)
           ) + "\n" + "property float x\n" + "property float y\n" + "property float z\n" + "property uchar red\n" + "property uchar green\n" + "property uchar blue\n" + "end_header\n")
        domain = int(len(self.dynamicPoints))
        for i in range(int(domain)):
            for j in range(0, 3):
                f.write(str(self.dynamicPoints[i][j]) + " ")
            for p in range(0, 3):
                if p < 2:
                    f.write(str(int(255)) + " ")
                else:
                    f.write(str(int(0)))
            f.write("\n")
        f.close()