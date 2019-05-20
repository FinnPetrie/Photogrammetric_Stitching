import math
from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt

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

        #staticX = np.r_[staticX, staticX[0]]
       # staticY = np.r_[staticY, staticY[0]]
        #dynamicX = np.r_[dynamicX, dynamicX[0]]
       # dynamicY = np.r_[dynamicY, dynamicY[0]]
        #rint(dynamicX)
      # print(dynamicY)
        self.tck_static, self.u_static, = interpolate.splprep([staticX, staticY], s=0, per=True)
        self.tck_dynamic, self.u_dynanic, = interpolate.splprep([dynamicX, dynamicY], s=0, per=True)

        dynamic_T = self.tck_dynamic[0]
        dynamic_C =self.tck_dynamic[1]
        dynamic_C = np.transpose(dynamic_C)
        dynamic_K = self.tck_dynamic[2]
        static_T = self.tck_static[0]
        static_C = self.tck_static[1]
        static_C = np.transpose(static_C)
        static_K = self.tck_static[2]

        #print(dynamic_T)
       # print(dynamic_C)
       # print(dynamic_K)
       # knots, control, degree = interpolate.splprep([staticX, staticY],s=0, per=True)[0]
        #print(knots)
       # print(degree)
     #   control = np.transpose(control)
       #index = np.arange(len(control))
       # print(index)
       # t = interpolate.BSpline(knots, control, degree)
       # print(t(2))
      #  print(knots, control, degree)
        index = np.arange(len(static_C))

        self.B_Dynamic = interpolate.BSpline(dynamic_T, dynamic_C, dynamic_K)
        self.B_Static = interpolate.BSpline(static_T, static_C, static_K, extrapolate=True)

        staticXMin = staticX[0]
        staticXMax = staticX[len(staticX) -2]
       # c = interpolate.InterpolatedUnivariateSpline(dynamic_T, dynamic_C, dynamic_K)
        N = 1000
        
        xx = np.linspace(staticXMin, staticXMax, N)
        self.write_B(self.B_Static(xx), "dig")
        #print(self.B_Static(2))

    def write_B(self, B, name):
        print(B)
        f = open(name + ".ply", "w+")
        f.write("ply\n" + "format ascii 1.0\n" + "element vertex " + str(len(B)) + "\n" + "property float x\n" + "property float y\n" + "property float z\n" + "property uchar red\n" + "property uchar green\n" + "property uchar blue\n" + "end_header\n")
        domain = int(len(B))
        for i in range(domain):

            f.write(str(0) + " "  + str(B[i][0]) + " " + str(B[i][1]) + " ")
            for p in range(0, 3):
                if p < 2:
                    f.write(str(int(255)) + " ")
                else:
                    f.write(str(int(0)))
            f.write("\n")
        f.close()



    def evalSpline(self):

        staticX = self.staticPoints[:, 1]
        staticY = self.staticPoints[:, 2]
        # rint(staticX)
        # print(self.staticPoints)
        dynamicX = self.dynamicPoints[:, 1]
        dynamicY = self.dynamicPoints[:, 2]

        # print(dynamicX)
        # print(dynamicY)
        staticX = np.array(staticX)
        staticY = np.array(staticY)

        dynamicX = np.array(dynamicX)
        dynamicY = np.array(dynamicY)
        points = np.array([[0, 1, 8, 2, 2],
                           [1, 0, 6, 7, 2]]).T
        p = np.array([dynamicX, dynamicY]).T
        # print(points)
        # print(p)

        d = np.cumsum(np.sqrt(np.sum(np.diff(p, axis=0)**2, axis=1)))
        d = np.insert(d, 0, 0)/d[-1]

        alpha = np.linspace(0, 1, 75)
        interp = interpolate.interp1d(d, p, kind='cubic', axis=0)
        i = interp(alpha)

        splines = [interpolate.UnivariateSpline(d, coords, k=3, s=.2) for coords in p.T]
        print(splines)
        for q in range(len(splines)):
            print(splines[q])
            print("Our q: " + str(q))
        print(len(p))

        # print(i)
        # print(d[-1])
        # print(d)
        plt.figure(figsize=(7, 7))
        plt.plot(*i.T, '-', label='cubic');

        plt.plot(*p.T, 'ok', label='original points');
        plt.axis('equal');
        plt.legend();
        plt.xlabel('x');
        plt.ylabel('y');
        plt.show()
    def print(self):
        print("dynamic spline co-efficients: " +  str(self.dSpline[1]))
        print("static spline co-efficients: " + str(self.sSpline[1]))

    # def evaluate(self, dynamicI, theta):
    #
    #         v = np.zeros(2)
    #         v[0] = np.cos(theta)
    #         v[1] = np.sin(theta)
    #         #setup
    #         slope = dynamicI[1]/dynamicI[0]
    #         pSlope = np.zeros(2)
    #         pSlope[0] = slope
    #         pSlope[1] = slope*dynamicI[0] + dynamicI[1]
    #         #want to find y = slope*x
    #         #define the line that passes through dynamicI
    #         #find the intersection in the tck_static
    #         #return point

    def spline(self):
        self.tck_static, self.u_static = interpolate.splprep([self.staticX, self.staticY], s=0, per=True)
        self.tck_dynamic, self.u_dynamic = interpolate.splprep([self.dynamicX, self.dynamicY], s=0, per=True)


    def evalaute(self, theta):
        evals = []
        for t in(self.tck_dynamic[1]):
            i = 0
            while(i < len(t)):
                rSin = t[i]*pow(math.cos(theta), 3) + t[i +1]*pow(math.cos(theta), 2) + t[i + 2]*math.cos(theta) + t[i + 3]
                rTheta = np.array([theta, rSin])
                evals.append(rSin)
                i +=4
        print(evals)

        return evals




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
        errorDistance = 0
        dynamicCosine = []
        for theta in range(0, 360):
            dynamicI = (self.B_Dynamic(math.cos(theta)))
            #dynamicI = (interpolate.splev(math.cos(math.radians(theta)), self.tck_dynamic))

            #print(dynamicI)
            # dI = self.B_Dynamic.basis_element([0, 1, 2, 3, 4])
            # t = dI(math.cos(theta))
            # print("Our t: " + str(t))
            #print(dynamicI)
           # staticI = (interpolate.splev(math.cos(math.radians(theta)), self.tck_static))
            staticI = self.B_Static(math.cos(theta))
            # sI = self.B_Static.basis_element([0,1,2,3,4])
            # s = sI(math.cos(theta))
            # print("Our dynamic value: " + str(dynamicI) + " our static value : " + str(staticI))
            # distanceX = dynamicI[0] - staticI[0]
            # distanceY = dynamicI[1] - staticI[1]
            #   d = np.(distanceX, distanceY)
            dynamicNP = np.array(dynamicI)
            staticNP = np.array(staticI)
            dynamicCosine.append(dynamicNP)
          # print("Our dynamic: " + str(dynamicNP))
           #print("Our static: " + str(staticNP))
            # print("Our s : " + str(s))
           # d = dI - sI
            d  = dynamicNP - staticNP
            errorDistance += np.linalg.norm(d)
       # self.writeCosine(dynamicCosine, "Cosine")
        return errorDistance






    def writeCosine(self, cosine, name):
        f = open(name + ".ply", "w+")
        f.write("ply\n" + "format ascii 1.0\n" + "element vertex " + str(len(cosine)) + "\n" + "property float x\n" + "property float y\n" + "property float z\n" + "property uchar red\n" + "property uchar green\n" + "property uchar blue\n" + "end_header\n")
        domain = int(len(cosine))
        for i in range(int(domain)):
            print(cosine[i][0])
            print(cosine[i][1])
            f.write(str(cosine[i][0]) + " " + str(cosine[i][1]) + " " + str(0) + " ")
            for p in range(0, 3):
                if p < 2:
                    f.write(str(int(255)) + " ")
                else:
                    f.write(str(int(0)))
            f.write("\n")
        f.close()


    def run(self):
        self.write("BeforeMatched")
        theta = 0
        minTheta = 0
        minError = math.inf

        while(theta <= 360):
            self.setup_splines()
           # self.evalaute(math.radians(theta))
            self.rotate(theta)
            e = self.error()
            if(e < minError):
                print(e)
                minError = e
                minTheta = theta
            theta += 1
        print(minError)
        print(minTheta)
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