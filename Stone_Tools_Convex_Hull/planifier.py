from plyfile import PlyData, PlyElement
import os
import numpy as np
import random
from scipy.spatial import ConvexHull

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
	#print("Our points: " + str(p1) + " " + str(p2) + " " + str(p3))
	n = np.cross(p3-p1, p2-p1)
	n = n / np.linalg.norm(n)
	# point = (p3-p1)/np.linalg.norm(p3-p1)
	#secondPoint = np.cross(n, point)
	#  print("\nOrthogonal tests:\n normal with point: " + str(np.dot(n, point)))
	# print("\npoint with secondPoint : " + str(np.dot(point, secondPoint)))
	#print("\nnormal with secondPoint : " + str(np.dot(n, secondPoint)))
	return (p1, n)


def computeXYPlane():

	normal =np.array([0, 0, 1])
	point = np.array([1, 1, 0])
	return (point, normal)


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
			print("Number of inliers: " + str(len(bestInliers)) )
	return bestPlane

def project(point, plane):

	norm = plane[1]
	orig = plane[0]
	# print("\nOur normal : " + str(norm))
	# print("\nOur origin: " + str(orig))
	#print(np.dot(orig, np.transpose(norm)))
	#scale = np.dot(orig,np.transpose(norm)) / np.dot(point, np.transpose(norm))
	scale = np.dot(point, np.transpose(norm))
	print(scale)
	return scale* point

def secondProject(point, plane):
	norm = plane[1]
	#print(norm)
	#print(point)
	scale = np.dot(point,np.transpose(norm)) / np.dot(norm, np.transpose(norm))*norm
	return point - scale

def write(vertices, colours, filename):
	f = open(filename + ".ply", "w+")
	f.write("ply\n" + "format ascii 1.0\n" + "element vertex " + str(vertices.size/3) + "\n" + "property float x\n"
			+ "property float y\n" + "property float z\n" + "property uchar red\n" + "property uchar green\n" + "property uchar blue\n"
			+ "end_header\n")
	print("Size of our vertices: " + str(vertices.size/3) + "\n")

	for i in range(vertices.size/3 ):
		#  print(str(i) + "\n")

		for j in range(0, 3):
			#      print("Our j :" + str(j) + "\n")
			f.write(str(vertices[i][j]) + " ")

		for p in range(0, 3):
			#  print("Our colour: " + str(colours[i][p]) + "\n")
			if(p < 2):
				f.write(str(int(colours[i][p])) + " ")
			else:
				f.write(str(int(colours[i][p])))

		f.write("\n")

	f.close()
	print("Finished writing")



def writePlane(plane, filename):
	print(plane)
	f = open(filename + ".plane", "w+")
	f.write(np.array2string(plane[0]) + "\n")
	##this is our normal
	f.write(np.array2string(plane[1]) + "\n")

	#f.write(plane[1] + "\n")



def standardBasis():
	x = np.array([1,0,0])
	y = np.array([0,1,0])
	z = np.array([0,0,1])

	return(x, y, z)

def computeMean(data, vertexNumber):
	mean_x = 0
	mean_y = 0
	mean_z = 0
	for i in range(vertexNumber):
		mean_x += data[i][0]
		mean_y += data[i][1]
		mean_z += data[i][2]
	mean_x /= vertexNumber
	mean_y /= vertexNumber
	mean_z /= vertexNumber
	return(np.array([[mean_x], [mean_y], [mean_z]]))

def computeXZPlane():
	normal =np.array([0, 1, 0])
	point = np.array([1, 0, 1])
	return (point, normal)
def computeYZPlane():
	normal =np.array([1, 0, 0])
	point = np.array([0, 1, 1])
	return (point, normal)

def computePlaneBasis(plane):
	normal = plane[1]
	origin = plane[0]
	perp = np.cross(normal, origin)
	print("Dot product tests: " + str(np.dot(normal, origin)))
	print("\n" + str(np.dot(normal, perp)))
	print("\n" + str(np.dot(origin, perp)))
	return (origin, normal, perp)
#x, y, z


def computeScatterMatrix(meanVector, data, numVertices):
	scatterMatrix = np.zeros((3,3))
	for i in range (numVertices):
		scatterMatrix += (data[:,i].reshape(3, 1) - meanVector).dot((data[:,i].reshape(3,1) - meanVecvtor).T)
	print(scatterMatrix)


def cobMatrix(fromBasis, toBasis):
	print('our basis:\n ', fromBasis)
	print('to basis:\n ', toBasis)
	fromBasisMat = np.asmatrix([fromBasis[0], fromBasis[1], fromBasis[2]], dtype = np.float64)
	#fromBasisTranspose = fromBasisMat.transpose()
	fromBasisInverse = np.linalg.inv(fromBasisMat)
	print(fromBasisInverse)
	toBasisMat = np.asmatrix([toBasis[0], toBasis[1], toBasis[2]], dtype = np.float64)

	#print(fromBasisTranspose)
	print(toBasisMat)

	return np.matmul(toBasisMat, fromBasisInverse)


def transformedPlane(cBasMat, plane):
	point = np.matmul(cBasMat, plane[0])
	norm = np.matmul(cBasMat, plane[1])


	return (point,norm)



#def constructEigenPlane(eigenMat, eigenValues):
#	 biggestEigenValue = 0
#	 2ndBiggestEigenValue = 0
#	 eigenIndex
#	 2ndEigenIndex
#	for i in range len(eigenValues):
#		if(abs(eigenValues[i]) > biggestEigenValue):
#			eigenIndex = i
#		biggestEigenValue = eigenValues[i]

#for i in range len(eigenValues):
#	if(abs(eigenValues[i]) > 2ndBiggestEigenValue and i != eigenIndex):
#		2ndBiggestEigenValue = eigenValues[i]
#
#print(biggestEigenValue + ' ' + 2ndBiggestEigenValue + ' ' + eigenIndex + ' ' + 2ndEigenIndex)



def recontexCent(points, centroid, numVertices):
	#for i in range(3):
	#print('our centroid : ', centroid[i])

	for i in range(numVertices):
		points[i][0] -= centroid[0]
		points[i][1] -= centroid[1]
		points[i][2] -= centroid[2]

def centroid(points, numVertices):
	ptCentr = np.zeros((3))
	#print(ptCentr)
	for i in range(numVertices):

		ptCentr[0] += points[i][0]
		ptCentr[1] += points[i][1]
		ptCentr[2] += points[i][2]

	#for i in range(numVertices):
	ptCentr /= numVertices
	#print(ptCentr)
	return ptCentr

def translateToHull(point, transVec):
	#print(point)
	p = point
	#print(p)
	p[0] += transVec[0]
	p[1] += transVec[1]
	p[2] += transVec[2]
	return p


#What do I want to do here?
#Do the same computations, but remapping the object's plane.
def planify(src, plane, tgt):
	# Read in source ply data
	print(' - Reading data')
	ply = PlyData.read(src)
	plane = PlyData.read(plane)
	#denseCloud = PlyData.read('VentralToPlane.ply')
	numVertices = ply['vertex'].count
	numPlaneVertices = plane['vertex'].count
	planePts = np.zeros((numPlaneVertices, 3))
	planeColours = np.zeros((numPlaneVertices, 3))
	srcPts = np.zeros((numVertices, 3))
	srcColours = np.zeros((numVertices, 3))

	#denseNumber = denseCloud['vertex'].count
	#densePoints = np.zeros((denseNumber, 3))



	for i in range(numVertices):

		srcPts[i][0] = ply['vertex']['x'][i]
		srcPts[i][1] = ply['vertex']['y'][i]
		srcPts[i][2] = ply['vertex']['z'][i]

		srcColours[i][0] = ply['vertex']['red'][i]
		srcColours[i][1] = ply['vertex']['green'][i]
		srcColours[i][2] = ply['vertex']['blue'][i]

	for i in range(numPlaneVertices):
		planePts[i][0] = plane['vertex']['x'][i]
		planePts[i][1] = plane['vertex']['y'][i]
		planePts[i][2] = plane['vertex']['z'][i]
		planeColours[i][0] = plane['vertex']['red'][i]
		planeColours[i][1] = plane['vertex']['green'][i]
		planeColours[i][2] = plane['vertex']['blue'][i]
		

	# densePoints[i][0] = denseCloud['vertex']['x'][i]
	#densePoints[i][1] = denseCloud['vertex']['y'][i]
	#densePoints[i][2] = denseCloud['vertex']['z'][i]
	#print("red: " + str(srcColours[i][0]) + "\nblue : " + str(srcColours[i][1]) + "\ngreen: " + str(srcColours[i][2]))
	#compute convexhull
	#print(srcPts)
	#print(srcPts[0, :])

	cov_Comp = np.transpose(srcPts)
	#print(cov_Comp)
	cov_mat = np.cov([cov_Comp[0,:],cov_Comp[1,:],cov_Comp[2,:]])

	eigen_Values, eigen_Vectors = np.linalg.eig(cov_mat)

	# print('Covariance Matrix:\n', cov_mat)
	print('Eigen vectors:\n', eigen_Vectors)
	print('\neigen values: \n', eigen_Values)

	#compute the table plane.
	#print("Computing plane..")
	#plane = estimatePlaneRansac(densePoints)
	#planeBasis = computePlaneBasis(plane)
	#print(planeBasis)

	changeBasis = cobMatrix(eigen_Vectors, standardBasis())
	print("Our change of basis: " + str(changeBasis))
	mean = computeMean(srcPts, numVertices)
	# print("OUR MEAN: " + str(mean))

	# print(planeBasis[0].dot(planeBasis[1]))
	#print(planeBasis[0].dot(planeBasis[2]))
	#print(planeBasis[1].dot(planeBasis[2]))
	# planeToStandard = cobMatrix(planeBasis, standardBasis())
	# print("Our transform test: 1st planeElement " + str(np.dot(planeToStandard, planeBasis[0])))
	# print("Our transform test: 2nd planeElement " + str(np.dot(planeToStandard, planeBasis[1])))
	# print("Our transform test: 3rd planeElement " + str(np.dot(planeToStandard, planeBasis[2])))
	#
	print("\ndone")

	#transPlane = transformedPlane(planeToStandard, plane)
	#print("Our plane + : " + str(transPlane))
	#project to plane

	twoDPoints = np.zeros((numVertices, 2))
	zElement = np.zeros((numVertices, 1))
	newPoints = np.zeros((numVertices, 3))
	greaterVertices = 0
	lesserVertices = 0
	tag = 0
	if(numVertices > numPlaneVertices):
		greaterVertices = numVertices
		#the srcPts have more vertices
		tag = 0
	else:
		greaterVertices = numPlaneVertices
		#the plane has more vertices
		tag = 1
		lesserVertices = numVertices

	if(tag):
		for i in range(greaterVertices):
			#change basis
			planePts[i] = np.matmul(changeBasis, planePts[i])
			if(lesserVertices - i > 0):
				srcPts[i] = np.matmul(changeBasis, srcPts[i])
				#newPoint = secondProject(srcPts[i], computeYZPlane())
				#newPoints[i] = newPoint

				##for dorsal
				# twoDPoints[i][0] = srcPts[i][1]
				# twoDPoints[i][1] = srcPts[i][2]
				twoDPoints[i][0] = srcPts[i][0]
				twoDPoints[i][1] = srcPts[i][1]
				zElement[i] = srcPts[i][2]

	else:
		for i in range(greaterVertices):
			srcPts[i] = np.matmul(changeBasis, srcPts[i])
			#newPoint = secondProject(srcPts[i], computeYZPlane())

			#newPoints[i] = newPoint

			##for dorsal
			#twoDPoints[i][0] = srcPts[i][1]
			#twoDPoints[i][1] = srcPts[i][2]
			twoDPoints[i][0] = srcPts[i][0]
			twoDPoints[i][1] = srcPts[i][1]
			zElement[i] = srcPts[i][2]
			if(lesserVertices - i > 0):
				planePts[i] = np.matmul(changeBasis, planePts[i])

# recontexCent(srcPts, cent, numVertices)

	p = estimatePlaneRansac(planePts)
	writePlane(p, "ransacPlane")
	#print(srcPts[i][0])
	#print(srcPts[i][1])
	#print(srcPts[i][2])
	# write(twoDPoints, srcColours, "PlaneTest")


	#write(srcPts, srcColours, "UnprojectedOriginal")
	#write(newPoints, srcColours, "NewPointsproj")
	# for i in range(numVertices):
	#srcPts[i][2] = 0

	hull = ConvexHull(twoDPoints)
	cent = centroid(srcPts, numVertices)
	tgtPts = np.zeros((hull.vertices.size, 3))
	# Just a copy for now
	print(' - Planification')
	# Find the best plane and project all points onto it

	#for i in range(numVertices):
	#  tgtPts[i] = project(srcPts[i], plane)
	# Update the Ply file data

	#vertexFill = np.zeros(hull.vertices.size*3, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

	tgtColours = np.zeros((hull.vertices.size, 3))
	tgtNormals = np.zeros((hull.vertices.size, 3))


	for i in range(hull.vertices.size):

		##for dorsal##
		#tgtPts[i][0] = 0
		#tgtPts[i][2] = twoDPoints[hull.vertices[i]][1]
		#tgtPts[i][1] = twoDPoints[hull.vertices[i]][0]

		tgtPts[i][0] = twoDPoints[hull.vertices[i]][0]
		tgtPts[i][1] = twoDPoints[hull.vertices[i]][1]
		tgtPts[i][2] = 0

		#recolour data
		tgtColours[i][0] = 255
		tgtColours[i][1] = 0
		tgtColours[i][2] = 0
	#print("red: " + str(tgtColours[i][0]) + "\nblue : " + str(tgtColours[i][1]) + "\ngreen: " + str(tgtColours[i][2]))

	#map to plane:
	
	hullCentroid = centroid(tgtPts, hull.vertices.size)
	# recontexCent(tgtPts, hullCentroid, hull.vertices.size)
	print(cent)
	cent.reshape(3, 1)

	##dorsal centroid target = computeYZPlane()
	centroidTarget = secondProject(cent, computeXYPlane())
	translateVector = centroidTarget - cent
	for i in range(numVertices):
		srcPts[i] = translateToHull(srcPts[i], translateVector)

	for i in range(numPlaneVertices):
		planePts[i] = translateToHull(planePts[i], translateVector)

	write(planePts, planeColours, "Reorientated_Plane")
	write(srcPts, srcColours, "Reorientated_Surface")
	write(tgtPts, tgtColours, "ConvexHull")

	#flat = tgtPts.flatten()

	#print("Our tgt : ", flat)
	#print(vertex.shape)


	#flat.dtype

	# hullPly = PlyElement.describe(flat, 'vertex')
	#print(hull.vertices)
	print(' - Writing data')
	for i in range(numVertices):

		#hullPly = PlyElement.describe(srcPts[hull.vertices[i]], 'vertex')
		ply['vertex']['x'][i] = srcPts[i][0]
		ply['vertex']['y'][i] = srcPts[i][1]
		ply['vertex']['z'][i] = srcPts[i][2]
	# Write the result
	ply.write(tgt)


for f in os.listdir('output'):
	if os.path.isdir('output/' + f):
		print(f)
		src = 'output/' + f + '/' + f + '_point_cloud.ply'
		plane = 'output/' + f + '/' + f + '_plane.ply'
		tgt = 'output/' + f + '/' + f + '_point_cloud_planified.ply'
		planify(src, plane, tgt)
