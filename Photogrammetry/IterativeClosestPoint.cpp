/*
 * IterativeClosestPoint.cpp
 *
 *  Created on: 23/01/2019
 *      Author: finn
 */

#include "IterativeClosestPoint.h"
#include <cstdlib>
#include <math.h>
using namespace Eigen;


/* General constructor, use this if you do not want to match convex hulls.
 *
 * parameters:
 * stcPts - these points will are static, the algorithm will compute the matrix and vector necessary to match the dynamic points to this cloud.

 * dynPts - these points are dynamic, they will be rotated and translated by the algorithm until
 * an optimisation is reached.
 */
IterativeClosestPoint::IterativeClosestPoint(PlyFile stcPts, PlyFile dynPts): staticPoints(stcPts), dynamicPoints(dynPts){

}


/* Constructor for matching convex hulls. Essentially a 'stitching constructor'
 *
 * parameters:
 *  stcPts - the source pointcloud, ICP will use this as the static cloud to match the dynamic cloud. In this case it would be a convex hull.
 *  dynPts - the dynamic pointcloud. ICP will match this to the static cloud.
 *  mtchPts - the points that will be matched, i.e., these points will be 'stitched' onto the static surface
 *  opsSrf - the opposing surface, this is the surface belonging to the convex hull, essentially a static point cloud.
 */
IterativeClosestPoint::IterativeClosestPoint(PlyFile stcPts, PlyFile dynPts, PlyFile mtchPts, PlyFile opsSrf): staticPoints(stcPts), dynamicPoints(dynPts), matchPoints(mtchPts), opposingSurface(opsSrf){

}



/* Constructor for matching convex hulls and the surface's plane. Essentially a 'stitching constructor'
 *
 * parameters:
 *  stcPts - the source pointcloud, ICP will use this as the static cloud to match the dynamic cloud. In this case it would be a convex hull.
 *  dynPts - the dynamic pointcloud. ICP will match this to the static cloud.
 *  mtchPts - the points that will be matched, i.e., these points will be 'stitched' onto the static surface
 *  opsSrf - the opposing surface, this is the surface belonging to the convex hull, essentially a static point cloud.
 *  stcPln - the static cloud's plane. Will be used to remove noise from the reconstruction.
 *  dynPln - the dynamic cloud's plane.
 */
IterativeClosestPoint::IterativeClosestPoint(PlyFile stcPts, PlyFile dynPts, PlyFile mtchPts, PlyFile opsSrf, PlyFile stcPln, PlyFile dynPln): staticPoints(stcPts), dynamicPoints(dynPts), matchPoints(mtchPts), opposingSurface(opsSrf), staticPlane(stcPln), dynamicPlane(dynPln){
planeObject = false;

}

IterativeClosestPoint::IterativeClosestPoint(PlyFile stcPts, PlyFile dynPts, PlyFile mtchPts, PlyFile opsSrf, Plane stcPln, Plane dynPln): staticPoints(stcPts), dynamicPoints(dynPts), matchPoints(mtchPts), opposingSurface(opsSrf), planeStatic(stcPln), planeDynamic(dynPln){
planeObject = true;

}





/* Stochastic implementation of ICP. This uses a random sampling technique to avoid local minima.
 *
 *
 * parameters:
 * threshold - an error threshold, the program will halt when the error is beneath this.
 * numRandSamples - the number of samples to be randomly chosen from the point clouds.
 * isStiching - tells the algorithm whether to apply the resultant transforms to the
 * matching point cloud.
 */
void IterativeClosestPoint::computeStochastic(double threshold, int numRandSamples, bool isStitching){
	//closestPoints should be an array, the index i being the closest point in the targetPoints for source point i.

 	double closestPointDistance;
	Matrix3d rotation;
	//identity matrix assignment
	rotation << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;

	Vector3d translation(0, 0, 0);
	//MatrixXd(sourcePoints.size(), targetPoints.size());
	double e;
	int iteration = 0;
	double currentDistance;

	Vector3d staticCentroid = centroid(staticPoints);



	do{
		Matrix3d W(3,3);

		Vertex closestPoints [numRandSamples] = {};
		int dynamicRandIndex[numRandSamples] = {};

		//reset error
		e = 0;
		//find the closest point in the target cloud for every point in the source cloud.
		//speed this up with Kd trees

		//reset matrices
		rotation << 1, 0, 0,
					0, 1, 0,
					0, 0, 1;
		translation << 0, 0, 0;
		int randomIndex;
		Vector3d dynamicCentroid = centroid(dynamicPoints);
	for(int i = 0; i < numRandSamples; i++){

		randomIndex = rand()%dynamicPoints.size();
		while(!indexNotRepeated(randomIndex, dynamicRandIndex)){
			randomIndex = rand()%dynamicPoints.size();
		}
		dynamicRandIndex[i] = randomIndex;
		Vector3d p = dynamicPoints[randomIndex].location;

		closestPointDistance = std::numeric_limits<double>::infinity();
		for(int j = 0; j < staticPoints.size(); j++){
			currentDistance   = EuclideanDistance(p, staticPoints[j].location);
			if(currentDistance < closestPointDistance){
				closestPoints[i] = staticPoints[j];
				closestPointDistance = currentDistance;
			}
		}
	}
	//translate and rotate each point in our target cloud using our rotation and translation matrices, then compute the error.

		//reset W and translation
			W << 0, 0, 0,
				 0, 0, 0,
				 0, 0, 0;

			for(int i = 0; i < numRandSamples; i++){
				Vector3d p = dynamicPoints[dynamicRandIndex[i]].location;
				//get closest point
				Vector3d x = closestPoints[i].location;

				//reorientate points w.r.t to their centroids
				Vector3d qd = p - dynamicCentroid;
				Vector3d qs = x - staticCentroid;
				//Matrix3d wPrime = qs*qd.transpose();
				//add the resulting outer product to W.
				W += qd*qs.transpose();
				//where p is a dynamic point and x a static.
				e += error(rotation, translation, qd, qs);
			}



			//std::cout << "\nComputing singular value decomposition matrix...\n";

			JacobiSVD<MatrixXd> svd(W, ComputeFullU | ComputeFullV);
		//	std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
//
		///	std::cout << "Its left singular vectors are the columns of the full U matrix:" << std::endl << svd.matrixU() << std::endl;
		//	std::cout << "Its right singular vectors are the columns of the full V matrix:" << std::endl << svd.matrixV() << std::endl;

			Matrix3d V = svd.matrixV();
			Matrix3d U = svd.matrixU();

			Matrix3d X = V*U.transpose();


			double determinant = X.determinant();

			if (determinantCheck(determinant)){
							//std::cout << "Does determinant == 1? " << determinant << "\n";
							rotation = X;
							//std::cout << "\nOur translation : \n" << translation << std::endl;
						}else{
						//	std::cout << "Does determinant == -1 ?  " << determinant << "\n";
						//	std::cout << "V before mod: " << V.col(2) << "\n";
								/**
							 V(0, 2) = -V(0, 2);
							 V(1, 2) = -V(1, 2);
							 V(2, 2) = -V(2, 2);
							 */
							V.col(2) = -V.col(2);
							// std::cout << "V after mod: " << V.col(2) << "\n";
							//std::cout << "The matrix was a reflection" << std::endl;
							//X = V * U.tranpose
							X = V*U.transpose();
							//std::cout << "Our determinant after modification : " << X.determinant() << std::endl;

							rotation = X;
						}

			translation = staticCentroid - rotation*dynamicCentroid;

			//std::cout << "Error check : " << e << std::endl;
			//std::cout << "Our rotation mat : " << rotation << std::endl;


			//update the points.
			for(int i = 0; i < dynamicPoints.size(); i++){
				Vector3d roTransPoint = rotation * dynamicPoints[i].location;
				roTransPoint += translation;
				dynamicPoints.updateLocation(roTransPoint, i);
			}

			if(isStitching){
				for(int i = 0; i < matchPoints.size(); i++){
					Vector3d roTransPoint = rotation * matchPoints[i].location;
					roTransPoint += translation;
					matchPoints.updateLocation(roTransPoint, i);
				}
				applyToPlane(rotation, translation);
			}

			iteration++;

			//write the point cloud incrementally so we can check the rotations.
			//dynamicPoints.writeBlue("DynamicPoints" + std::to_string(writeCount) + ".ply" );
	}while(e > threshold && iteration < 1000);
	//add weights to pairs of points: set the weights in the dependence of the direction of normal vectors as their dot product: Wij = NiNj

	std::cout << "Writing files: " << std::endl;
	write();

	//if matchPoints != Null augment(static, dynamic, match, opposing);
	return;

}


/* helper method, used here to see if an index was repeated in the randomly chosen indices.
 * parameters:
 * index - the index to look for
 * d - the array to check for repeats in
 *
 *
 */
bool IterativeClosestPoint::indexNotRepeated(int index, int* d){
	for(int i = 0; i < sizeof(d)/sizeof(d[0]); i++){
		if(d[i] == index){
			return false;
		}
	}
	return true;

}

bool IterativeClosestPoint::write(){
	if(planeObject){
		planeDynamic.write("DynamicPlane.ply");
		planeStatic.write("StaticPlane.ply");
	}else{
		dynamicPlane.write("DynamicPlane.ply");
		staticPlane.write("StaticPlane.ply");

	}
	dynamicPoints.writeBlue("DynamicHull.ply");
	staticPoints.writeRed("StaticHull.ply");
	matchPoints.write("DynamicSurface.ply");
	opposingSurface.write("StaticSurface.ply");

}


void IterativeClosestPoint::applyToPlane(Eigen::Matrix3d rotation, Eigen::Vector3d translation){
	if(planeObject){
	//	std::cout << "translating plane infomation\n";
		//planeDynamic.translatePlane(translation);
		planeDynamic.rotatePlane(rotation);
		planeDynamic.translatePlane(translation);
		for(int i = 0; i < planeDynamic.size(); i++){
		//	std::cout << i << std::endl;
			Vector3d roTransPoint = rotation * planeDynamic[i].location;
			roTransPoint += translation;
			planeDynamic.updateLocation(roTransPoint, i);
		}
	}else{
		std::cout << "IN here NO!" << std::endl;
		for(int i = 0 ; i < dynamicPlane.size(); i++){
			Vector3d roTransPoint = rotation * dynamicPlane[i].location;
			roTransPoint += translation;
			dynamicPlane.updateLocation(roTransPoint, i);
		}
	}
}
/* standard implementation of ICP, this may find local minima, and can generally be slower than
 * the stochastic version.
 *
 * parameters:
 * threshold - if the error falls below this threshold the algorithm will halt.
 */
void IterativeClosestPoint::compute(double threshold){
	//closestPoints should be an array, the index i being the closest point in the targetPoints for source point i.
	Vertex closestPoints [dynamicPoints.size()] = {};


	double closestPointDistance;
	Matrix3d rotation;
	//identity matrix assignment
	rotation << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;

	Vector3d translation(0, 0, 0);
	//MatrixXd(sourcePoints.size(), targetPoints.size());
	double e;
	int iteration = 0;
	Matrix3d W(3,3);
	double currentDistance;

	Vector3d staticCentroid = centroid(staticPoints);
	//dynamicPoints.print();
	/**
	for(int i =0 ; i < staticPoints.size(); i++){
		//have its centre as the origin.
		Vector3d qs = staticPoints[i].location - staticCentroid;
		staticPoints.updateLocation(qs, i);
		}
		*/

	do{

		/**for(int i = 0; i < closestPoints.size(); i++){
			closestPoints[i] = 0;
		}*/

		//reset error
		e = 0;
		//find the closest point in the target cloud for every point in the source cloud.
		//speed this up with Kd trees

		//reset matrices
		rotation << 1, 0, 0,
					0, 1, 0,
					0, 0, 1;
		translation << 0, 0, 0;
		Vector3d dynamicCentroid = centroid(dynamicPoints);
	for(int i = 0; i < dynamicPoints.size(); i++){
		Vector3d p = dynamicPoints[i].location;

		closestPointDistance = std::numeric_limits<double>::infinity();
		for(int j = 0; j < staticPoints.size(); j++){
			currentDistance   = EuclideanDistance(p, staticPoints[j].location);
			if(currentDistance < closestPointDistance){
				closestPoints[i] = staticPoints[j];
				closestPointDistance = currentDistance;
			}
		}
	}
	//translate and rotate each point in our target cloud using our rotation and translation matrices, then compute the error.

		//reset W and translation
			W << 0, 0, 0,
				 0, 0, 0,
				 0, 0, 0;

			for(int i = 0; i < dynamicPoints.size(); i++){
				Vector3d p = dynamicPoints[i].location;
				//get closest point
				Vector3d x = closestPoints[i].location;

				//reorientate intpoints w.r.t to their centroids
				Vector3d qd = p - dynamicCentroid;
				Vector3d qs = x - staticCentroid;
				//Matrix3d wPrime = qs*qd.transpose();
				//add the resulting outer product to W.
				//std::cout << wPrime << std::endl;
				W += qd*qs.transpose();
				//where p is a dynamic point and x a static.
				e += error(rotation, translation, p, x);
			//	std::cout << e << std::endl;
			}


			std::cout << "\nComputing singular value decomposition matrix...\n";

			JacobiSVD<MatrixXd> svd(W, ComputeFullU | ComputeFullV);
			std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;

			std::cout << "Its left singular vectors are the columns of the full U matrix:" << std::endl << svd.matrixU() << std::endl;
			std::cout << "Its right singular vectors are the columns of the full V matrix:" << std::endl << svd.matrixV() << std::endl;

			Matrix3d V = svd.matrixV();
			Matrix3d U = svd.matrixU();

			Matrix3d X = V*U.transpose();

			std::cout << "Our determinant before being assigned : " << X.determinant() << std::endl;


			int determinant = ceil(X.determinant());
			if (determinantCheck(determinant)){
				std::cout << "Does determinant == 1? " << determinant << "\n";
				rotation = X;
				std::cout << "\nOur translation : \n" << translation << std::endl;
			}else{
				std::cout << "Does determinant == -1 ?  " << determinant << "\n";
				std::cout << "V before mod: " << V.col(2) << "\n";

				V.col(2) = -V.col(2);
				 std::cout << "V after mod: " << V.col(2) << "\n";
				std::cout << "The matrix was a reflection" << std::endl;
				X = V*U.transpose();
				std::cout << "Our determinant after modification : " << X.determinant() << std::endl;

				rotation = X;
			}

			translation = staticCentroid - rotation*dynamicCentroid;



			std::cout << "Error check : " << e << std::endl;
			std::cout << "Our rotation mat : " << rotation << std::endl;


			//update the points.
			for(int i = 0; i < dynamicPoints.size(); i++){
				Vector3d roTransPoint = rotation * dynamicPoints[i].location;
			//	std::cout << roTransPoint << std::endl;
				roTransPoint += translation;
				dynamicPoints.updateLocation(roTransPoint, i);
			}

			iteration++;

			//write the point cloud incrementally so we can check the rotations.
			//dynamicPoints.writeBlue("DynamicPoints" + std::to_string(writeCount) + ".ply" );
	}while(e > threshold && iteration < 400);
	//add weights to pairs of points: set the weights in the dependence of the direction of normal vectors as their dot product: Wij = NiNj

	std::cout << "Writing files: " << std::endl;
	dynamicPoints.writeBlue("DynamicSecond.ply");
	staticPoints.writeRed("StaticSecond.ply");
	return;

}



/* incomplete function, an initial implementation of error checking.
 *
 *
 */
double IterativeClosestPoint::error(Matrix3d rotation, Vector3d translation, Vector3d p, Vector3d x){
	double error = 0;
	//rotate p
	Vector3d rotatedP = rotation*p;
	Vector3d errorCheck = x - rotatedP;
	error = errorCheck.dot(errorCheck);

	/**
	for(int i =0 ; i < staticPoints.size(); i++){
		Vector3d errorCheck = staticPoints[i].location - rotation*dynamicPoints[i].location + translation;
			error += errorCheck.dot(errorCheck);
	}
	*/
	//incomplete- include weights- read up at home.
	return error;
}


/* makes sure the determinant is 1, or -1. Necessary because of floating point errors.
 * parameters:
 * x - determinant
 * returns:
 * true if it is 1, false otherwise.
 *
 */
bool IterativeClosestPoint::determinantCheck(double x){
	//std::cout << "Checking x: " << x << std::endl;
	if(x >= 0){
		return true;
	}else{
		return false;
	}
}

/*Computes the euclidean distance using the norm.
 *parameters:
 * v - a vector
 * t - this will be subtracted from v, to find the vector v - t. The euclidean distance will be
 * found from this.
 * returns:
 * double, a measure of distance.
 *
 */
double IterativeClosestPoint::EuclideanDistance(Vector3d v, Vector3d t){
	/**return sqrt(pow(v.location.x() - t.location.x()), 2) + pow((v.location.y() - t.location.y()),2) + pow(v.location.z() - t.location.z(), 2))
	*/
	Eigen::Vector3d p = v - t;
	return p.norm();
}

/**
MatrixXd IterativeClosestPoint::centroidMatrix(Vector3d pointsCentroid, PlyFile points){
	std::cout << "Computing centroid matrix";

	MatrixXd centMat;
	centMat.resize(3, points.size());
//std::cout << centMat;
	for(int i =0 ; i < 3; i++){
		Vector3d currentPoint = points[i].location - pointsCentroid;
		for(int j = 0 ; j < points.size(); j++){
		centMat(i, j) = currentPoint(i);
		}
	}

	return centMat;
}
*/

/* Finds the centroid of a pointcloud, would be better O.O practice to put this in PlyFile.
 * parameters:
 * p - a pointcloud
 * returns:
 * centroid - the 3D vector representing p's centroid.
 *
 */
Vector3d IterativeClosestPoint::centroid(PlyFile p){
	Vector3d centroid(0,0,0);
	for(int i = 0 ; i < p.size(); i++){
		centroid += p[i].location;
	}

	centroid /= p.size();
	return centroid;
}


/*
Matrix3d IterativeClosestPoint::covariance(MatrixXd targetCentroid, MatrixXd sourceCentroid){
	return targetCentroid*sourceCentroid.transpose();
}

/**
void  IterativeClosestPoint::singularValueDecomposition(Vector3d centroidTarget, Vector3d centroidSource){
		Vector3d targetQ[targetPoints.size()] = {};
		Vector3d sourceQ[sourcePoints.size()] = {};

		//
		double covariance;

		for(int i = 0 ; i < sourcePoints.size(); i++){
			sourceQ[i] = sourcePoints[i].location - centroidSource;

		}
		for(int i = 0 ; i < targetPoints.size(); i++){
			targetQ[i] = targetPoints[i].location - centroidTarget;
		}

		for(int i = 0; i < sourcePoints.size(); i++){
			covariance += targetQ[i].dot(sourceQ[i]);
		}
		*/





