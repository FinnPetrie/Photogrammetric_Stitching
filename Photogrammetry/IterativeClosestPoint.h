/*
 * IterativeClosestPoint.h
 *
 *  Created on: 12/03/2019
 *      Author: finn
 */

#ifndef ITERATIVECLOSESTPOINT_H_
#define ITERATIVECLOSESTPOINT_H_

#include "PlyFile.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Dense>
#include "Plane.h"


class IterativeClosestPoint{
	private:

	//points that will stay still
		PlyFile staticPoints;

	//points that will be matched with the static points.
		PlyFile dynamicPoints;

	//points that will be matched to the hull.
		PlyFile matchPoints;

	//points in the opposing surface - either the ventral or dorsal - these points are kept
	//here to easily augment them to the stitched output.

		PlyFile opposingSurface;

	//the plane that the static points lie on.
		PlyFile staticPlane;

	//the plane that the opposing surface, or dynamic points lie on.
		PlyFile dynamicPlane;

		//true if we are representing the plane with a 'Plane' object, false otherwise, i.e., PlyFile representation.
		bool planeObject;

		Plane planeStatic;
		Plane planeDynamic;


	public:

		//constructers
		IterativeClosestPoint(PlyFile source, PlyFile target);
		IterativeClosestPoint(PlyFile source, PlyFile target, PlyFile  matchPoints, PlyFile opposingSurface);
		IterativeClosestPoint(PlyFile source, PlyFile target, PlyFile sourceSurface, PlyFile opposingSurface, PlyFile sourcePlane, PlyFile opposingPlane);
		IterativeClosestPoint(PlyFile source, PlyFile target, PlyFile sourceSurface, PlyFile opposingSurface, Plane sourcePlane, Plane opposingPlane);


		//used by stochastic ICP to make sure that the same index isn't sampled twice.
		bool indexNotRepeated(int index, int* d);

		//runs ICP, the threshold specifies the amount of allowed error for the function to halt.
		void compute(double threshold);

		/*
		 * runs a stochastic version of ICP
		 *
		 * parameters:
		 * threshold - the amount of allowed error for the function to halt
		 * numRanSamples - the number of random samples to be taken, tweaking this will result in faster optimisations
		 * isStitching - a boolean that tells the algorithm whether to apply the rotation and translate to the points to be stitched.
		 */
		void computeStochastic(double threshold, int numRanSamples, bool isStitching);

		/*
		 * checks the whether the determinant is 1, or -1. This function is used to avoid rounding errors present in eigen's .determinant()
		 *
		 * parameters:
		 * x - the determinant the check against.
		 *
		 */


		void applyToPlane(Eigen::Matrix3d rotation, Eigen::Vector3d translation);
		bool determinantCheck(double x);

		/*
		 * Computes the euclidean distance between two vectors based on the pythagorean theorem
		 *
		 * parameters:
		 * v, t - the two vectors, the vector v - t will be used, and the distance computed as its norm.
		 *
		 * returns:
		 * the distance of v - t.
		 */
		double EuclideanDistance(Eigen::Vector3d v, Eigen::Vector3d t);

		/*
		 * Finds the centroid of an input pointcloud p
		 *
		 * parameters:
		 * p - a point cloud.
		 *
		 * returns:
		 * a vector that is the centroid of the input cloud.
		 */

		Eigen::Vector3d centroid(PlyFile p);
		Eigen::Matrix3d covariance(Eigen::MatrixXd model, Eigen::MatrixXd source);
		Eigen::MatrixXd centroidMatrix(Eigen::Vector3d pointsCentroid, PlyFile points);
		double error(Eigen::Matrix3d rotation, Eigen::Vector3d translation, Eigen::Vector3d p, Eigen::Vector3d x);


		bool write();
};





#endif /* ITERATIVECLOSESTPOINT_H_ */
