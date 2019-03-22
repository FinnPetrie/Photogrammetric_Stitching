/*
 * Measure.cpp
 *
 *  Created on: 14/02/2019
 *      Author: finn
 */

#include "Measure.h"
#include <vector>
using namespace Eigen;


Measure::Measure(PlyFile toMeasure) : toMeasure(toMeasure){
	covarianceMat = toMeasure.covariance();
	// TODO Auto-generated constructor stub

}

Measure::~Measure() {
	// TODO Auto-generated destructor stub
}


/* Measurement function, depening on i, a different measurement is computed.
 * 0 for maximum thickness, 2 for maximum dimension.
 *
 * returns the measurement.
 */
double Measure::measure(int i){
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covEigens(covarianceMat);
		Eigen::Vector3d dimension = covEigens.eigenvectors().col(i);

		//project all points, take the furtherest points.
		std::vector<Eigen::Vector3d> projectedPoints;
		Eigen::Vector3d projected;
		for(int i = 0; i < toMeasure.size(); i++){
			projected = project(dimension, toMeasure[i].location);
			projectedPoints.push_back(projected);
		}

		double largestMagnitude = 0;
		double magnitude;

		Eigen::Vector3d pairs;
		//or, we could check the values along the concerned axis, choosing the most positive, and the most negative, subtract them, and take their norm
		for(int i = 0 ; i < projectedPoints.size(); i++){
			for(int j = 0; j < projectedPoints.size(); j++){
				if(j != i){
					pairs = projectedPoints[i] - projectedPoints[j];
					magnitude = pairs.norm();
					if(magnitude > largestMagnitude){
						largestMagnitude = magnitude;
					}

				}
			}
		}
		return largestMagnitude;
}


/* Computes maximum dimension using measure(int i)
 * returns maximum dimension
 *
 */double Measure::maximumDimension(){
	return measure(2);

}


/* Projects a vector to another vector.
 * Parameters:
 * a - the vevtor to project.
 * b - the vector to project onto.
 *
 * returns the projected vector into vector form.
 */

Eigen::Vector3d Measure::project(Eigen::Vector3d a, Eigen::Vector3d b){
	double normA = a.norm();
	double scalarProj = a.dot(b)/normA;

	return scalarProj*a/normA;


}


Eigen::Vector3d Measure::highestPoint(){

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covEigens(covarianceMat);
			Eigen::Vector3d dimension = covEigens.eigenvectors().col(0);

			//project all points, take the furtherest points.
			std::vector<Eigen::Vector3d> projectedPoints;
			Eigen::Vector3d projected;
			for(int i = 0; i < toMeasure.size(); i++){
				projected = project(dimension, toMeasure[i].location);
				projectedPoints.push_back(projected);
			}

			double largestMagnitude = 0;
			double magnitude;
			Eigen::Vector3d highestPoint;
			for(int i =0 ; i < projectedPoints.size(); i++){
				magnitude = projectedPoints[i].norm();
				if(magnitude > largestMagnitude){
					largestMagnitude = magnitude;
					highestPoint = toMeasure[i].location;
				}

			}
			return highestPoint;

}

/* measures maximum thickness
 *
 */
double Measure::maximumThickness(){
	//measuring the thickness is the same as measuring the dimension, but instead the points are projected onto the
	//the principal component with the least variation, i.e., column 0 in Eigen's covariance eigenvector matrix.
	return measure(0);
}
