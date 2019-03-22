/*
 * Measure.h
 *
 *  Created on: 14/02/2019
 *      Author: finn
 */

#ifndef MEASURE_H_
#define MEASURE_H_

#include "PlyFile.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class Measure {

private:
	//the covariance matrix of the point cloud, useful for taking maximum thickness and dimension measurements.
	Eigen::Matrix3d covarianceMat;

	//a pointcloud whose measurements will be taken.
	PlyFile toMeasure;

	/*
	 * used by the public layer measurement functions, maximum thickness and dimension.
	 *
	 * parameter:
	 * i - the index to the covariance eigen-vectors matrix.
	 * Depending on the concerened measurement, this index will be different.
     *
     * returns -
     * the relevant measurement.
	 */
	double measure(int i);

public:
	Measure(PlyFile ply);
	virtual ~Measure();

	//other measurements, yet to be implemented.
	double percussionLength();
	double maximumLength();
	double maximumWidth();

	/*
	 * Computes the maximum thickness via PCA.
     *
     * returns the maximum thickness.
	 */
	double maximumThickness();
	/*
	 * Computes the maximum dimension via PCA.
     *
	 * returns the maximum dimension.
	 */
	double maximumDimension();

    /* Projects a vector to another vector.
     * Here it's used to project a point cloud to a principle component.
     *
     */
	Eigen::Vector3d project(Eigen::Vector3d point, Eigen::Vector3d toProject);


	Eigen::Vector3d highestPoint();
};

#endif /* MEASURE_H_ */
