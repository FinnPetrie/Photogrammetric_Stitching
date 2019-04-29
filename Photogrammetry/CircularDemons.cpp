#include "CircularDemons.h"
#include <cmath>
#include "CubicSpline.h"


CircularDemons::CircularDemons(PlyFile fHull, PlyFile dHull, PlyFile fSurface, PlyFile dSurface): fixedHull(fHull), dynamicHull(dHull), fixedSurface(fSurface), dynamicSurface(dSurface){
	Eigen::Vector3d centre = fixedHull.centroid();
	Eigen::Vector3d dynamicCentre = dynamicHull.centroid();

	fixedCircle.setCentre(centre);
	dynamicCircle.setCentre(dynamicCentre);
}





void CircularDemons::runSpline(){
	//split into positive and negative surfaces.

	//CubicSpline c;
	fixedHull.orientateAroundYAxis();
	fixedHull.sortAlongAxis(1, 0, fixedHull.size());
	std::vector<Vertex> fixedPositive = fixedHull.collectPositiveVertices(2);
	std::vector<Vertex> fixedNegative = fixedHull.collectNegativeVertices(2);

	PlyFile fixedPos(fixedPositive);
	PlyFile fixedNeg(fixedNegative);

	fixedPos.write("FixedPos.ply");
	fixedNeg.write("FixedNeg.ply");

	CubicSpline c(fixedHull);
	c.runLib("FixedHull");
	std::cout << "finshing  compute splines" << std::endl;

	PlyFile positiveSplines("fixedSplinesPos.ply");
	PlyFile fixedSplines("FixedHull");
	PlyFile negativeSplines("fixedSplinesNeg.ply");



	///to do: compute the splines for the divisions
	///fuse the computed spline into one ply.
	///project t



	//project the spline to the circle, we want this to be bijective.
	//for a set of samples along the arclength of the circle, project the spline curve to that point, and associate curvature

	/**for(int i = 0; i < 180; i++){
		//take the ith value of the spline

	}*/





}

void CircularDemons::run(){
	//compute the curvature along each convex hull. Append this curvature to the respective points.

	//project the points to the circle
	std::vector<Vertex> projections;

	std::vector<Vertex> dynamicProjections;
	for(int i =0 ; i < fixedHull.size(); i++){
		std::cout << i << std::endl;
		Eigen::Vector3d p = fixedHull[i].location;
		double curvature = fixedHull.curvatureAtPoint(i);
		std::cout << p << std::endl;
		std::cout << curvature << std::endl;
		Eigen::Vector3d proj = fixedCircle.projectToCircle(p);

		Vertex v;
		v.location = proj;
		v.curvature = curvature;
		projections.push_back(v);

	}

	for(int i =0 ; i < dynamicHull.size(); i++){
		Eigen::Vector3d p = dynamicHull[i].location;
		double curvature = dynamicHull.curvatureAtPoint(i);
		Eigen::Vector3d proj = dynamicCircle.projectToCircle(p);

		Vertex v;
		v.location = proj;
		v.curvature = curvature;
		dynamicProjections.push_back(v);
	}

	PlyFile fixedPro(projections);
	fixedPro.write("CircleProjection.ply");
	PlyFile dynPro(dynamicProjections);
	dynPro.write("DynamicCircProj.ply");
	//pro.write("CircleProjection.ply");

	//here we rotate, incrementally by one degree
	Eigen::Matrix3d rotations;
	double smallestError = std::numeric_limits<double>::infinity();
	double bestRotation = 0;
	for(int i = 0 ; i < 360; i++){
		//rotate dynamic circle by one degree
		Eigen::AngleAxis<double> angle(i*M_PI/180,Eigen::Vector3d(1,0, 0) );
		rotations = angle;
		dynPro.rotateCloud(rotations);

		//for each projected point, i.e., dynPro, find closest point in fixedProjection.
		double e = error(fixedPro, dynPro);
		std::cout << i << std::endl;
		std::cout << e << std::endl;
		if(e < smallestError){
			smallestError = e;
			bestRotation = i;
		}

	}
	std::cout << bestRotation << std::endl;

	Eigen::AngleAxis<double> angle(bestRotation*M_PI/180, Eigen::Vector3d(1,0,0));
	rotations = angle;
	dynPro.rotateCloud(rotations);
	dynPro.write("CirleDynamic.ply");
	dynamicHull.rotateCloud(rotations);
	dynamicSurface.rotateCloud(rotations);

	fixedSurface.write("CircularDemonsStatic.ply");
	dynamicHull.write("CircularDemonsDynamic.ply");
	dynamicSurface.write("CircularDemonsDynamicSurface.ply");

}


double CircularDemons::error(PlyFile fixedPro, PlyFile dynPro){
	double error;
	for(int i =0 ; i < fixedPro.size(); i++){
		Vertex v = closestPoint(fixedPro[i].location, dynPro);
		error += fixedPro[i].curvature - v.curvature;
	}
	return abs(error);

}

Vertex CircularDemons::closestPoint(Eigen::Vector3d l, PlyFile dynPro){
	int closestIndex;

		double closestPointDistance = std::numeric_limits<double>::infinity();
		for(int j = 0 ; j < dynPro.size(); j++){
			Eigen::Vector3d fD = l - dynPro[j].location;
			double currentDistance = fD.norm();
			if(currentDistance < closestPointDistance){
				closestIndex = j;
				closestPointDistance = currentDistance;
			}
		}

	return dynPro[closestIndex];
}
/**
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
*/
