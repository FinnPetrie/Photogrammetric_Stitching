#include "CircularDemons.h"
#include <cmath>

CircularDemons::CircularDemons(PlyFile fHull, PlyFile dHull, PlyFile fSurface, PlyFile dSurface): fixedHull(fHull), dynamicHull(dHull), fixedSurface(fSurface), dynamicSurface(dSurface){
	Eigen::Vector3d centre = fixedHull.centroid();
	Eigen::Vector3d dynamicCentre = dynamicHull.centroid();

	fixedCircle.setCentre(centre);
	dynamicCircle.setCentre(dynamicCentre);
}



void CircularDemons::run(){
	//compute the curvature along each convex hull. Append this curvature to the respective points.

	//project the points to the circle
	std::vector<Vertex> projections;
	for(int i =0 ; i < fixedHull.size(); i++){
		std::cout << i << std::endl;
		Eigen::Vector3d p = fixedHull[i].location;
		std::cout << p << std::endl;
		Eigen::Vector3d proj = fixedCircle.projectToCircle(p);
	//	double t = atan(p[1]/p[2]);
		//Eigen::Vector3d toCircum(1, 0, t);
		//std::cout << toCircum << std::endl;
		Vertex v;
		v.location = proj;
		projections.push_back(v);

	}
	PlyFile pro(projections);
	pro.write("CircleProjection.ply");

}
