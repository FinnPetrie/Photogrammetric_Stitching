#include <string>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "rply.h"
#include "PlyFile.h"
#include "KMeans.h"
#include "IterativeClosestPoint.h"
#include <eigen3/Eigen/Dense>
#include "Measure.h"
#include "Plane.h"
#include "Surface.h"
#include "Procrustes.h"
#include "CircularDemons.h"
#include "CubicSpline.h"

/* Helper method to create a rotation matrix as Eigen is a bit weird with this.
 *
 */

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}



/* main function, refer to the steps in the README.md
 *
 * The steps here are commented out. TODO - move each step to helper functions.
 *
 */

//what I want to do: read in a ventral surface, and its plane + the convex hull. Do the same with a dorsal surface.
//match the dorsal convex to the ventral convex with ICP. The rotations will be used on the plane and the surface.
//this works, now need to find the highest point a surface, project it to the opposite surface's plane, and use this vector projection as the translation





void translatePlanes(Plane ventralPlane, Plane dorsalPlane, PlyFile dorsalSurface, PlyFile ventralSurface){
	Measure dorsalHeight(dorsalSurface);
	Measure ventralHeight(ventralSurface);
	Eigen::Vector3d highestVentral = ventralHeight.highestPoint();
	std::cout << "\nVentral : " << highestVentral << std::endl;

	Eigen::Vector3d highestDorsal = dorsalHeight.highestPoint();

	std::cout << "Dorsal: " << highestDorsal << std::endl;

	//project the highest point on the dorsal surface to the ventral plane, and vice versa. These are our transforms.
	Eigen::Vector3d transDorsal = ventralPlane.projectToPlane(highestDorsal);
	Eigen::Vector3d transVentral = dorsalPlane.projectToPlane(highestVentral);

	//translate each point cloud.
	dorsalSurface.translateCloud(transDorsal);
	ventralSurface.translateCloud(transVentral);

	//translate each plane.
	ventralPlane.translateCloud(transDorsal);
	dorsalPlane.translateCloud(transVentral);

	dorsalSurface.write("TranslatedDorsal.ply");
	ventralSurface.write("TranslatedVentral.ply");

	ventralPlane.write("TransVentPlane.ply");
	dorsalPlane.write("TransDorsalPlane.ply");







}

void computeDemons(){
	PlyFile dorsal("StaticSurface.ply");
	Plane dPlane("StaticPlane.ply.plane", "StaticPlane.ply");
	PlyFile ventral("DynamicSurface.ply");
	Plane vPlane("DynamicPlane.ply.plane", "DynamicPlane.ply");
	PlyFile dorPlane("StaticPlane.ply");
	PlyFile venPlane("RotatedPlane2.ply");

	PlyFile staticHull("StaticHull.ply");
	PlyFile dynamicHull("DynamicHull.ply");
	CubicSpline c(staticHull);

	//c.computeSpline();
	//Surface vent(ventral, vPlane);
	//vent.rotate(0,0);

/**	PlyFile ventral2("RotatedVentral2.ply");
	Plane ventralPlane3(venPlane);
	Plane dorsalPlane3(dorPlane);

	Procrustes p(dorsal, ventral);
	p.removeTranslation();
	Procrustes hull(staticHull, dynamicHull);
	hull.removeTranslation();



	CircularDemons cDemon(hull.getFirst(), hull.getSecond(), p.getFirst(), p.getSecond());
	cDemon.run();*/
}


void splineTest(){
	PlyFile splineTest("StaticHull.ply");
	CubicSpline c(splineTest);
	c.approximateHull();
}

void filter(std::string Ventral, std::string Dorsal){
	PlyFile ventral(Ventral);
	PlyFile dorsal(Dorsal);

	ventral.colourThreshold(Eigen::Vector3i(0,0,0), 25.0, "ventral_filter");
	dorsal.colourThreshold(Eigen::Vector3i(0,0,0), 25.0, "dorsal_filter");


}
int main(void) {
	//filter("Ventral_Fused.ply", "Dorsal_Fused.ply");
	//computeDemons();
	//splineTest();
	PlyFile dynamic("DynamicSurface.ply");
	//std::cout << "Size YO : " << dynamic.size();
	int size = dynamic.size();
	//std:: cout << (size)/2;
	dynamic.sortAlongAxis(1, 0, size);
	//dynamic.write("SortedDynamic.ply");



	return 0;
}
