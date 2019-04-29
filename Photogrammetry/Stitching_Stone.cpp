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
#include "spline.h"
#include <cmath>
#include  <tinysplinecpp.h>
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


void computeDemons2nd(){
	PlyFile staticHull("StaticHull.ply");
	PlyFile dynamicHull("DynamicHull.ply");

	PlyFile dorsal("StaticSurface.ply");
	PlyFile ventral("DynamicSurface.ply");

	Procrustes p(dorsal, ventral);
	p.removeTranslation();
	Procrustes hull(staticHull, dynamicHull);
	hull.removeTranslation();


	CircularDemons cDemon(hull.getFirst(), hull.getSecond(), p.getFirst(), p.getSecond());
	cDemon.runSpline();
}






void splineTest(){
	PlyFile splineTest("StaticHull.ply");
	CubicSpline c(splineTest);
	c.runLib("LibrarySplines.ply");
}

void filter(std::string Ventral, std::string Dorsal){
	PlyFile ventral(Ventral);
	PlyFile dorsal(Dorsal);

	ventral.colourThreshold(Eigen::Vector3i(0,0,0), 25.0, "ventral_filter");
	dorsal.colourThreshold(Eigen::Vector3i(0,0,0), 25.0, "dorsal_filter");


}

void splineTestTwo(){
	PlyFile staticHull("StaticHull.ply");
	staticHull.orientateAroundYAxis();
//	staticHull.sortAlongAxis(1, 0, staticHull.size());
	CubicSpline c(staticHull);
	c.periodic();
	//c.computeSplines(1, "hello");
}




int tinySplineTest()
{
    // Create a spline...
    tinyspline::BSpline spline(
        3, // ... of degree 3...
        2, // ... in 2D...
        7, // ... consisting of 7 control points...
        TS_CLAMPED // ... using a clamped knot vector.
    );

    // Setup control points.
    std::vector<tinyspline::real> ctrlp = spline.controlPoints();
    ctrlp[0]  = -1.75f; // x0
    ctrlp[1]  = -1.0f;  // y0
    ctrlp[2]  = -1.5f;  // x1
    ctrlp[3]  = -0.5f;  // y1
    ctrlp[4]  = -1.5f;  // x2
    ctrlp[5]  =  0.0f;  // y2
    ctrlp[6]  = -1.25f; // x3
    ctrlp[7]  =  0.5f;  // y3
    ctrlp[8]  = -0.75f; // x4
    ctrlp[9]  =  0.75f; // y4
    ctrlp[10] =  0.0f;  // x5
    ctrlp[11] =  0.5f;  // y5
    ctrlp[12] =  0.5f;  // x6
    ctrlp[13] =  0.0f;  // y6
    spline.setControlPoints(ctrlp);

    // Stores our evaluation results.
    std::vector<tinyspline::real> result;

    // Evaluate `spline` at u = 0.4 using 'evaluate'.
    result = spline.eval(0.4f).result();
    std::cout << "x = " << result[0] << ", y = " << result[1] << std::endl;

    // Derive `spline` and subdivide it into a sequence of Bezier curves.
    tinyspline::BSpline beziers = spline.derive().toBeziers();

    // Evaluate `beziers` at u = 0.3 using '()' instead of 'evaluate'.
    result = beziers(0.3f).result();
    std::cout << "x = " << result[0] << ", y = " << result[1] << std::endl;

    return 0;
}


int main(void) {

	splineTestTwo();
	//computeDemons2nd();
//	tinySplineTest();
	return 0;
}
