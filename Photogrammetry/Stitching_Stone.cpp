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
	c.approximateHull();
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
	staticHull.sortAlongAxis(1, 0, staticHull.size());


	std::vector<Vertex> neg = staticHull.collectNegativeVertices(2);
	PlyFile negatives(neg);
	int size = negatives.size();
	negatives.sortAlongAxis(1, 0, size);
	negatives.write("NegativesSortedY.ply");
		std::vector<double> x;
		std::vector<double> y;

		std::vector<Vertex> pos = staticHull.collectPositiveVertices(2);
		PlyFile positives(pos);
		positives.sortAlongAxis(1, 0, pos.size());

		//attempt to interpolate between the positive and negative interpolation
		//negatives.push_back(positives[0]);
		//negatives.push_back(positives[positives.size()]);
		//negatives.sortAlongAxis(1,0, negatives.size());

		std::vector<double> xPos;
		std::vector<double> yPos;

		for(int i =0 ; i < positives.size(); i++){
			xPos.push_back(positives[i].location[1]);
			yPos.push_back(positives[i].location[2]);


		}



		for(int i = 0; i < negatives.size(); i++){
			//std::cout << negatives[i].location(1) << std::endl;
		//std::cout << negatives[i].location(2) << std::endl;
			x.push_back(negatives[i].location[1]);

			y.push_back(negatives[i].location[2]);

		}



		tk::spline sNeg;

		sNeg.set_points(x, y);
		std::vector<Vertex> vertices;
		/**
		for(int i =0 ;i < size-1; i++){
			//std::cout <<  "In here" << std::endl;
			//std::cout << negatives.size();
			std::cout << "Next value in x vector : " << x[i+1] << std::endl;
			std::cout << "Current value in x vector : " <<  x[i] << std::endl;
			double difference = abs(x[i+1] - x[i]);

			for(double x1 = 0.0; x1 < difference; x1+= 0.001){
				Vertex v;
				Eigen::Vector3d point;
				std::cout << "Our x increment : " << x1 << std::endl;
				std::cout << "Our spline interpolation : " << s(x1) << std::endl;

				point[0] = 0;
				point[1] = x1 + x[i];
				point[2] = s(x1 + x[i]);
				v.location = point;

				vertices.push_back(v);


			}
			std:: cout << "Difference between next and current value " << difference  << std::endl;

		}
		*/


			//first point
			// draw a set number of points between
			//next point.

		tk::spline sPos;
		sPos.set_points(xPos, yPos);

		for(double x1 = x[0]; x1 < abs(x[negatives.size()] - x[0]); x1 += 0.001){
			std::cout << x1 << std::endl;


			Vertex v;
			Eigen::Vector3d point;
			point[0] = 0;
			point[1] = x1;
			point[2] = sNeg(x1);

			v.location = point;

			vertices.push_back(v);
		}

		//for each interval


		PlyFile negInterp(vertices);
		negInterp.write("NegInterp.ply");


		std::vector<Vertex> posVertices;


		/**
		for(int i = 0; i < positives.size(); i++){
				double nextY = y[i+1];

				do{
					double x1 = x[i];
					double y = sNeg(x1);
					Vertex v;
					Eigen::Vector3d point;
					point[0] = 0;
					point[1] = x1;
					point[2] = y;
					v.location = point;
					posVertices.push_back(v);
					x1 += 0.001;
				}while(y != nextY);
			}*/


		for(double x1Pos = xPos[0]; x1Pos < abs(xPos[positives.size()] - xPos[0]); x1Pos += 0.001){
			Vertex v;
			Eigen::Vector3d point;
			point[0] = 0;
			point[1] = x1Pos;
			point[2] = sPos(x1Pos);

			v.location = point;
			posVertices.push_back(v);
		}

		PlyFile posInterp(posVertices);
		posInterp.write("PosInterp.ply");

}


int main(void) {

	//splineTestTwo();
	computeDemons2nd();

	return 0;
}
