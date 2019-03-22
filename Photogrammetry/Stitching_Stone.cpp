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
int main(void) {

	/**PlyFile ventralPlane("Ventral_Reorientated_Plane.ply");
	PlyFile ventral("Ventral_Reorientated_Surface.ply");
	PlyFile ventralConvex("VentralConvexHull.ply");
	Plane vPlane("Ventral_ransacPlane.plane", "Ventral_Reorientated_Plane.ply");

	PlyFile dorsalPlane("Dorsal_Reorientated_Plane.ply");
	PlyFile dorsal("Dorsal_Reorientated_Surface.ply");
	PlyFile dorsalConvex("DorsalConvexHull.ply");
	Plane dPlane("Dorsal_ransacPlane.plane", "Dorsal_Reorientated_Plane.ply");

	//IterativeClosestPoint i(dorsalConvex, ventralConvex, ventral, dorsal, dPlane, vPlane);
	//i.computeStochastic(0.0001, 5, true);*/

	PlyFile dorsal("StaticSurface.ply");
	Plane dPlane("StaticPlane.ply.plane", "StaticPlane.ply");
	PlyFile ventral("DynamicSurface.ply");
	Plane vPlane("DynamicPlane.ply.plane", "DynamicPlane.ply");

	ventral.rotateAxis(2, -1);
	ventral.write("RotatedVentral.ply");


	//translatePlanes(vPlane, dPlane, ventral, dorsal);
	//PlyFile ventral("DynamicSurface.ply");
	//PlyFile ventralPlane("DynamicPlane.ply");
	//Plane vPlane(ventralPlane);


	//Surface newSurface(ventral, ventralPlane);
	//PlyFile newPly = ventral + ventralPlane;
	//std::cout << "New ply created" << std::endl;
	//newPly.write("AugmentedVentral.ply");

	//newSurface.rotate(0,0);

	//ventral.augment(ventralPlane);


	//translatePlanes(vPlane, dPlane, dorsal, ventral);
	//IterativeClosestPoint i(dorsalConvex, ventralConvex, ventral, dorsal, dorsalPlane, ventralPlane);
	//i.computeStochastic(0.0001, 5, true);

	//ply = ply.colourThreshold(Eigen::Vector3i(0, 0, 0), 20, "Dorsal_Filtered");

	/**
	PlyFile dorsalConv("DorsalConvexHull.ply");
	PlyFile ventralConv("VentralConvexHullXY.ply");
	PlyFile orgVentral("VentralRealignedXY.ply");
	PlyFile orgDorsal("DorsalRealigned.ply");



	//PlyFile ply("Dorsal_Fused.ply");
//	PlyFile stitching("Stitching.ply");
	//PlyFile stiched("Stiched.ply");

	//ply = ply.colourThreshold(Eigen::Vector3i(0, 0, 0), 25, "VentralcolourFilteredDense");
//	PlyFile colourFiltered("VentralcolourFilteredDense_Reformed.ply");
	//KMeans k(11, &colourFiltered);
	//k.clustering(0.1, "VentralCluster ");

	//for(int i = 0; i < dorsal.size(); i++){
		//dorsal[i].location = dorsal[i].location + Eigen::Vector3d(5, 5, 5);
	//}


	Eigen::Affine3d rotate = create_rotation_matrix(0.0, 0.5, 0.0);
	Eigen::Matrix4d fourRotate = rotate.matrix();
	Eigen::Matrix3d ro;
	ro << fourRotate(0,0), fourRotate(0,1), fourRotate(0,2),
					fourRotate(1,0), fourRotate(1, 1), fourRotate(1,2),
					fourRotate(2, 0), fourRotate(2,1), fourRotate(2,2);
	//std::cout << ro << std::endl;
	 * */

	//stiched.rotateCloud(ro);

/**


	Eigen::Matrix3d cov = ventral.covariance();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covEigens(cov);
	std::cout << "The eigenvalues of A are: \n" << covEigens.eigenvalues() << std::endl;
	std::cout << "Here's a matrix whose columns are eigenvectors of A \n"
	        << "corresponding to these eigenvalues:\n"
	        << covEigens.eigenvectors() << std::endl;

	std::cout << "\n\n\"" << std::endl;
	std::cout << cov << std::endl;
	std::cout << "The second eigenvector of the 3x3 covariance matrix is:"
	     << std::endl << covEigens.eigenvectors().col(2) << std::endl;

	 Eigen::Matrix3d m;
	 Eigen::Matrix3d z;
	 m = Eigen::AngleAxisd(M_PI, covEigens.eigenvectors().col(2));
	 z = Eigen::AngleAxisd(-0.8*M_PI, covEigens.eigenvectors().col(0));
	// stiched.rotateOrigin(reflection);
	ventral.rotateAxis(2, 1);
	ventral.rotateAxis(0, -0.8);
	ventralPlane.rotateOrigin(m);
	ventralPlane.rotateOrigin(z);
	//vPlane.rotateAxis(2, 0.8);
	//vPlane.rotateAxis(0, -0.8);
	//vPlane.rotateOrigin(m);
	//vPlane.reColour(0, 128, 128);
	// vPlane.write("RotatedVentralPlane.ply");
	// stiched.translateCloud(Eigen::Vector3d(-0.01, 0.06, -0.05));
	 ventral.reColour(0, 255, 0);
	 ventral.write("RotatedVentral.ply");
	 ventralPlane.write("RotatedPlaneVent.ply");




	//PlyFile aug("AugmentedStitch.ply");

	//Measure measure(aug);

	//std::cout << measure.maximumDimension() << std::endl;
	//std::cout << measure.maximumThickness() << std::endl;







//	Eigen::Vector3d translate = Eigen::Vector3d(0.5, 0.5, 0.5);
	//for(int i =0 ; i < dorsalConvex.size(); i++){
	//	Eigen::Vector3d rot = ro * dorsalConvex[i].location + translate;
//
	//	toRotate.updateLocation(rot, i);
	//}*/
	//toRotate.write("DynamicRotate.ply");



	//i.compute(0.1);
	//ply.print();
	//k.print();

	return 0;
}
