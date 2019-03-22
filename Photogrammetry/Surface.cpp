#include "Surface.h"

Surface::Surface(PlyFile surface, Plane plane) : surface(surface), plane(plane){
	augmentation = surface + plane.getPlanePly();
}


void Surface::rotate(int axis, double amount){
	Eigen::Matrix3d cov = augmentation.covariance();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covEigens(cov);


	std::cout << "The eigenvalues of A are: \n" << covEigens.eigenvalues() << std::endl;
		std::cout << "Here's a matrix whose columns are eigenvectors of A \n"
		        << "corresponding to these eigenvalues:\n"
		        << covEigens.eigenvectors() << std::endl;

		std::cout << "\n\n\"" << std::endl;
		std::cout << cov << std::endl;
		std::cout << "The third eigenvector of the 3x3 covariance matrix is:"
		     << std::endl << covEigens.eigenvectors().col(2) << std::endl;

		 Eigen::Matrix3d m;
		 Eigen::Matrix3d z;
		 m = Eigen::AngleAxisd(1.05*M_PI, covEigens.eigenvectors().col(2));
		 z = Eigen::AngleAxisd(-1*M_PI, covEigens.eigenvectors().col(0));
		 Eigen::Matrix3d mz = z * m;

		 surface.rotateOrigin(mz);
		 //surface.translateCloud(Eigen::Vector3d(0, 0.05, -0.03));

		 //surface.rotate(z.transpose());
		// drawVectors(covEigens.eigenvectors().col(2), 30);

		 plane.rotateAboutPoint(mz, surface.centroid());
		// plane.translateCloud(Eigen::Vector3d(0, -0.05, 0.03));

		 //plane.rotateOrigin(z.transpose());
		// augmentation.rotateOrigin(m);
		// augmentation.rotateOrigin(z);
		 surface.write("RotatedVentral2.ply");
		 plane.write("RotatedPlane2.ply");
		 augmentation.write("RotatedAugment.ply");



}

void Surface::drawVectors(Eigen::Vector3d t, int x){
	for(int i = 0; i < x; i++){
		Vertex v;
		v.location = (t*x + t);
		v.colour = Eigen::Vector3i(0, 255, 0);
		surface.push_back(v);
	}

}


