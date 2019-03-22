#ifndef PLANE_H_
#define PLANE_H_

#include "PlyFile.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <fstream>

class Plane{

private:

	PlyFile planePly;
	Eigen::Vector3d point;
	Eigen::Vector3d planeNormal;

	bool read(std::string path);


public:
	Plane();
	Plane(PlyFile plane);
	Plane(std::string pathToPlane, std::string pathToPly);

	Eigen::Vector3d computeNormal();
	Eigen::Vector3d normal();
	Eigen::Vector3d projectToPlane(Eigen::Vector3d toProject);


	void translatePlane(Eigen::Vector3d trans);

	bool write(std::string filename);

	void translateCloud(Eigen::Vector3d trans);

	void rotateOrigin(Eigen::Matrix3d rotation);

	void rotateAxis(int axis, double amount);

	void reColour(int r, int g, int b);

	void setPlanePly(PlyFile plane);

	int size();

	void rotatePlane(Eigen::Matrix3d rotation);

	const Vertex& operator[](size_t i) const{
		return planePly[i];
	}

	void updateLocation(Eigen::Vector3d newLocation, int index);
};

#endif
