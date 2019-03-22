#ifndef SURFACE_H_
#define SURFACE_H_

#include "PlyFile.h"
#include "Plane.h"
class Surface{
private:
	PlyFile surface;
	//PlyFile plane;
	Plane plane;
	PlyFile augmentation;
public:

	Surface(PlyFile surface, Plane plane);

	void rotate(int axis, double amount);
	void drawVectors(Eigen::Vector3d t, int x);

};

#endif
