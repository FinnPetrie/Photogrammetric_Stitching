#ifndef SURFACE_H_
#define SURFACE_H_

#include "PlyFile.h"

class Surface{
private:
	PlyFile surface;
	PlyFile plane;
	PlyFile augmentation;
public:

	Surface(PlyFile surface, PlyFile plane);

	void rotate(int axis, double amount);
	void drawVectors(Eigen::Vector3d t, int x);

};

#endif
