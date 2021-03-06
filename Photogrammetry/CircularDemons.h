#ifndef CIRCULAR_DEMONS_H_
#define CIRCULAR_DEMONS_H_


#include "Procrustes.h"
#include "Circle.h"
#include "spline.h"
class CircularDemons{

private:

	PlyFile fixedHull;
	PlyFile dynamicHull;
	PlyFile fixedSurface;
	PlyFile dynamicSurface;

	Circle fixedCircle;
	Circle dynamicCircle;

public:

	CircularDemons(PlyFile fixedHull, PlyFile dynamicHull, PlyFile fixedStone, PlyFile dynamicStone);\

	Vertex closestPoint(Eigen::Vector3d fix, PlyFile dyn);
	double error(PlyFile fixedP, PlyFile dynP);
	void run();
	void runSpline();


};



#endif
