#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include "PlyFile.h"

class CubicSpline{
private:
	PlyFile points;
	double* fillA(int n, int axis);

public:

	CubicSpline(PlyFile points);
	std::vector<Eigen::Vector4d> computeSpline();

	std::vector<Eigen::Vector4d> computeSplines();

	void drawSplines(std::vector<Eigen::Vector4d> splines);

};

#endif
