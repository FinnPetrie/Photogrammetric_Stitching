#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include "PlyFile.h"
#include "spline.h"

class CubicSpline{
private:
	PlyFile points;
	double* fillA(int n, int axis);
	std::vector<Eigen::Vector4d> splines;
	std::vector<double> solutionVector;

public:

	CubicSpline(PlyFile points);
	std::vector<Eigen::Vector4d> computeSpline();

	std::vector<Eigen::Vector4d> computeSplines(int axis, std::string t);

	std::vector<Eigen::Vector4d> computeSplineYZPlane(std::string t);
	std::vector<Eigen::Vector4d> computeSplinesYZ(std:: string t);

	double evaluateAt(double x, int axis);
	void drawSplines(std::vector<Eigen::Vector4d> splines, int axis, std::string t);

	void runLib(std::string s);
	void drawSplinesYZ(std::vector<Eigen::Vector4d> splines, std::string t);
	void approximateHull();

	void periodic();
	void Interpolate( std::string t, int resolution, std::vector<double> h);

};

#endif
