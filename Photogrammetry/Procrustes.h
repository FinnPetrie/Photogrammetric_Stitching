#ifndef PROCRUSTES_H_
#define PROCRUSTES_H_

#include <eigen3/Eigen/Core>
#include "PlyFile.h"
#include <iostream>


class Procrustes{
private:
	PlyFile firstShape;
	PlyFile secondShape;


public:

	Procrustes(PlyFile fS, PlyFile sS);
	Eigen::Matrix3d leastSquares(Eigen::Matrix3d covarianceDynamic, Eigen::Matrix3d covarianceFixed);
	void removeTranslation();
	void removeRotation();
	void superImpose();
	void write(std::string first, std::string second);

	PlyFile getFirst();
	PlyFile getSecond();




};

#endif
