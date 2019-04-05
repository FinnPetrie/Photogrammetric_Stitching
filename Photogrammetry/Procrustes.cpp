#include "Procrustes.h"

Procrustes::Procrustes(PlyFile fS, PlyFile sS) : firstShape(fS), secondShape(sS){

}


void Procrustes::removeTranslation(){
	firstShape.translateToOrigin(firstShape.centroid());
	secondShape.translateToOrigin(secondShape.centroid());
}

void Procrustes::write(std::string fst, std::string snd){
	firstShape.write(fst);
	secondShape.write(snd);
}


Eigen::Matrix3d Procrustes::leastSquares(Eigen::Matrix3d covarianceDynamic, Eigen::Matrix3d covarianceFixed){
	//Eigen::Matrix3d
}

void Procrustes::removeRotation(){
	//compute covariance matrices eigen-vectors.
	Eigen::Matrix3d firstShapeCovariance = firstShape.covariance();
	Eigen::Matrix3d secondShapeCovariance = secondShape.covariance();

	//we fix the second shape.
	//use least squares fitting to match the eigen vectors


}

PlyFile Procrustes::getFirst(){
	return firstShape;
}

PlyFile Procrustes::getSecond(){
	return secondShape;

}
