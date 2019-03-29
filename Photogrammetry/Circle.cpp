#include "Circle.h"

Circle::Circle(){
	radius = 1;

	centre = Eigen::Vector3d(0,0,0);

}

Circle::Circle(Eigen::Vector3d centre, double radius): centre(centre), radius(radius){

}



void Circle::setRadius(double radi){
	this->radius = radi;
}

void Circle::setCentre(Eigen::Vector3d cent){
	this->centre = cent;
}



Eigen::Vector3d Circle::getCentre(){
	return this->centre;
}

double Circle::getRadius(){
	return this->radius;
}

Eigen::Vector3d Circle::projectToCircle(Eigen::Vector3d point){
	Eigen::Vector3d p = point;
	p = point - centre;

	double pNorm = p.norm();

	p = p/pNorm;
	p = p *radius;
	p = p + centre;

	return p;
}
