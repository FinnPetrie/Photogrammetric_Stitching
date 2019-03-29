#ifndef CIRCLE_H_
#define CIRCLE_H_

#include <eigen3/Eigen/Core>

class Circle{
private:
	double radius;
	Eigen::Vector3d centre;

public :

	Circle();
	Circle(Eigen::Vector3d centre, double radius);

	Eigen::Vector3d getCentre();
	double getRadius();

	void setRadius(double radi);
	void setCentre(Eigen::Vector3d centre);
	Eigen::Vector3d projectToCircle(Eigen::Vector3d point);
};

#endif
