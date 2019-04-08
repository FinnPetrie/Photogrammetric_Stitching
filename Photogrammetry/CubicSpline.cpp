#include "CubicSpline.h"
#include <eigen3/Eigen/Core>

CubicSpline::CubicSpline(PlyFile p) : points(p){
}

std::vector<Eigen::Vector4d> CubicSpline::computeSpline(){
		//create a new array of size n + 1
	int n = points.size();
	int k = n - 1;

	double a[n + 1];
	double b [n];
	double d [n];
	double u[n];

	//compute a values
	for(int j = 0; j < points.size(); j++){
		Eigen::Vector3d yJ = points[j].location;
		a[j] = yJ[2];
	}

	double h[k];


	//compute h values.
	for(int i = 0; i < k; i++){
		Eigen::Vector3d next = points[i + 1].location;
		Eigen::Vector3d current = points[i].location;
		std::cout << next[0] << std::endl;
		std::cout << current[0] << std::endl;
		double hValue = next[1] - current[1];

		std::cout << hValue;
		h[i] = hValue;
		std::cout << h[i] << "Our h of " << i << std::endl;
	}

	double beta[k];

	for(int i = 1; i < k; i++){
		double aNeighbourHigh = a[i + 1] - a[i];
		double aNeighbourLow = a[i] - a[i -1];

		beta[i] = 3/h[i]*aNeighbourHigh - 3/h[i - 1]*aNeighbourLow;
	}

	double c[n+1], l[n+1], z[n + 1];

	l[0] = 1;
	z[0] = 0;
	u[0] = 0;

	for(int i = 1; i < k ; i++){
		Eigen::Vector3d next = points[i+1].location;
		Eigen::Vector3d previous = points[i - 1].location;
		double xDifference = next[1] - previous[1];

		l[i] = 2*(xDifference) - h[i -1]*u[i - 1];
		u[i] = h[i]/l[i];

		z[i] = (beta[i] -(h[i - 1]*z[i-1]))/l[i];
	}

	l[n] = 1;
	z[n] = 0;
	c[n] = 0;


	for(int i = n - 1; i > 0; i--){
		c[i] = z[i] - u[i]*c[i+1];
		b[i] = ((a[i + 1] - a[i])/h[i]) - (h[i]*(c[i + 1] + 2*c[i]))/3;
		d[i] = (c[i + 1] - c[i])/3*h[i];
	}

	std::vector<Eigen::Vector4d> outputSpline;
	for(int i = 0; i < k - 1; i++){
		Eigen::Vector4d Pi;
		Pi[0] = a[i];
		Pi[1] = b[i];
		Pi[2] = c[i];
		Pi[3] = d[i];
		outputSpline.push_back(Pi);
		std::cout << "Polynomial at interval i : " << i << " = " << Pi[0] << "*x^3 + " << Pi[1] << "*x^2 + " << Pi[2] << "*x + " << Pi[3] << std::endl;
		/*
		std::cout << a[i] << std::endl;
		std::cout << b[i] << std::endl;
		std::cout << c[i] << std::endl;
		std::cout << d[i] << std::endl;
		*/
	}

	return outputSpline;


}
