#include "CubicSpline.h"
#include <eigen3/Eigen/Core>

CubicSpline::CubicSpline(PlyFile p) : points(p){
}

std::vector<Eigen::Vector4d> CubicSpline::computeSpline(){
		//create a new array of size n + 1
	int n = points.size();
	std::cout << "This is n: " <<  n << std::endl;
	int k = n;

	double a[n + 1];
	double b [n];
	double d [n];
	double u[n];

	//compute a values
	for(int j = 0; j < points.size(); j++){
		Eigen::Vector3d yJ = points[j].location;
		a[j] = yJ[1];
	}

	double h[k];


	//compute h values.
	for(int i = 0; i < k; i++){
		Eigen::Vector3d next = points[i + 1].location;
		Eigen::Vector3d current = points[i].location;
		std::cout << next[0] << std::endl;
		std::cout << current[0] << std::endl;
		//x here is y
		double hValue = next[0] - current[0];

		std::cout << hValue;
		h[i] = hValue;
		std::cout << h[i] << "Our h of " << i << std::endl;
	}

	double beta[k];

	for(int i = 1; i < k; i++){
		double aNeighbourHigh = a[i + 1] - a[i];
		double aNeighbourLow = a[i] - a[i -1];

		beta[i] = (3/h[i])*aNeighbourHigh- (3/h[i - 1])*aNeighbourLow;
	}

	double c[n+1], l[n+1], z[n + 1];

	l[0] = 1;
	z[0] = 0;
	u[0] = 0;

	for(int i = 1; i < k ; i++){
		Eigen::Vector3d next = points[i+1].location;
		Eigen::Vector3d previous = points[i - 1].location;
		double xDifference = next[0] - previous[0];

		l[i] = 2*(xDifference) - (h[i -1]*u[i - 1]);
		u[i] = h[i]/l[i];

		z[i] = (beta[i] -(h[i - 1]*z[i-1]))/l[i];
	}
	for(int i = 0; i < n; i++){

	}

	l[n] = 1;
	z[n] = 0;
	c[n] = 0;


	for(int i = n - 1; i >= 0; i--){
		c[i] = z[i] -( u[i]*c[i+1]);
		b[i] = (a[i + 1] - a[i])/h[i] - h[i]*(c[i + 1] + 2*c[i])/3;
		d[i] = (c[i + 1] - c[i])/(3*h[i]);
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
	drawSplines(outputSpline);

	return outputSpline;


}

double* CubicSpline::fillA(int n, int axis){
	double a[n];
	for(int i =0 ; i < n; i++){
		Eigen::Vector3d yJ = points[i].location;
		double y = yJ[axis];
		a[i] = y;
	}
	return a;
}

std::vector<Eigen::Vector4d> CubicSpline::computeSplines(){
	int n = points.size();
	n--;
	double a[n + 1];

	for(int i =0 ; i < n + 1; i++){
		Eigen::Vector3d yJ = points[i].location;
		double y = yJ[1];
		//std::cout << y;
		a[i] = y;
		std::cout << a[i] << " our a " << std::endl;
	}

	double h[n];

	for(int i = 0; i < n; i++){
		Eigen::Vector3d next = points[i + 1].location;
		Eigen::Vector3d current = points[i].location;
		double hI = next[0] - current[0];
		std::cout << hI << " our hI " << std::endl;
		h[i] = hI;

	}

	double alpha[n];
	for(int i = 1; i < n; i++){
		alpha[i] = (3/h[i])*(a[i+1] - a[i]) - (3/h[i-1])*(a[i] - a[i-1]);

	}

	double lamda[n +1], mu[n + 1], z[n+1];
	lamda[0] = 1;
	mu[0] = z[0] = 0;
	std::cout << "our mu[0] = " << mu[0] << std::endl;
	std::cout << "our z[0] = " << z[0] << std::endl;
	for(int i = 1; i < n; i++){
			Eigen::Vector3d next = points[i+1].location;
			Eigen::Vector3d previous = points[i -1].location;
			double lamdaI = next[0] - previous[0];
			lamda[i] = 2*(lamdaI) - (h[i-1]*mu[i-1]);
			mu[i] = h[i]/lamda[i];
			z[i] = (alpha[i] - h[i-1]*z[i-1])/lamda[i];

	}

	lamda[n] = 1;
	z[n] = 0;

	double c[n+1];
	double b[n];
	double d[n];
	c[n] = 0;

	for(int j = n-1; j >= 0; j--){
		c[j] = z[j] - mu[j]*c[j+1];
		b[j] = (a[j+1] - a[j])/(h[j]) - h[j]*(c[j+1] + 2*c[j])/3;
		d[j] = (c[j+1] - c[j])/(3*h[j]);

	}

	std::vector<Eigen::Vector4d> coefficients;
	for(int i = 0; i < n  ; i++){
		Eigen::Vector4d coeffs;
		coeffs[0] = a[i];
		coeffs[1] = b[i];
		coeffs[2] = c[i];
		coeffs[3] = d[i];

		coefficients.push_back(coeffs);
	}

	drawSplines(coefficients);
	return coefficients;



}
void CubicSpline::drawSplines(std::vector<Eigen::Vector4d> splines){

	//go over each spline on the interval xi -> xi + 1
	//get the corresponding piecewise polynomial, i.e., splines[i]
	//draw points at x incremental intervals.
	std::vector<Vertex>  splinePoints;
	std::cout << "Size of the splines : " <<  splines.size() << std::endl;
	for(int i = 0; i < points.size(); i++){
		std::cout << splines[i] << std::endl;
		std::cout << "Our i : " << i << std::endl;
		double measure  = points[i+1].location[0] - points[i].location[0];
		for(double x = 0.0; x < measure; x+= 0.01){
			Vertex v;
			Eigen::Vector3d point;
			Eigen::Vector4d polynomial = splines[i];
		//	std::cout << "Our x : " << points[i].location[0] << std::endl;
			double x1 = x + points[i].location[0];
			double evaluate =(x1 - points[i].location[0]);
			double y = pow(evaluate, 3)*polynomial[3] + pow(evaluate, 2)*polynomial[2] + evaluate*polynomial[1] + polynomial[0];
		std::cout <<  "This is Y: " << y << std::endl;

			std::cout << x1 << " our x1 " << std::endl;
			point[2] = 0;
			point[0] = x1;
			point[1] = y;
			v.location = point;
			splinePoints.push_back(v);
		}

	}


	PlyFile p(splinePoints);
	p.write("SplineApproximates.ply");
}
