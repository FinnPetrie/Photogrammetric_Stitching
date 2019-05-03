#include "CubicSpline.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
CubicSpline::CubicSpline(PlyFile p) : points(p){

}

CubicSpline::CubicSpline(PlyFile p, bool period) : points(p), periodic(period){

}


void CubicSpline::spline(int xAxis, std::string t){
	//find ai, and hi
	std::vector<double> a;
	std::vector<double> h;
	for(int i =0 ; i < points.size(); i++){
		//a[i] = the y value in our ith vector
		a.push_back(points[i].location[(xAxis+1)%3]);
	}

	points.print();
	for(int i =0 ; i < points.size() -1; i++){
		//h[i] = the difference between xi+1 and xi
	//	std::cout << points[i+1].location[xAxis] << std::endl;
		std::cout << " points[i].x = " << points[i].location[0];
		double xDifference = points[i+1].location[xAxis] - points[i].location[xAxis];
		h.push_back(xDifference);
		std::cout << "our h_i = " << std::endl;
		std::cout << h[i] << std::endl;
	}

	for(int i =0 ; i < points.size(); i++){
		std::cout << "x = " << points[i].location[0] << " y = " << points[i].location[1] << " z = " << points[i].location[2] << std::endl;
	}


	//need to fill matrix
	int order;
	if(periodic){
		order = points.size() - 1;
	}else{
		order = points.size() - 2;
	}
	Eigen::MatrixXf H(order, order);
	Eigen::VectorXf solutionH(order);


	//fill matrix, tridiagonal.

	/**for(int i = 0; i < points.size() -2; i++){

		for(int k = 0; k < points.size() -2; k++){
			if(i == k){
				H(i, k) = 2*(h[n] + h[n + 1]);
			}
			if(k == (i + 1)){
				H(i, k) = h[n];
			}
			if(k == (i - 1)){
				H(i, k) = h[n+ 1];
			}
		}
		n++;
	}*/

	for(int i =0 ; i < order; i++){
		std::cout << "It okay" << std::endl;
		if(i == 0){
			std::cout << "Instantiating the first H([i,0])" << std::endl;
			H(i, 0) = 2*(h[0] + h[1]);
			std::cout << "Instantiated" << std::endl;
		if(periodic){
			std::cout << "Yes periodic" << std::endl;
			//by the required matrix
			H(0, order -1) = h[0];
			H(order- 1, 0) = h[0];
			std::cout << "Got passed order assignment" << std::endl;
			}
		}else{
			H(i, i -1) = h[i];
			H(i, i) = 2*(h[i] + h[i + 1]);
			if(i < points.size() - 3){
				H(i, i+1) = h[i + 1];
			}
		}
		//still need to modify our solution vector to work with periodic functions.
		if(periodic){
			if(i != (order -1)){
				solutionH(i) = ((a[i + 2] - a[i + 1])/h[i+1] - (a[i+1] - a[i])/h[i])*3.0f;
			}else{
				solutionH(i) =((a[1] - a[0])/h[0] - (a[0] - a[i])/h[i])*3.0f;
				}
			}else{
				if((h[i] != 0.0f) && (h[i+1] != 0.0f)){
				solutionH(i) = ((a[i + 2] - a[i + 1])/h[i+1] - (a[i+1] - a[i])/h[i])*3.0f;
				}
			}
	}
	std::cout << H << std::endl;


/**	for(int i = 0; i < points.size() -2; i++){
		double solutionI = 3*(a[n + 1] -a[n])/(h[n -1]) - 3*(a[n-1] -a[n-2])/(h[n-2]);
		solutionH[i] = solutionI;
		n++;

	}*/

	std::cout << "\n\n\n\nOur solution to H: " << std::endl;
	std::cout << solutionH << std::endl;
	std::cout << "Our H : \n" << H << "\n\n\n\n" <<  std::endl;

	for(int i =0 ; i < h.size(); i++){
		std::cout << "H of " << i << " = " << h[i] << " ";

	}


	Eigen::VectorXf x = H.colPivHouseholderQr().solve(solutionH);
	std::cout << "Our  x = \n" << x << std::endl;
	std::vector<double> b;
	std::vector<double> c;
	std::vector<double> d;
	c.push_back(0);
	for(int i =0 ; i < x.size(); i++){
		//put our X vectors values into c, with c[0] = 0, and c[n] = 0;
		c.push_back(x[i]);

	}
	c.push_back(0);
	for(int i =0 ; i < c.size(); i++) {
		std::cout << c[i] << " our c" << std::endl;
	}

	for(int i =0 ; i < points.size() - 1 ; i++){
		//solve d and b in terms of c.
		double bCoefficient = ((a[i + 1] - a[i])*(1.f/h[i]) - (h[i]/3.f)*(c[i+1] + 2.0f*c[i]));
		double dCoefficient = (c[i+1] - c[i])/(3.f*h[i]);
		d.push_back(dCoefficient);
		b.push_back(bCoefficient);

	}
	for(int i =0 ; i < b.size(); i++){
	std::cout << "Our b coefficient: " << b[i] << std::endl;
	std::cout << "Our d coefficient: " << d[i] << std::endl;
	//std::cout << d[i] << std::endl;
	}
	std::vector<Eigen::Vector4d> coefficients;

	for(int i =0 ; i < points.size(); i++){
				Eigen::Vector4d coeffs;
				coeffs[0] = a[i];
				coeffs[1] = b[i];
				coeffs[2] = c[i];
				coeffs[3] = d[i];

				coefficients.push_back(coeffs);
			}
	this->splines = coefficients;
	//Interpolate("Interpolate.ply", 200, h);
	drawSplines(coefficients, xAxis, t);
}


void CubicSpline::Interpolate( std::string t, int resolution, std::vector<double> h){

	double timestamp;
	std::vector<Vertex> vertices;
	for(int i =0 ; i < points.size() -1; i++){
		Eigen::Vector4d spline = this->splines[i];
		double measure  = points[i+1].location[0] - points[i].location[0];
		for(double x = 0.0; x < measure; x+= 0.001){
			Vertex v;
			double xPrime = x + points[i].location[1];
			double yPrime = spline[0] + spline[1]*(x) + spline[2]*pow(x, 2) + spline[3]*pow(x,3);
			v.location = Eigen::Vector3d(xPrime, yPrime, 0);
			vertices.push_back(v);

		}
	}
	PlyFile p(vertices);
	p.write("Interpolate.ply");


}

std::vector<Eigen::Vector4d> CubicSpline::computeSplines(int axis, std::string t){
	int n = points.size();
	n--;
	double a[n + 1];

	for(int i =0 ; i < n + 1; i++){
		Eigen::Vector3d yJ = points[i].location;
		double y = yJ[(axis+1)%3];
		//std::cout << y;
		a[i] = y;
		std::cout << a[i] << " our a " << std::endl;
	}

	double h[n];

	for(int i = 0; i < n; i++){
		Eigen::Vector3d next = points[i + 1].location;
		Eigen::Vector3d current = points[i].location;
		double hI = next[axis] - current[axis];
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
			double lamdaI = next[axis] - previous[axis];
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

	for(int i = 0 ; i< n+1; i++){
		std::cout << "These are our cs : " << c[i] << std::endl;
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

	splines = coefficients;
	//drawSplines(coefficients, axis, t);
	return coefficients;



}


void CubicSpline::drawSplines(std::vector<Eigen::Vector4d> splines, int axis, std::string t){

	//go over each spline on the interval xi -> xi + 1
	//get the corresponding piecewise polynomial, i.e., splines[i]
	//draw points at x incremental intervals.
	std::vector<Vertex>  splinePoints;
	std::cout << "Size of the splines : " <<  splines.size()  << std::endl;
	for(int i = 0; i < points.size() -1 ; i++){
		//std::cout << splines[i] << std::endl;
		//std::cout << "Our i : " << i << std::endl;

		double measure  = points[i+1].location[axis] - points[i].location[axis];
		//std::cout << measure << std::endl;
		for(double x = 0.0; x < measure; x+= 0.001){
			Vertex v;
			Eigen::Vector3d point;
			Eigen::Vector4d polynomial = splines[i];
		//	std::cout << "Our x : " << points[i].location[0] << std::endl;
			double x1 = x + points[i].location[axis];
			double evaluate (x1 - points[i].location[axis]);
			std::cout << "X1 = !!!!! " << x1 << std::endl;
			double y = pow(evaluate, 3)*polynomial[3] + pow(evaluate, 2)*polynomial[2] + evaluate*polynomial[1] + polynomial[0];
		//std::cout <<  "This is f(x): " << y << std::endl;
			std::cout << "Y == !!!!! " << y << std::endl;
		//	std::cout << x1 << " our x " << std::endl;
			point[(axis+2)%3] = 0;
			point[axis] = x1;
			point[(axis+1)%3] = y;
			v.location = point;
			v.colour = Eigen::Vector3i(0, 255, 0);
			v.normal = Eigen::Vector3d(0,0,0);
			splinePoints.push_back(v);
		}

	}

	PlyFile p(splinePoints);
	p.write(t);
}

void CubicSpline::runLib(std::string s){
		points.orientateAroundYAxis();
		points.sortAlongAxis(1, 0, points.size());

		std::vector<Vertex> neg = points.collectNegativeVertices(2);
		PlyFile negatives(neg);
		int size = negatives.size();
		negatives.sortAlongAxis(1, 0, size);
		negatives.write("NegativesSortedY.ply");
		std::vector<double> x;
		std::vector<double> y;

		std::vector<Vertex> pos = points.collectPositiveVertices(2);
		PlyFile positives(pos);
		positives.sortAlongAxis(1, 0, pos.size());

		//attempt to interpolate between the positive and negative interpolation
		//negatives.push_back(positives[0]);
		//negatives.push_back(positives[positives.size()]);
		//negatives.sortAlongAxis(1,0, negatives.size());

		std::vector<double> xPos;
		std::vector<double> yPos;

		for(int i =0 ; i < positives.size(); i++){
			xPos.push_back(positives[i].location[1]);
			yPos.push_back(positives[i].location[2]);
		}

		for(int i = 0; i < negatives.size(); i++){
		//std::cout << negatives[i].location(1) << std::endl;
		//std::cout << negatives[i].location(2) << std::endl;
		x.push_back(negatives[i].location[1]);
		y.push_back(negatives[i].location[2]);
		}

		tk::spline sNeg;
		sNeg.set_points(x, y);
		std::vector<Vertex> vertices;
		/**
		for(int i =0 ;i < size-1; i++){
			//std::cout <<  "In here" << std::endl;
			//std::cout << negatives.size();
			std::cout << "Next value in x vector : " << x[i+1] << std::endl;
			std::cout << "Current value in x vector : " <<  x[i] << std::endl;
			double difference = abs(x[i+1] - x[i]);

			for(double x1 = 0.0; x1 < difference; x1+= 0.001){
				Vertex v;
					Eigen::Vector3d point;
					std::cout << "Our x increment : " << x1 << std::endl;
					std::cout << "Our spline interpolation : " << s(x1) << std::endl;

					point[0] = 0;
					point[1] = x1 + x[i];
					point[2] = s(x1 + x[i]);
					v.location = point;

					vertices.push_back(v);


				}
				std:: cout << "Difference between next and current value " << difference  << std::endl;

			}
			*/
		//first point
				// draw a set number of points between
				//next point.

			tk::spline sPos;
			sPos.set_points(xPos, yPos);

			for(double x1 = x[0]; x1 < abs(x[negatives.size()] - x[0]); x1 += 0.001){
				std::cout << x1 << std::endl;

				Vertex v;
				Eigen::Vector3d point;
				point[0] = 0;
				point[1] = x1;
				point[2] = sNeg(x1);

				v.location = point;

				vertices.push_back(v);
			}

			//for each interval


			PlyFile negInterp(vertices);
			negInterp.write(s +"NegInterp.ply");


			std::vector<Vertex> posVertices;


			/**
			for(int i = 0; i < positives.size(); i++){
					double nextY = y[i+1];

					do{
						double x1 = x[i];
						double y = sNeg(x1);
						Vertex v;
						Eigen::Vector3d point;
						point[0] = 0;
						point[1] = x1;
						point[2] = y;
						v.location = point;
						posVertices.push_back(v);
						x1 += 0.001;
					}while(y != nextY);
				}*/


			for(double x1Pos = xPos[0]; x1Pos < abs(xPos[positives.size()] - xPos[0]); x1Pos += 0.001){
				Vertex v;
				Eigen::Vector3d point;
				point[0] = 0;
				point[1] = x1Pos;
				point[2] = sPos(x1Pos);

				v.location = point;
				posVertices.push_back(v);
			}

			PlyFile posInterp(posVertices);
			posInterp.write(s + "PosInterp.ply");

}


double CubicSpline::evaluateAt(double x, int axis){
	double low = points[0].location[axis];
	double high = points[points.size()].location[axis];
	if(x >= low && x <= high){
		for(int i =0 ; i < splines.size(); i++){
			if(x >= points[i].location[axis] && x <= points[i].location[axis]){
				Eigen::Vector4d polynomial = splines[i];
				//get point w.r.t the last point.
				double x1 = x - points[i].location[axis];
				//evaluate
				double y = pow(x1, 3)*polynomial[3] + pow(x1, 2)*polynomial[2] + x1*polynomial[1] + polynomial[0];

				return y;
			}
		}
	}

	return 0;

	//find the interval of f.
}
void CubicSpline::approximateHull(){
	//reorientate Hull in terms of its greater principle component.
	points.orientateAroundYAxis();
	//divide the hull in two - those elements that lie above the axis, and those below.
	//here seperate the points into two PlyFiles.
	std::vector<Vertex> positives = points.collectPositiveVertices(2);
	std::vector<Vertex> negatives = points.collectNegativeVertices(2);

	PlyFile pos(positives);
	//pos.rotateAxis(0, 1);
	PlyFile neg(negatives);

	//neg.sortAlongAxis(1, 0, neg.size());

	neg.write("Negative.ply");
	points = pos;
	//move negative components to the top, i.e., rotation by 180,
	spline(1, "PositiveSpline.ply");
	Eigen::Vector3d cent = pos.centroid();
	pos.rotateAxisAboutPoint(0, 1, cent);
	pos.write("Positive.pmly");
	points = neg;
	spline(1, "NegativeSpline.ply");

	PlyFile posSpline("PositiveSpline.ply");
	posSpline.rotateAxisAboutPoint(0,1, cent);
	posSpline.write("PositiveSpline.ply");
}


void CubicSpline::drawSplinesYZ(std::vector<Eigen::Vector4d> splines, std::string t){

	//go over each spline on the interval xi -> xi + 1
	//get the corresponding piecewise polynomial, i.e., splines[i]
	//draw points at x incremental intervals.
	std::vector<Vertex>  splinePoints;
	std::cout << "Size of the splines : " <<  splines.size() << std::endl;
	for(int i = 0; i < points.size() -1; i++){
		std::cout << splines[i] << std::endl;
		std::cout << "Our i : " << i << std::endl;
		double measure  = points[i+1].location[1] - points[i].location[1];

		for(double y = points[i].location[1]; y < points[i+1].location[1]; y+= 0.001){
			Vertex v;
			Eigen::Vector3d point;
			Eigen::Vector4d polynomial = splines[i];
		//	std::cout << "Our x : " << points[i].location[0] << std::endl;
			//double y1 = y + (points[i].location[1]);
			//double evaluate =(y1 - points[i].location[1]);
			double evaluate = y;

			double z = pow(evaluate, 3)*polynomial[3] + pow(evaluate, 2)*polynomial[2] + evaluate*polynomial[1] + polynomial[0];
		std::cout <<  "This is Y: " << y << std::endl;

			std::cout << y << " our x1 " << std::endl;
			point[0] = 0;
			point[1] = y;
			point[2] = z;
			v.location = point;
			v.colour = Eigen::Vector3i(0, 255, 0);
			splinePoints.push_back(v);
		}

	}
	PlyFile p(splinePoints);
	p.write(t);
}
