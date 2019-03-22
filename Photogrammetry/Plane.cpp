#include "Plane.h"




Plane::Plane(){
	this->planeNormal = Eigen::Vector3d(0,1,0);
	this->point = Eigen::Vector3d(1,0,0);
}
Plane::Plane(PlyFile plane) : planePly(plane){
	this->planeNormal = computeNormal();
	std::cout << planeNormal;
}



Plane::Plane(std::string pathToNormal, std::string pathToPly){
 this->read(pathToNormal);
 PlyFile plane(pathToPly);
 this->planePly = plane;
}
Eigen::Vector3d Plane::computeNormal(){
	Eigen::Vector3d point0 = planePly[3].location - planePly[0].location;
	Eigen::Vector3d point1 = planePly[1].location - planePly[0].location;
	Eigen::Vector3d pNormal = point0.cross(point1);
	return pNormal;

}


bool Plane::read(std::string path){
	std::ifstream plane;
	//plane.ignore('[', )
	plane.open(path);
	double x;
	if(!plane){
		std::cout << "Unable to open plane file. Check that the file exists/that path is correct" << std::endl;
		exit(1);
	}

	std::string line;
	std::vector<std::string> values;
	while(std::getline(plane, line, '\n')){
		std::istringstream input(line);
		std::string v;

		while(std::getline(input, line, ' ')){
			values.push_back(line);
			//std::cout << line;
		}
	}

	for(int i =0 ; i < values.size(); i++){
		std::cout << "\nValue @ : " << i << " = " << values[i] << std::endl;
	}

	std::cout << line;
	Eigen::Vector3d normal;
	Eigen::Vector3d point;
	for(int i = 0; i < 3; i++){
		std::string point1 = values[i];
		std::string point2 = values[3 + i];
		double normalComponent = std::atof(point2.c_str());
		double pointComponent = std::atof(point1.c_str());
		normal[i] = normalComponent;
		point[i] = pointComponent;

	}

	this->point = point;
	this->planeNormal = normal;
	std::cout << "Our normal: " << std::endl;
	std::cout << normal << std::endl;
	std::cout << "Our point: " << std::endl;
	std::cout << point << std::endl;
	std::cout << "\n\nPoint printed\n\n";
	return true;
}

Eigen::Vector3d Plane::projectToPlane(Eigen::Vector3d toProject){
	double normA = planeNormal.norm();
	double scalarProj = planeNormal.dot(toProject)/normA;

	return scalarProj*planeNormal/normA;

}



void Plane::updateLocation(Eigen::Vector3d newLocation, int index){
	planePly.updateLocation(newLocation, index);
}
void Plane::setPlanePly(PlyFile p){
	this->planePly = p;
}

int Plane::size(){
	return planePly.size();
}
void Plane::reColour(int r, int g, int b){
	planePly.reColour(r, g, b);
}

void Plane::rotateAxis(int axis, double amount){
	planePly.rotateAxis(axis, amount);
}

void Plane::translatePlane(Eigen::Vector3d trans){
	planeNormal += trans;
	point += trans;
}


void Plane::translateCloud(Eigen::Vector3d trans){
	translatePlane(trans);
	planePly.translateCloud(trans);
}
void Plane::rotatePlane(Eigen::Matrix3d rotation){
	Eigen::Vector3d rotNormal = rotation*planeNormal;
	Eigen::Vector3d rotPoint = rotation*point;
	point = rotPoint;
	planeNormal = rotNormal;
	//planePly.rotateCloud(rotation);
}


PlyFile Plane::getPlanePly(){
	return planePly;
}

void Plane::rotateAboutPoint(Eigen::Matrix3d rotation, Eigen::Vector3d point){
	planeNormal -= point;
	this->point -= point;
	Eigen::Vector3d newNormal = rotation * planeNormal;
	Eigen::Vector3d newPoint = rotation * point;
	newNormal -= -(point);
	newPoint -= -(point);
	this->point = newPoint;
	planeNormal = newNormal;

	planePly.rotateAboutPoint(rotation, point);

}

bool Plane::write(std::string filename){

	planePly.write(filename);

	std::ofstream fout(filename + ".plane");
		    if (!fout) {
		        std::cerr << "Failed to open PLY file " << filename << " for writing" << std::endl;
		        return false;
		    }
	fout << point[0] << " " << point[1] << " " << point[2] << "\n";
	fout << planeNormal[0] << " " << planeNormal[1] << " " << planeNormal[2] << "\n";

	fout.close();


		    return true;

}

void Plane::rotateOrigin(Eigen::Matrix3d rotation){
	Eigen::Vector3d cent = planePly.centroid();
	planeNormal -= cent;
	point -= cent;
	Eigen::Vector3d newNormal = rotation * planeNormal;
	Eigen::Vector3d newPoint = rotation * point;
	newNormal -= -(cent);
	newPoint -= -(cent);
	point = newPoint;
	planeNormal = newNormal;
	planePly.rotateOrigin(rotation);
}
