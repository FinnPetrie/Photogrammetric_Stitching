#pragma once

#include "rply.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <cstdlib>


	struct Vertex{
		Eigen::Vector3d location;
		Eigen::Vector3i colour;
		Eigen::Vector3d normal;

		void print(){
			std::cout << location << std::endl;
			std::cout << colour << std::endl;
			std::cout << normal << std::endl;
		}

		const bool operator ==(Vertex v){
			if((this->location == v.location) && (this->colour == v.colour) && (this->normal == v.normal)){
				return true;
			}
			return false;
		}
	};

class PlyFile{

private:

		std::vector<Vertex> points_;


public:

		PlyFile(std::vector<Vertex> init);
		PlyFile(std::string pathToPly);
		PlyFile();

		bool read(const std::string& filename);

		bool write(const std::string& filename);

		bool writeBlue(const std::string& filename);

		bool writeRed(const std::string& filename);

		std::vector<Vertex> getPoints();

		Vertex getPointAt(int i);

		Vertex getRandomPoint();

		void print();

		int size();

		void reColour(int r, int g, int b);

		PlyFile colourThreshold(Eigen::Vector3i colour, double threshold, std::string filename);

		void clear();

		Eigen::Matrix3d changeOfBasis(Eigen::Matrix3d toBasis, Eigen::Matrix3d fromBasis);

		void representUnderChangeBasis(Eigen::Matrix3d toBasis, Eigen::Matrix3d fromBasis, Eigen::Vector3d centroid);

		void updateLocation(Eigen::Vector3d update, int index);

		void translateCloud(Eigen::Vector3d translation);

		void rotateCloud(Eigen::Matrix3d rotation);

		Eigen::Vector3d centroid();

		void translateToOrigin(Eigen::Vector3d centr);

		void rotateOrigin(Eigen::Matrix3d rotation);

		Eigen::Matrix3d covariance();

		void augment(PlyFile toAugment);

		void rotateAboutPoint(Eigen::Matrix3d rotation, Eigen::Vector3d point);

		void rotateAxis(int axis, double amount);

		const Vertex& operator[](size_t i){
			return points_[i];
		}

		 PlyFile operator+(PlyFile p){
			std::vector<Vertex> newPly;
//
			//std::cout << " IN here";
			for(int i =0 ; i < size(); i++){
			//	std::cout <<"first domain" << std::endl;
				newPly.push_back(points_[i]);
			}
		//	std::cout << "next domain" << std::endl;
			for(int i = 0; i < p.size(); i++){
				//std::cout << "second domain" << std::endl;
				newPly.push_back(p[i]);
			}

			std::cout << "init augment" << std::endl;
			PlyFile augPly(newPly);

			return augPly;
		}

		const Vertex& operator[](size_t i) const {
			return points_[i];
		}

		void push_back(Vertex vertex){
			points_.push_back(vertex);
		}


};


