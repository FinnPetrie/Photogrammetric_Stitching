/*
 * KMeans.cpp
 *
 *  Created on: 14/01/2019
 *      Author: finn
 */
#include "KMeans.h"
#include "PlyFile.h"
#include <array>
#include <memory>


/* Function borrowed from Hamza. Essentially makes sure that there are no centroids that are the
 * same within the array.
 *
 * parameters:
 * centroids - a vector of centroids.
 * size - the size of the centroid vector.
 * returns - true if the array is distinct, false otherwise.
 */
bool KMeans::distinctValueArray(std::vector<Vertex> centroids, int size){

    for (int i = 0; i < size; i++){
        for (int k = i + 1; k < size; k++){
            if (centroids[i] == centroids[k]){
                return false;
            }
        }
    }
    return true;
}


/* An implementation of the K-Means clustering algorithm, this segments a pointcloud into k
 * seperate point clouds.
 *  parameters:
 * errorLimit - the error threshold, when the error goes below this, the algorithm will halt.
 * filename - the filename to save the clusters under.
 *
 */
void KMeans::clustering(double errorLimit, std::string filename){

	double error =  std::numeric_limits<double>::infinity();
	std::vector<Vertex> centroids(kmeans);
	double smallestEuclideanDistance;

	PlyFile* clustered = new PlyFile[kmeans];

	//randomly initialise k centroids.
	while(!distinctValueArray(centroids, kmeans)){
	for(int k = 0; k < kmeans; k++){
		std::cout<< "clustering" << std::endl;
		centroids[k] = toCluster.getRandomPoint();
		//centroids[k].print();
		}
	}

	std::cout << "About to enter while loop\n";


	do{

		for(int i = 0; i < kmeans; i++){
			clustered[i].clear();
							//std::cout << "Clearing index : " << i << std::endl;
		}

			std::cout << "Our toCluster size: " << toCluster.size();
		for(int i = 0 ; i < toCluster.size(); i++){
			Vertex currentPoint = toCluster.getPointAt(i);
			smallestEuclideanDistance = std::numeric_limits<double>::infinity();
			int smallestIndex = -1;

			for(int k = 0; k < kmeans; k++){
					Vertex currentCentroid = centroids[k];
				if(EuclideanDistance(toCluster.getPointAt(i), currentCentroid) < smallestEuclideanDistance){

					smallestEuclideanDistance = EuclideanDistance(currentPoint, currentCentroid);
					//std::cout << "Smallest distance: " << smallestEuclideanDistance << std::endl;
					smallestIndex =  k;
				}

			}

			clustered[smallestIndex].push_back(currentPoint);
		}

		//calculate new centroids;
		error = 0;
		std::vector<Vertex> newCentroids;
		for(int k = 0; k < kmeans; k++ ){
			std::cout<< "In new centroids\n";
			Vertex newCentroid;
			newCentroid.location = Eigen::Vector3d(0,0,0);
					std::cout << "our cluster: " << k <<"'s size " << clustered[k].size() << std::endl;
				for(size_t i = 0; i < clustered[k].size(); i++){
					newCentroid.location += clustered[k][i].location;
					//average the location by the number of elements in the cluster.


					//calculate error = distance sum(0->size) of the sqrt(distance);


				}

				newCentroid.location /= clustered[k].size();

				error += EuclideanDistance(newCentroid, centroids[k]);
								std::cout << "our Error = " << error << "\n";

				newCentroids.push_back(newCentroid);
		}
		this->oldCentroids = centroids;
		centroids = newCentroids;
	}while(error > errorLimit);

	std::cout << "Writing files\n";
	for(int i = 0; i < kmeans; i++){
		clustered[i].write(filename + std::to_string(i) + ".ply");
	}
	return;

}




/* Tests whether the K-Means algorithm should stop. i.e., when the centroids are not being
 * updated.
 * parameters:
 * oldCentroids - the centroids of the previous iteration.
 * newCentroids - the centroids of the current iteration.
 *
 * returns:
 * true if there was no change, otherwise false.
 */
bool KMeans::shouldStop(std::vector<Vertex> oldCentroids, std::vector<Vertex> newCentroids){
	assert(oldCentroids.size() == newCentroids.size());
		//or, test the distance from the new centroid to the old one, and if it hasn't changed enough, end.
			//for all centroids,
				// if norm(oldCentroid[i] - newCentroid[i] < thershold){
					// return true;

			//return false;
	for(size_t i = 0; i < oldCentroids.size(); i++){
		if(oldCentroids[i].location != newCentroids[i].location){
			std::cout << "Centroids do not match\n";
			return false;
		}
	}
	return true;
}


/* Computes the euclidean distance between two vertices.
 * parameters:
 * v - first vertex.
 * t - second vertex.
 *
 * returns:
 * a measurement of the euclidean distance between v and t.
 */
double KMeans::EuclideanDistance(Vertex v, Vertex t){
	/**return sqrt(pow(v.location.x() - t.location.x()), 2) + pow((v.location.y() - t.location.y()),2) + pow(v.location.z() - t.location.z(), 2))
	*/
	Eigen::Vector3d p = v.location - t.location;
	return p.norm();
}

/* Error metric, uses the euclidean distance between all points in a segmented point cloud
 * and their centroid.
 * parameters - read EuclideanDistance.
 *
 *
 */
double KMeans::errorCalculate(Eigen::Vector3d v , Vertex t){
	Eigen::Vector3d p = v - t.location;
	return p.norm();
}

/* Printing function for showing the centroids.
 *
 *
 */
void KMeans::print(){

	std::cout << "Printing centroids" << std::endl;

	for(size_t i = 0; i < oldCentroids.size(); i++){
		oldCentroids[i].print();
	}

	std::cout << "Printing data" << std::endl;

	//toCluster.print();
}
