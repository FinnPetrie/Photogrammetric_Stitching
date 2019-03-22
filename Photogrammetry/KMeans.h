#pragma once
#include "PlyFile.h"
#include <vector>
//K-Means clustering object
class KMeans{

private:

    //the number of k-clusters to divide the point cloud into.
	int kmeans;
    //the 'old centroids', i.e., centroids from the previous iteration.
	std::vector<Vertex> oldCentroids;
    // the point cloud to segment.
	 PlyFile& toCluster;

public:

    //constructor.
	KMeans(int kmeans, PlyFile* toCluster) : kmeans(kmeans), toCluster(*toCluster){
	}

    /* Makes sure the array is distinctly valued, so that no two clusters have the same centroid.
     * parameters:
     * centroids - an array of centroids
     * size - the size of the array.
     *
     * returns: true if the centroids are distinct, false otherwise.
     */
	bool distinctValueArray(std::vector<Vertex> centroids, int size);

    /* Clustering algorithm, this is the implementation of K-Means.
     * parameters:
     * error - the error metric, K-Means will halt when the computed error is below this.
     * filename - the name to save the file to.

     */
	void clustering(double error, std::string filename);

    /* prints the data held by the object.
     */
	void print();

    /* Euclidean distance - measures the distance between vertices v and t.
     * parameters:
     * Vertices v, t - the vertices to find the distance between.
     * returns: the euclidean distance.
     */
	double EuclideanDistance(Vertex v, Vertex t);

    /* Calculates the required error metric for K-Means.
     * parameters:
     * v - in this case a centroid
     * t - a vertex belonging to the kth cluster.
     *
     * returns:
     * the error value.
     */
	double errorCalculate(Eigen::Vector3d v, Vertex t);

    /* Computes whether K-Means should halt. This is done by checking if there are updates
     * in the centroids, or if the algorithm has converged.
     *
     * parameters:
     * oldCentroids - the centroids of the previous iteration.
     * newCentroids - the centroids of the current iteration.
     *
     * returns:
     * true if the the algorithm has converged, otherwise false.
     */
	bool shouldStop(std::vector<Vertex> oldCentriods, std::vector<Vertex> newCentroids);


};
