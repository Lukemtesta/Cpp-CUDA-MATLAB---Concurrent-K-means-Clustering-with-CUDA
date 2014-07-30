/*
 * FeatureSpace.cpp
 *
 */

/*

Copyright (c) 2014 Luke Marcus Biagio Testa
All rights reserved.

Redistribution and use in source and binary forms are permitted
provided that the above copyright notice and this paragraph are
duplicated in all such forms and that any documentation,
advertising materials, and other materials related to such
distribution and use acknowledge that the software was developed
by the Luke Marcus Biagio Testa. The name of the
Luke Marcus Biagio Testa may not be used to endorse or promote products derived
from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 */

#include "FeatureSpace.h"



// ------------------ Constructors ----------------

// Parameter constructor takes in number of centroid
FeatureSpace::FeatureSpace( const int K ) : CentroidSize(K), DataSize(0)
{
	jbutil::randgen Rand( std::time(0) );
	Centroids = new FeaturePoint[CentroidSize];

	// Initialize Centroids at random positions and number of points per cluster to 0
	for(int i = 0; i < CentroidSize; i++)
	{
		// Centroid Subset member is the number of points currently allocated to the cluster the centroid represents
		//FeaturePoint v( Rand.fval(LIMIT_L, LIMIT_U), Rand.fval(LIMIT_L, LIMIT_U), 0 );
		FeaturePoint v( i, i, 0 );
		Centroids[i] = v;
	}


};


// ------------------ Operator Overloads -------------------


// Store data from file in X Y row format into point cloud
std::istream& operator >>(std::istream& in, FeatureSpace& cloud)
{
	cloud.DataSize++;

	// Create temporary point and initialize with 2D data from file
	FeaturePoint temp;
	in >> temp;

	// Copy 2D data into point cloud. Initialize point's cluster index to 0.
	cloud.PointCloud.push_back(temp);

	return in;
}



// ------------------ Destructors -----------------

FeatureSpace::~FeatureSpace()
{
	//std::cout << "[DELETING] FeatureSpace" << std::endl;
	delete [] Centroids;
}


// ----------------- Member Functions --------------

// Print Data Point Cloud to Console Output
void FeatureSpace::printData()
{
	std::cout << "***** Printing Feature Space ******" << std::endl;
	std::cout << "DataSize: " << DataSize << std::endl;

	for(int i = 0; i < DataSize; i++)
		PointCloud[i].print();
}

// Print Centroids to Console Output
void FeatureSpace::printCentroids()
{
	std::cout << "***** Printing Centroids *****" << std::endl;
	std::cout << "Number of Centroids: " << CentroidSize << std::endl;

	for(int i = 0; i < CentroidSize; i++)
		Centroids[i].print();
}


FeaturePoint* FeatureSpace::getCentroids()
{
	return Centroids;
}

// Get current Data points with cluster information
std::vector<FeaturePoint> FeatureSpace::getDataPoints()
{
	return PointCloud;
}



// Partition feature space into clusters and recursively converge centroids until local minimum achieved
int FeatureSpace::ClusterSearch()
{
	float currEuclidean = 0, prevEuclidean;
	int iterations = 0;

	// while points have not converged
	do
	{
		iterations++;

		// store cluster function output
		prevEuclidean = currEuclidean;
		currEuclidean = 0;
				
		// Fit points to centroid
		SnapSubsets();

		// Converge centroid to cluster mean
		UpdateCentroids();

		// Find total sum of euclidean distances
		for( int i = 0; i < DataSize; i++)
			currEuclidean += Euclidean(Centroids[ PointCloud[i].Cluster() ],PointCloud[i] );
	}
	while( currEuclidean != prevEuclidean );

	return iterations;
};

