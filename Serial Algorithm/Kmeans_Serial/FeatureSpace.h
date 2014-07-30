/*
 * FeatureSpace.h
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


#include "FeaturePoint.h"
#include "jbutil.h"
#include <limits>
#include <ctime>
#include <istream>
#include <cfloat>

#define LIMIT_U 100
#define LIMIT_L -100


// Contains the point cloud for all centroids and data points. Contains the cluster functions for converging centroids
class FeatureSpace
{
	public:
			// Parameter constructor takes in number of centroid
			FeatureSpace( int K );
			virtual ~FeatureSpace();

			// Uses loop to converge centroid to local minimum
			// Returns number of iterations required to converge all centroids
			int ClusterSearch();
			// Get Current Data Points with Cluster Information
			std::vector<FeaturePoint> getDataPoints();
			FeaturePoint* getCentroids();

			// Store data from file from X Y row format
			friend std::istream& operator >>(std::istream& in, FeatureSpace& cloud);

			// Debug
			void printData();
			void printCentroids();

	private:
			// Stop implicit default constructor
			FeatureSpace();

			// Fits subset to nearest centroid based on Euclidean distance
			inline void SnapSubsets();
			// Moves centroid to mean position of cluster
			inline void UpdateCentroids();

			// Compute squared euclidean distance
			inline float Euclidean( Point a, const Point& b );

			std::vector<FeaturePoint> PointCloud;
			FeaturePoint* Centroids;

			int DataSize;
			int CentroidSize;
};


// -------------------------- Inline Member Functions -----------------------


// Return squared euclidean distance betwen points
float FeatureSpace::Euclidean( Point a, const Point& b )
{
		// Find square difference between points
		a -= b;
		a *= a;

		return (a.contents().first + a.contents().second );
};


// Find nearest Centroid to data point. Store centroid's index against data point
// Static Geometry members slice base class of FeaturePoint for processing
void FeatureSpace::SnapSubsets()
{
	// Store previous centroid distance to point
	float distance, newDistance; 

	// For each point
	for( int i = 0; i < DataSize; i++)
	{
		// initialize distance as maximum value of double
		distance = DBL_MAX;

		for( int j = 0; j < CentroidSize; j++)
		{
			newDistance = Euclidean(Centroids[j],PointCloud[i]);

			// If new distance found < previous distance, assign data point to cluster index j
			if( distance > newDistance )
			{
				distance = newDistance;
				PointCloud[i].Cluster() = j;
			}
		}
	}
};


// Converge centroid to mean of subsets
void FeatureSpace::UpdateCentroids()
{
	// IL = 3/2 arithmetic faster than doing whole mean. increase = (N + K)/N
	Point* meanPos = new Point[CentroidSize];

    // Create array for sum of cluster points aligned with centroid's index associated with cluster
	for( int i = 0; i < DataSize; i++)
	{
			// Append to Cluster's sum of contained points
			meanPos[ PointCloud[i].Cluster() ] += (Point&)PointCloud[i];
			// Increment number of points in centroid allocated to the current cluster
			Centroids[ PointCloud[i].Cluster() ].Cluster() ++;
	}

	for(int i = 0; i < CentroidSize; i++)
	{
		// mean = divide sum of data points in cluster by occurrance
		// Converge centroid to mean
		if( (double)Centroids[i].Cluster() )
		{
			meanPos[i] /= (double)Centroids[i].Cluster();
			(Point&)Centroids[i] = meanPos[i];
		}

		Centroids[i].Cluster() = 0;
	}
};

