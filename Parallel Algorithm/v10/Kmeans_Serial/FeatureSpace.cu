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




// Find nearest Centroid to data point. Store centroid's index against data point
// Static Geometry members slice base class of FeaturePoint for processing
//__global__ void SnapSubsets(double* ptr_centroids, double* ptr_data, double* ptr_cluster, long int DataSize, short int CentroidSize)
__global__ void SnapSubsets(double* ptr_centroids, double* ptr_data, double* ptr_cluster, int DataSize, short int CentroidSize)
{
	// Store previous centroid distance to point
	double distance, newDistance;
	long int id = ( threadIdx.x + (blockDim.x * blockIdx.x) );
	
	// initialize distance as maximum value of double
	distance = DBL_MAX;

	// Protect against Threads accessing inexistent point
	if( id < DataSize)
	{
		for( int j = 0; j < CentroidSize; j++)
		{
			//a = (ptr_centroids[j*2] - ptr_data[(id*2)] );
			//b = (ptr_centroids[(j*2) + 1] - ptr_data[(id*2) + 1] );

			//newDistance = a*a + b*b;
			newDistance = ((ptr_centroids[j*2] - ptr_data[(id*2)] )*(ptr_centroids[j*2] - ptr_data[(id*2)] )) + ((ptr_centroids[(j*2) + 1] - ptr_data[(id*2) + 1] )*(ptr_centroids[(j*2) + 1] - ptr_data[(id*2) + 1] ));

			// If new distance found < previous distance, assign data point to cluster index j
			if( distance > newDistance )
			{
				distance = newDistance;
				ptr_cluster[id] = j;
			}	
		}
	}

	//__syncthreads();
};


// Partition feature space into clusters and recursively converge centroids until local minimum achieved
int FeatureSpace::ClusterSearch()
{
	
	double currEuclidean = 0, prevEuclidean;
	int iterations = 0;

	double* device_ptr_centroids_xy, *device_ptr_data_xy, *host_ptr_centroids_xy, *host_ptr_data_xy, *host_ptr_point_cluster, *device_ptr_point_cluster;
	
	// Cannot use member functions on GPU. Rearrange Centroid/Feature data into 1D float arrays
	host_ptr_data_xy = new double[DataSize*2];
	host_ptr_point_cluster = new double[DataSize];
	host_ptr_centroids_xy = new double[CentroidSize*2];

	for(int i=0; i< DataSize; i++)
	{
		host_ptr_data_xy[2*i] = PointCloud[i].contents().first;
		host_ptr_data_xy[(2*i) + 1] = PointCloud[i].contents().second;
		host_ptr_point_cluster[i] = 0;
	}


	// Prepare GPU global memory for Feature Space.
	Success( cudaMalloc( &device_ptr_centroids_xy, sizeof(double) * 2 * CentroidSize ) );
	Success( cudaMalloc( &device_ptr_data_xy, sizeof(double) * 2 * DataSize ) );
	Success( cudaMalloc( &device_ptr_point_cluster, sizeof(double) * DataSize ) );

	// Feature Point [X Y] is constant. Copy Once to GPU once. Not used in constant memory as centroid [X Y] addresses are read more than any feature point address
	Success( cudaMemcpy(device_ptr_data_xy, host_ptr_data_xy, sizeof(double) * 2 * DataSize,  cudaMemcpyHostToDevice) );
	// Copy the cluster elements per point once.
	Success( cudaMemcpy(device_ptr_point_cluster, host_ptr_point_cluster, sizeof(double) * DataSize, cudaMemcpyHostToDevice) );

	// Stores point data on constant memory. Requires 32 bytes per thread: 32*1024 = 32768 bytes
	//cudaFuncSetCacheConfig(SnapSubsets, 2);

	// while points have not converged
	do
	{
		iterations++;

		// store cluster function output
		prevEuclidean = currEuclidean;
		currEuclidean = 0;

		// ------------------------------------ Parallel SnapSubsets -----------------------------------------

		// Format updated centroid information and copy to GPU
		for(int i = 0; i < CentroidSize; i++)
		{
			host_ptr_centroids_xy[2*i] = Centroids[i].contents().first;
			host_ptr_centroids_xy[(2*i) + 1] = Centroids[i].contents().second;
		};

		// Load new centroid positions to GPU
		Success( cudaMemcpy(device_ptr_centroids_xy, host_ptr_centroids_xy, sizeof(double) * 2 * CentroidSize, cudaMemcpyHostToDevice) );

		// Only 52 bytes needed of register space for this function. No shared memory access
		// Launch maximum number of threads
		SnapSubsets <<< (DataSize/THREADS_PER_BLOCK), THREADS_PER_BLOCK>>>(device_ptr_centroids_xy, device_ptr_data_xy, device_ptr_point_cluster, DataSize, CentroidSize);

		//cudaDeviceSynchronize();
		cudaError_t err = cudaGetLastError();

		if (err != cudaSuccess) 
		  printf("Error: %s\n", cudaGetErrorString(err));

		// Extract Cluster each point assigned. Allocated on GPU with centroid distance measurements (1 way transfer)
		Success( cudaMemcpy(host_ptr_point_cluster, device_ptr_point_cluster, sizeof(double) * DataSize, cudaMemcpyDeviceToHost) );

		for(int i=0; i<DataSize; i++)
			PointCloud[i].Cluster() = host_ptr_point_cluster[i];

		// -------------------------------Continue As Per Usual--------------------------------------------------------

		// Converge centroid to cluster mean
		UpdateCentroids();

		// Find total sum of euclidean distances
		for( int i = 0; i < DataSize; i++)
			currEuclidean += Euclidean(Centroids[ PointCloud[i].Cluster() ],PointCloud[i] );

	}
	while( currEuclidean != prevEuclidean );


	// Clear GPU memory. Centroid and Point data is unchanged.
	cudaFree(device_ptr_centroids_xy);
	cudaFree(device_ptr_point_cluster);
	cudaFree(device_ptr_data_xy);

	return iterations;
};


void FeatureSpace::Success(cudaError_t err) const
{
	if(err != cudaSuccess)
		std::cout << "Error: " << err << std::endl;
}



