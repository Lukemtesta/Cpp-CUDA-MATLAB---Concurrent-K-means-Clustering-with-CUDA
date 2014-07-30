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


// Constant Memory: Max 4000 2D clusters
const int N = 4000;
__constant__ double device_ptr_centroids_xy[N];


// Find nearest Centroid to data point. Store centroid's index against data point
// Static Geometry members slice base class of FeaturePoint for processing
__global__ void SnapSubsets(double* ptr_data, double* ptr_cluster, const long int DataSize, const short int CentroidSize)
{
	// Store previous centroid distance to point
	double distance, newDistance;
	long int id = ( threadIdx.x + (blockDim.x * blockIdx.x) );

	// declare shared memory. Maximum is 1024 2D double points.
	__shared__ double data_xy[2*THREADS_PER_BLOCK];
	__shared__ double data_centroid[THREADS_PER_BLOCK];

	__syncthreads();

	// Copy Point data from global to shared.
	data_xy[threadIdx.x*2] = ptr_data[id*2];
	data_xy[threadIdx.x*2 + 1] = ptr_data[id*2 + 1];

	// initialize distance as maximum value of double
	distance = DBL_MAX;

	// Protect against Threads accessing inexistent point
	if( id < DataSize)
	{
		for( int j = 0; j < CentroidSize; j++)
		{
			newDistance = ((device_ptr_centroids_xy[ j*2  ] - data_xy[threadIdx.x*2] )*(device_ptr_centroids_xy[ j*2 ] - data_xy[threadIdx.x*2] )) + ((device_ptr_centroids_xy[ j*2 + 1] - data_xy[threadIdx.x*2 + 1] )*(device_ptr_centroids_xy[ j*2  + 1] - data_xy[threadIdx.x*2 + 1] ));

			// If new distance found < previous distance, assign data point to cluster index j
			if( distance > newDistance )
			{
				distance = newDistance;
				data_centroid[threadIdx.x] = j;
			}	
		}
	}

	// copy from shared to global memory
	ptr_data[id*2] = data_xy[threadIdx.x*2];
	ptr_data[id*2 + 1] = data_xy[threadIdx.x*2 + 1];
	ptr_cluster[id] = data_centroid[threadIdx.x];
};


// Partition feature space into clusters and recursively converge centroids until local minimum achieved
int FeatureSpace::ClusterSearch()
{
	
	int iterations = 0;
	double *device_ptr_data_xy, *host_ptr_point_cluster = NULL, *host_ptr_data_xy = NULL, *device_ptr_point_cluster, *host_ptr_centroids_xy = NULL, currEuclidean = 0, prevEuclidean;

	// Cannot use member functions on GPU. Rearrange Centroid/Feature data into 1D float arrays
	Success( cudaMallocHost((void**)&host_ptr_data_xy, sizeof(double)*DataSize*2 ) );
	Success( cudaMallocHost((void**)&host_ptr_point_cluster, sizeof(double)*DataSize ) );
	Success( cudaMallocHost((void**)&host_ptr_centroids_xy, sizeof(double)*CentroidSize*2 ) );

	for(int i=0; i< DataSize; i++)
	{
		host_ptr_data_xy[2*i] = PointCloud[i].contents().first;
		host_ptr_data_xy[(2*i) + 1] = PointCloud[i].contents().second;
		host_ptr_point_cluster[i] = 0;
	}

	// Feature Point [X Y] is constant. Copy Once to GPU once. Not used in constant memory as centroid [X Y] addresses are read more than any feature point address
	Success( cudaMalloc( &device_ptr_data_xy, sizeof(double) * 2 * DataSize ) );
	Success( cudaMalloc( &device_ptr_point_cluster, sizeof(double) * DataSize ) );

	Success( cudaMemcpy(device_ptr_data_xy, host_ptr_data_xy, sizeof(double) * 2 * DataSize,  cudaMemcpyHostToDevice) );
	Success( cudaMemcpy(device_ptr_point_cluster, host_ptr_point_cluster, sizeof(double) * DataSize, cudaMemcpyHostToDevice) );

	cudaFuncSetCacheConfig(SnapSubsets, cudaFuncCachePreferL1);

	// while points have not converged
	do
	{
		iterations++;

		prevEuclidean = currEuclidean;
		currEuclidean = 0;

		// ------------------------------------ Parallel SnapSubsets -----------------------------------------

		// Format updated centroid information and copy to GPU
		for(int i = 0; i < CentroidSize; i++)
		{
			host_ptr_centroids_xy[2*i] = Centroids[i].contents().first;
			host_ptr_centroids_xy[(2*i) + 1] = Centroids[i].contents().second;
		};

		// Load new centroid positions to constant memory
		// Success( cudaMemcpy(device_ptr_centroids_xy, host_ptr_centroids_xy, sizeof(double) * 2 * CentroidSize, cudaMemcpyHostToDevice) );
		Success( cudaMemcpyToSymbol(device_ptr_centroids_xy, host_ptr_centroids_xy, sizeof(double)*CentroidSize*2) );

		// Only 52 bytes needed of register space for this function. No shared memory access
		// Launch maximum number of threads
		SnapSubsets <<< (DataSize/THREADS_PER_BLOCK), THREADS_PER_BLOCK>>>(device_ptr_data_xy, device_ptr_point_cluster, DataSize, CentroidSize);

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

	cudaFreeHost((void*)host_ptr_data_xy); 	
	cudaFreeHost((void*)host_ptr_point_cluster);	

	return iterations;
};


void FeatureSpace::Success(cudaError_t err) const
{
	if(err != cudaSuccess)
		std::cout << "Error: " << err << std::endl;
}



