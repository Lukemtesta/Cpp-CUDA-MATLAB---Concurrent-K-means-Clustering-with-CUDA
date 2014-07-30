/*
 * FeatureSpace.cpp
 * */

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


// ---------------------------------------- Debug Assertions -------------------------------------------


#ifndef NDEBUG
	#define Success(ans) { gpuAssert((ans), __FILE__, __LINE__, false); }
#else
	#define Success(ans) { gpuAssert((ans), __FILE__, __LINE__); }
#endif



// Check for CUDA error. If release print error and quit. If debug print error and continue
void gpuAssert(const cudaError_t err, const char* file, const int line, bool quit = true)
{
	if(err != cudaSuccess)
	{
		std::cout << "Error: " << cudaGetErrorString(err) << ", File: " << file << ", Line: " << line << std::endl;

		if (quit)
			exit(1);
	}
}




// ------------------ Constructors ----------------

// Parameter constructor takes in number of centroid
FeatureSpace::FeatureSpace( const int K ) : CentroidSize(K), DataSize(0)
{
	jbutil::randgen Rand( std::time(0) );
	Centroids = new FeaturePoint[CentroidSize];

	// Initialize Centroids at random positions and number of points per cluster to 0
	for(int i = 0; i < CentroidSize; i++)
	{
		FeaturePoint v( Rand.fval(LIMIT_L, LIMIT_U), Rand.fval(LIMIT_L, LIMIT_U), 0 );
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
__global__ void SnapSubsets(double* ptr_data, double* ptr_cluster)
{
	// Store previous centroid distance to point
	double distance, newDistance;
	long int id = ( threadIdx.x + (blockDim.x * blockIdx.x) );

	// declare shared memory. Maximum is 1024 2D double points.
	__shared__ double data_xy[2*THREADS_PER_BLOCK];
	__shared__ double data_centroid[THREADS_PER_BLOCK];

	__syncthreads();

	// Copy Point data from global to shared.
	const short int local_threadIdx = threadIdx.x;
	data_xy[local_threadIdx] = ptr_data[id];
	data_xy[local_threadIdx + THREADS_PER_BLOCK] = ptr_data[id + device_datasize];

	// initialize distance as maximum value of double
	distance = DBL_MAX;

	// Protect against Threads accessing inexistent point
	if( id < device_datasize)
	{
		for( int j = 0; j < device_centroidsize; j++)
		{
			// device_ptr_centroids = Constant Memory
			// data_xy = shared memory
			// local_threadIdx = register
			newDistance = ((device_ptr_centroids_xy[j] - data_xy[local_threadIdx] )*(device_ptr_centroids_xy[j] - data_xy[local_threadIdx])) + ((device_ptr_centroids_xy[j + device_centroidsize] - data_xy[local_threadIdx + THREADS_PER_BLOCK] )*(device_ptr_centroids_xy[j + device_centroidsize] - data_xy[local_threadIdx + THREADS_PER_BLOCK] ));

			// If new distance found < previous distance, assign data point to cluster index j
			if( distance > newDistance )
			{
				distance = newDistance;
				data_centroid[local_threadIdx] = j;
			}	
		}
	}

	// copy from shared to global memory
	ptr_data[id] = data_xy[local_threadIdx];
	ptr_data[id + device_datasize] = data_xy[local_threadIdx + THREADS_PER_BLOCK];
	ptr_cluster[id] = data_centroid[local_threadIdx];
};


// Partition feature space into clusters and recursively converge centroids until local minimum achieved
int FeatureSpace::ClusterSearch()
{
	
	int iterations = 0;
	double *device_ptr_data_xy, *host_ptr_point_cluster = NULL, *host_ptr_data_xy = NULL;
	double *device_ptr_point_cluster, *host_ptr_centroids_xy = NULL, currEuclidean = 0, prevEuclidean;

	// Cannot use member functions on GPU. Rearrange Centroid/Feature data into 1D float arrays
	Success( cudaMallocHost((void**)&host_ptr_data_xy, sizeof(double)*DataSize*2 ) );
	Success( cudaMallocHost((void**)&host_ptr_point_cluster, sizeof(double)*DataSize ) );
	Success( cudaMallocHost((void**)&host_ptr_centroids_xy, sizeof(double)*CentroidSize*2 ) );

	for(int i=0; i< DataSize; i++)
	{
		host_ptr_data_xy[i] = PointCloud[i].contents().first;
		host_ptr_data_xy[i + DataSize] = PointCloud[i].contents().second;
		host_ptr_point_cluster[i] = 0;
	}

	// Feature Point [X Y] is constant. Copy Once to GPU once. Not used in constant memory as centroid [X Y] addresses are read more than any feature point address
	Success( cudaMalloc( &device_ptr_data_xy, sizeof(double) * 2 * DataSize ) );
	Success( cudaMalloc( &device_ptr_point_cluster, sizeof(double) * DataSize ) );

	Success( cudaMemcpy(device_ptr_data_xy, host_ptr_data_xy, sizeof(double) * 2 * DataSize,  cudaMemcpyHostToDevice) );
	Success( cudaMemcpy(device_ptr_point_cluster, host_ptr_point_cluster, sizeof(double) * DataSize, cudaMemcpyHostToDevice) );

	Success( cudaMemcpyToSymbol(device_datasize, &DataSize, sizeof(int)) );
	Success( cudaMemcpyToSymbol(device_centroidsize, &CentroidSize, sizeof(int)) );

	cudaFuncSetCacheConfig(SnapSubsets, cudaFuncCachePreferShared);

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
			host_ptr_centroids_xy[i] = Centroids[i].contents().first;
			host_ptr_centroids_xy[i + CentroidSize] = Centroids[i].contents().second;
		};

		// Load new centroid positions to constant memory
		Success( cudaMemcpyToSymbol(device_ptr_centroids_xy, host_ptr_centroids_xy, sizeof(double)*CentroidSize*2) );

		// Launch maximum number of threads
		SnapSubsets <<< (DataSize/THREADS_PER_BLOCK), THREADS_PER_BLOCK>>>(device_ptr_data_xy, device_ptr_point_cluster);

		// Extract Cluster each point assigned to. Recycles a shared array on the GPU. Updates with centroid distance measurements (1 way transfer)
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


	// Clear GPU memory. Centroid and Point data is unchanged. Constant memory not required to be deleted
	cudaFree(device_ptr_point_cluster);
	cudaFree(device_ptr_data_xy);

	cudaFreeHost((void*)host_ptr_data_xy); 	
	cudaFreeHost((void*)host_ptr_point_cluster);	
	cudaFreeHost((void*)host_ptr_centroids_xy);

	return iterations;
};






