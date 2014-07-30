
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

#include "FileHandler.h"
#include "FeatureSpace.h"
#include <sstream>


/*
 * Running Information:
 *
 * x3 Command Line Arguments: [Dataset(.txt), output path, No Clusters]
 * 
 * Dataset.txt is stored in assignment 1 repository root
 *
 */




// Command line inputs: [Program InputFile OutputFilePath NoClusters]
int main(int argc, char *argv[])
{

	// Check 2 command line inputs
	if ( argc != 4 )
	{
		std::cout << "Needs 4 Arguments: [Program Dataset K OutputPath]" << std::endl;
		std::cout << "Using " << argc << " Arguments" << std::endl;

		for (int i = 0; i < argc; i++)
			std::cout << argv[i] << std::endl;

		return 0;
	}

	// Ensure file exists and open file as read only. Initialize FeatureSpace with N clusters
	FileHandler File( argv[1], true );
	FeatureSpace Dictionary( atoi(argv[3]) );


	// Store data points in feature space
	while ( File.filePtr().peek() != EOF  )
	{
			File.filePtr() >> Dictionary;
	}


	// Start Timer. Find Clusters.

	double executionTime = jbutil::gettime();

	int iterations = Dictionary.ClusterSearch();

	executionTime = jbutil::gettime() - executionTime;
	cudaDeviceReset();

	std::cout << "v10 Execution Time: " << executionTime << " s" << std::endl;
	std::cout << "v10 Iterations: " << iterations << std::endl;

	// End of Timer. Found Clusters. Get Cluster-Allocated Data Points.
	std::vector<FeaturePoint> data = Dictionary.getDataPoints();
	FeaturePoint* centers = Dictionary.getCentroids();

	std::cout << "------------------- Cluster Information -----------------" << std::endl;

	for(int i = 0; i < atoi(argv[3]) ; i++)
		std::cout << "Centroid[" << i << "]: (" << centers[i].contents().first << "," << centers[i].contents().second << ")" << std::endl;


	// -------------------- Output information to file ---------------------------------
/*
	// Timing information
	std::string log_speed(argv[2]);
	log_speed.append("Timing_Information");

	FileHandler speedFile(log_speed, false);
	speedFile.filePtr() << executionTime;

	// Iterations
	std::string log_iteration(argv[2]);
	log_iteration.append("Iteration_Information");

	FileHandler iterationFile(log_iteration, false);
	iterationFile.filePtr() << iterations;

	std::cout << "Execution Time: " << executionTime << " s" << std::endl;
	std::cout << "Iterations: " << iterations << std::endl;


	// Output data to file. For each cluster [ip 3 on command line], for each point in cluster[i]
	for(int i = 0; i < atoi(argv[3]); i++)
	{
		// Format output file name
		std::string temp(argv[2]);
		temp.append("Kmeans_Cluster_");

		std::stringstream val;
		val << i << "_Data";
		temp.append( val.str() );

		// Open Output File
		FileHandler outputFile( temp, false );

		//Write Cluster's Centroid to File
		outputFile.filePtr() << (Point&)centers[i];

		// Write cluster's data points to file
		for(int j = 0; j < data.size(); j++)
		{
			// Output data point to file if contained within current cluster
			if ( data[j].Cluster() == i )
				outputFile.filePtr() << (Point&)data[j];
		}
	}
	*/

	return 0;
}
