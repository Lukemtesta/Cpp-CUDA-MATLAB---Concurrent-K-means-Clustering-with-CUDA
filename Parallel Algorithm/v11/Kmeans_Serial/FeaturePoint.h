/*
 * FeaturePoint.h
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


#include "Point.h"

#ifndef FEATUREPOINT_H_
#define FEATUREPOINT_H_


// Contains the cluster index for the point [ generic solution for data point and centroid container]
class FeaturePoint : public Point
{
	public:
			inline FeaturePoint();
			inline FeaturePoint(const FeaturePoint& pt);
			inline FeaturePoint(const double x, const double y, const int k);
			virtual ~FeaturePoint();

			// Copy operator
			inline FeaturePoint& operator =(FeaturePoint& pt);
			// Reading X Y point data from file
			friend std::istream& operator >>(std::istream& in, FeaturePoint& pt);

			// Return reference to subset index
			inline int& Cluster();

			// Print contents
			void print();

	private:
			int Subset;
};


// ------------------------------- Inline Constructors -------------------------

// Default Constructor
FeaturePoint::FeaturePoint() : Point(), Subset(0) {};

// Copy Constructor : Deep copy to base class using slicing
FeaturePoint::FeaturePoint(const FeaturePoint& pt) : Point(pt), Subset(pt.Subset) {};

// Initialize Feature Data with point position and cluster belong to
FeaturePoint::FeaturePoint(const double x, const double y, const int k) : Point(x,y), Subset(k) {};



// ------------------------------ Inline Operators -----------------------------

// Copy operator : explicit deep copy
FeaturePoint& FeaturePoint::operator =(FeaturePoint& pt)
{
	Subset = pt.Subset;
	(Point&)*this = pt;

	return *this;
}


// ------------------------------ Member Functions ---------------------------------

// Return changable reference to Subset index
int& FeaturePoint::Cluster()
{
	return Subset;
};


#endif /* FEATUREPOINT_H_ */
