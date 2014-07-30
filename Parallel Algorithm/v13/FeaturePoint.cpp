/*
 * FeaturePoint.cpp
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


// --------------------- Destructors -----------------------

FeaturePoint::~FeaturePoint()
{
	//std::cout << "[DELETING] FeaturePoint containing";
	//this->print();
}


// ------------------------------ Operators -----------------------------

// Parse stream data to base class
std::istream& operator >>(std::istream& in, FeaturePoint& pt)
{
	in >> (Point&)pt;

	return in;
}


// ------------------------------ Member Functions --------------------

// Print contents of class : X Y subset
void FeaturePoint::print() const
{
	std::cout << "Subset: " << Subset << std::endl;
	((Point&)(*this)).print();
}
