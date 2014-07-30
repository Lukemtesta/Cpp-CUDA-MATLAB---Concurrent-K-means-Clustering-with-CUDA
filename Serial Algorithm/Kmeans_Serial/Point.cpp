/*
 * Point.cpp
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




// --------------------------- Destructors ---------------------

Point::~Point()
{
	//std::cout << "[DELETING] Point Containing ";
	//print();
}



// --------------------------- Operator Overloads ---------------------



// Parse first two numbers to file. Assumes input is in correct format
std::istream& operator >>(std::istream& in, Point& pt)
{

	in >> pt.Position.first >> pt.Position.second;

	// check if fail occurred
	if ( in.eof() )
		std::cout << "End of File Reached" << std::endl;
	else if ( in.fail() )
			throw IncompleteFieldException("Position Field was not Read in X Y row format");

	return in;
};


// Parse two numbers to file and append new line character. Assumes input is in correct format
std::ostream& operator <<(std::ostream& out, Point& pt)
{
	out << pt.Position.first << " " << pt.Position.second << std::endl;

	if ( out.fail() )
		throw IncompleteFieldException("Position Field was not Written Correctly");

	return out;
};



// ---------------------------------------- Member Functions ----------------------------------


// Display the current X and Y co-ordinates of the stored point
void Point::print() const
{
	std::cout << "(" << Position.first << "," << Position.second << ")" << std::endl;
};

