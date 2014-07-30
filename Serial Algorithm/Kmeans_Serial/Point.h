/*
 * Point.h
 *
 *  Created on: 7 Mar 2014
 *      Author: lt00089
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

#include "Exceptions.h"
#include <sstream>

#ifndef POINT_H
#define POINT_H

// Generic 2D point class. Contains the X and Y positions of point with interface for file writing and vector/scalar oprations
class Point
{
	public:
			inline Point();
			inline Point(const Point& pt);
			inline Point(const double x, const double y);
			virtual ~Point();

			// Copy operator
			inline Point& operator=(const Point& pt);

			// Reading/Writing numbers from file in X Y row format
			friend std::istream& operator >>(std::istream& in, Point& pt);
			friend std::ostream& operator <<(std::ostream& out, Point& pt);

			// Vector/Scalar algebra operations: Manipulating Two Points or Manipulating a point and scalar using an operator interface
			inline Point& operator +=(const Point& pt);
			inline Point& operator -=(const Point& pt);
			inline Point& operator *=(const Point& pt);
			inline Point& operator *=(const double val);
			inline Point& operator /=(const double val);

			// debug printing and return stored point's X and Y positions respectively
			virtual void print() const;
			inline std::pair<double,double> contents() const;

	private:

			std::pair<double, double> Position;
};



// ------------------------ Inline Constructors -------------


// Default constructor initializes point at origin
Point::Point() : Position( std::pair<double,double>(0,0) ) {};

// Explicit copy constructor uses deep copy (not exception safe)
Point::Point(const Point& pt) : Position(pt.Position) {};

// Parameter Constructor: Initialize pt with X and Y
Point::Point(const double x, const double y) : Position( std::pair<double,double>(x,y) ) {};



// ----------------------- Inline Operator Overloads ----------------


// Copy operator : deep copy not exception safe
Point& Point::operator =(const Point& pt)
{
	Position = pt.Position;
	return *this;
};



// Sum of Two Points
Point& Point::operator +=(const Point& pt)
{
	Position.first += pt.Position.first;
	Position.second += pt.Position.second;

	return *this;
}


// Subtract of Two Points
Point& Point::operator -=(const Point& pt)
{
	Position.first -= pt.Position.first;
	Position.second -= pt.Position.second;

	return *this;
}


// Divide each dimension by a number
Point& Point::operator /=(const double val)
{
	Position.first /= val;
	Position.second /= val;

	return *this;
}

// per element multiplication
Point& Point::operator *=(const Point& pt)
{
	Position.first *= pt.Position.first;
	Position.second *= pt.Position.second;

	return *this;
}

// Multiply each dimension by a number
Point& Point::operator *=(const double val)
{
	Position.first *= val;
	Position.second *= val;

	return *this;
}



// ----------------------- Inline Member Functions ------------------


// return copy of Position
std::pair<double,double> Point::contents() const
{
	return Position;
};


#endif

