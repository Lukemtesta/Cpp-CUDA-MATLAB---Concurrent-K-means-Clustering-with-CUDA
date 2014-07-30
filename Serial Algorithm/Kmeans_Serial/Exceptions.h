//
//  Exceptions.h
//  

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

#include <iostream>
#include <string>
#include <stdexcept>

#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

// Parent Exception Class. Contains exception debug message so all exceptions can be caught using slicing
class Exception
{
	public:
		    Exception(std::string input);

		    virtual ~Exception();

		    std::string errorMessage;
		    inline void show();

	private:
		    // Stop default constructor
		    Exception();
};


class BadFileException: public Exception
{
	public:
			BadFileException(std::string);
};


class IncompleteFieldException : public Exception
{
	public:
			IncompleteFieldException(std::string);
};


// ------------------------ Inline Definitions ------------------------------------

void Exception::show() { std::cout << "[EXCEPTION] " << errorMessage << std::endl; };


#endif
