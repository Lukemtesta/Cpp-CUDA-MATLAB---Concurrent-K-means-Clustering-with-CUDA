//
//  Exceptions.cpp
//  CPP-Coursework
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

#include "Exceptions.h"


Exception::Exception(std::string input): errorMessage(input){};
Exception::~Exception()
{
	//std::cerr << "[DELETING] Exception Class" << std::endl;
};


BadFileException::BadFileException(std::string input): Exception(input){};
IncompleteFieldException::IncompleteFieldException(std::string input): Exception(input){};











