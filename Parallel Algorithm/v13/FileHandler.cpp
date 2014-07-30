//
//  FileHandler.cpp
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

#include "FileHandler.h"


// ------------------------------- Constructors ----------------------------------


// Parameter Constructor
// Archive input filepath and open file.
// Exit if file does not exist
FileHandler::FileHandler(std::string filePath, bool Read) : filePath(filePath)//, fileHandler(filePath.c_str(), RW)
{
	// Open read only or writtable file
	if(Read)
	{
		fileHandler.open(filePath.c_str(), std::ios::in);

		//check if file exists. If not exist throw exception
		try
		{
			fileExist();
		}
		catch(Exception& e)
		{
			e.show();
			std::cout << "File: " << filePath << std::endl;
			std::exit(0);
		}
	}
	else
		fileHandler.open(filePath.c_str(), std::ios::out);

};



//------------------------------ Destructors ----------------------------------------

// Destructor.
// Safely deletes dynamically allocated stream pointer
// and closes file
FileHandler::~FileHandler()
{
    if( fileHandler )
    {
        if( fileHandler.is_open() )
            fileHandler.close();
    }
};



//------------------------------ Member Functions -----------------------------


// IGNORE NOT EXECUTED IN RUN-TIME: print file contents for debugging
void FileHandler::print_fileContents() const
{
    std::string fileLine;
    std::ifstream fileBuffer( filePath.c_str(), std::ios::in );

    // Check if file is empty by peeking next char
    if( fileBuffer.peek() == EOF)
            return;

    // Until EOF, print line
    while(fileBuffer)
    {
        std::getline(fileBuffer,fileLine);
        std::cout << fileLine << std::endl;
    }
};
