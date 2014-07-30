//
//  FileHandler.h
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



#include "Point.h"
#include <fstream>
#include <cstdlib>


// Creates an input or output file stream and handles error checks such as file exist, print file contents etc.
class FileHandler
{
    public:
				// Parameter Constructors
				// Archives filepath and locates file.
                FileHandler(std::string filePath, bool Read);
    
                // Destructors
                virtual ~FileHandler();

                // Debug: check if file has been opened
                // by printing file contents
                void print_fileContents() const;
                inline std::fstream& filePtr();
    
    private:
                // Prevent Implicit Default Constructor. Stop Implicit Bitwise Semantics
                FileHandler();
                FileHandler(FileHandler& copy);
                FileHandler& operator =(FileHandler& copy);

                // If file does not exist, throw Badfile exception
                inline void fileExist() const ;

                // Archive filepath and store point to file stream
                std::string filePath;
                std::fstream fileHandler;

};



//------------------------------ Inline Member Functions -----------------------------


// Returns reference to stream
std::fstream& FileHandler::filePtr()
{
	return fileHandler;
}


// Opens existing file as input stream. If file does not exist, creates file as an output stream
void FileHandler::fileExist() const
{
	if( !fileHandler.good() )
	    throw BadFileException("File Does Not Exist");
}
