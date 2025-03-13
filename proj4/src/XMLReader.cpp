// Created Wednesday Feb. 5th 2025
#include <iostream>
#include "XMLReader.h"

struct CXMLReader::SImplementation {
    std::shared_ptr<CDataSource> Source;
    bool EndOfFile; //true or false for if we are at the end of the file
    std::string Leftovers; //Leftovers = text we didn't process yet

    SImplementation(std::shared_ptr<CDataSource> src) {
        Source = src; // Initializing Source with src
        EndOfFile = false;  // Initializing EndOfFile to false
        Leftovers = ""; // Initializing the leftovers as an empty string
    }
};


// Constructor
CXMLReader::CXMLReader(std::shared_ptr<CDataSource> src) : DImplementation(std::make_unique<SImplementation>(src)) {
}


// Destructor
CXMLReader::~CXMLReader()
{
}

// Check if end of data source is reached
bool CXMLReader::End() const
{
    //return whether or not we are at the end of the file
   return DImplementation->EndOfFile;  
}

// Read an XML entity
bool CXMLReader::ReadEntity(SXMLEntity &entity, bool skipcdata) 
{
    char c;
    std::string Leftovers;  // Accumulate characters until an entity is detected
    bool insideTag = false;
    
    // read all the characters in the source file
    while (DImplementation->Source->Get(c)) 
    {
        // if the character is the start of an XML
        if (c == '<') 
        {  
            if (!Leftovers.empty()) 
            {
                entity.DType = SXMLEntity::EType::CharData;
                entity.DNameData = Leftovers;
                return true;
            }
            // clear the leftover string
            Leftovers.clear(); 
            // we are inside the XML tag
            insideTag = true;
        } 
        // we've reached the end of an XML tag
        else if (c == '>') 
        {  
            insideTag = false;

            if (Leftovers.empty()) 
            {
                continue;
            }

            // check if the first character is a forward slash
            if (Leftovers[0] == '/') 
            {  // end tag 
                entity.DType = SXMLEntity::EType::EndElement;
                // remove '/' from the tag 
                entity.DNameData = Leftovers.substr(1); 
            } 
            else if (Leftovers.back() == '/') 
            {  // if the last character is a forward slash
                entity.DType = SXMLEntity::EType::CompleteElement;
                // remove "/" from the tag
                entity.DNameData = Leftovers.substr(0, Leftovers.size() - 1);  
            } 
            else 
            {   
                entity.DType = SXMLEntity::EType::StartElement;

                // go through each element and find the first space in the tag 
                size_t position = Leftovers.find_first_of(' ');  
                // check if the position is at a valid index
                if (position != std::string::npos) 
                {
                    // the tag name is the first 'word' before any spaces
                    entity.DNameData = Leftovers.substr(0, position);

                    // go through elements after the first space until we find an equals
                    std::string attributes = Leftovers.substr(position + 1);
                    size_t pos = 0;
                    // find the positions of the equal signs
                    while ((pos = attributes.find('=')) != std::string::npos) 
                    {
                        // get the attribute name
                        std::string attributeName = attributes.substr(0, pos);
                        attributes = attributes.substr(pos + 1);

                        // find the first quote
                        size_t quotePos = attributes.find_first_of('\"');
                        if (quotePos != std::string::npos) 
                        {
                            // go immediately after the first quote
                            attributes = attributes.substr(quotePos + 1);  
                            quotePos = attributes.find_first_of('\"');
                            // find the index after the next quote
                            if (quotePos != std::string::npos) 
                            {
                                // get the value between the quotes
                                std::string attrValue = attributes.substr(0, quotePos);  
                                entity.DAttributes.emplace_back(attributeName, attrValue);
                                // go immediately after the second quote
                                attributes = attributes.substr(quotePos + 1);  
                            }
                        }
                    }
                } 
                else
                {
                    // the tag name is the entire string if there is no space in it
                    entity.DNameData = Leftovers;
                }
            }
            return true;
        } 
        else 
        {
            // total the characters in the leftovers variable
            Leftovers += c;  
        }
    }

    // end of the file
    DImplementation->EndOfFile = true;
    return false;  
}