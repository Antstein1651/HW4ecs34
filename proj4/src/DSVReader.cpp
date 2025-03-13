// Created Wednesday Feb. 5th 2025
// Rei Hernandez
// Anthony Richards
//
#include <vector>
#include <iostream>
#include <sstream>
#include "DSVReader.h"

// The private structure that holds the data members: Source, Delimeter and LeftOvers
struct CDSVReader::SImplementation { 
    std::shared_ptr<CDataSource> Source; //Pointer to the data source where the data gets read.
    char Delimiter; //Stands for , \t, and more...Used to seperate values
    std::string Leftovers; //Leftovers = text we didn't process yet
    SImplementation(std::shared_ptr<CDataSource> src, char delimiter){
        Source = src; //Initalizing src as Source
        Delimiter = delimiter; //Initalizing Delimiter as delimiter 
    }
};
//Assignment:
// Constructor for DSV reader that takes in two parameters,
// src specifies the data source
// delimiter specifies the delimiting character
CDSVReader::CDSVReader(std::shared_ptr< CDataSource > src, char delimiter){
    DImplementation = std::make_unique<SImplementation>(src, delimiter); //Initalizing pointer to manage memory //Default constructor print statement
}

// Assignment:
// Destructor for DSV reader
CDSVReader::~CDSVReader() = default;

// Returns true if all rows have been read from the DSV
// So it's true if...
// Source as no more data
// There's no more leftover characters to process
bool CDSVReader::End() const{
    return DImplementation->Source->End() && DImplementation->Leftovers.empty(); //Returns bool status of End()
}
// Assignment: 
// Returns true if the row is successfully read, 
// one string will be put in the row per column
bool CDSVReader::ReadRow(std::vector<std::string> &row){
    row.clear();  

    std::string line;

    // Ensure we have a full line to process
    while (true) {
        if (DImplementation->Leftovers.empty()) {
            std::vector<char> buffer(1024);
            if (!DImplementation->Source->Read(buffer, buffer.size()) || buffer.empty()) {
                return false; 
            }
            std::string newData(buffer.begin(), buffer.end());
            DImplementation->Leftovers += newData;
        }

        size_t newlinePos = DImplementation->Leftovers.find('\n');
        if (newlinePos != std::string::npos) {
            line = DImplementation->Leftovers.substr(0, newlinePos);
            DImplementation->Leftovers.erase(0, newlinePos + 1);
            break;
        } else {
            std::vector<char> buffer(1024);
            if (!DImplementation->Source->Read(buffer, buffer.size()) || buffer.empty()) {
                line = DImplementation->Leftovers;
                DImplementation->Leftovers.clear();
                break;
            }
            std::string newData(buffer.begin(), buffer.end());
            DImplementation->Leftovers += newData;
        }
    }

    if (line.empty()) {
        return false;
    }

    std::stringstream ss(line);
    std::string cell;
    bool insideQuotes = false;
    std::string currentCell;

    for (size_t i = 0; i < line.size(); ++i) {
        char ch = line[i];
    
        if (ch == '"') {
            if (insideQuotes && i + 1 < line.size() && line[i + 1] == '"') {
                // Handle escaped quotes inside a quoted value ("" → ")
                currentCell += '"';
                ++i;  // Skip the next quote
            } else {
                // Toggle insideQuotes flag
                insideQuotes = !insideQuotes;
            }
        } else if (ch == DImplementation->Delimiter && !insideQuotes) {
            row.push_back(currentCell);
            currentCell.clear();
        } else {
            currentCell += ch;
        }
    }
    
    // Add the last parsed cell
    row.push_back(currentCell);

    return !row.empty(); 
}