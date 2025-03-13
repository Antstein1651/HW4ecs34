// Created Wednesday Feb. 5th 2025 
// Rei Hernandez
//
#include <iostream>
#include "DSVWriter.h"

struct CDSVWriter::SImplementation{
    std::shared_ptr<CDataSink> Sink; //Initalizing shared_ptr Sink
    char Delimiter; //Initalizing delimiting Character variable
    bool QuoteAll; //Initalizing bool quoteall variable

    SImplementation(std::shared_ptr<CDataSink> sink, char delimiter, bool quoteall) {
        Sink = std::move(sink);
        Delimiter = delimiter; // Delimiter variable to hold seperating values
        QuoteAll = quoteall; // QuoteAll
    }
};

// From Assignment: 
// Constructor for DSV writer, sink specifies the data destination, delimiter
// specifies the delimiting character, and quoteall specifies if all values
// should be quoted or only those that contain the delimiter, a double quote,
// or a newline
CDSVWriter::CDSVWriter(std::shared_ptr< CDataSink > sink, char delimiter, bool quoteall){
    DImplementation = std::make_unique<SImplementation>(sink, delimiter, quoteall);
}

// Destructor for DSV writer
CDSVWriter::~CDSVWriter(){}

// Returns true if the row is successfully written, one string per column
// should be put in the row vector
bool CDSVWriter::WriteRow(const std::vector<std::string> &row) {
    // This just checks if the sink and DImplementation exists
    if (!DImplementation || !DImplementation->Sink) {
        return false;  // If no valid data sink, return false
    }

    std::string output;
    for (size_t i = 0; i < row.size(); ++i) {
        std::string cell = row[i];
        bool needsQuotes = DImplementation->QuoteAll;

        // Checking if the cell needs to be quoted
        if (!needsQuotes) {
            if (cell.find(DImplementation->Delimiter) != std::string::npos ||
                cell.find('"') != std::string::npos ||
                cell.find('\n') != std::string::npos) {
                needsQuotes = true;
            }
        }
        if (needsQuotes) {
            std::string quotedCell = "\"";
            for (char ch : cell) {
                if (ch == '"') {
                    quotedCell += "\"\"";  
                } else {
                    quotedCell += ch;
                }
            }
            quotedCell += "\"";
            cell = quotedCell;
        }
        output += cell;
        if (i < row.size() - 1) {
            output += DImplementation->Delimiter;  // Add delimiter between the columns
        }
    }

    output += '\n';  // End the row with newline

    std::vector<char> buffer(output.begin(), output.end()); //Converts string to char vector and gets written to sink
    return DImplementation->Sink->Write(buffer); // Should be a boolean valueee
}