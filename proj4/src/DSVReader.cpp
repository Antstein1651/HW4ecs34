#include "DSVReader.h"
#include <sstream>
#include <iostream>


struct CDSVReader::SImplementation {
    std::shared_ptr<CDataSource> Source; 
    char Delimiter; 
    

    SImplementation(std::shared_ptr<CDataSource> src, char delimiter): Source(std::move(src)), Delimiter(delimiter) {}
    
    bool ReadRow(std::vector<std::string> &row) {
        row.clear();
        std::string right; 
        char c; 
        bool quotes = false; 
        bool data = false; 
        
        while (!Source->End()) {
            if (!Source->Get(c)) return false; 
            data = true;
            
            if (c == '"') {
                char next;
                if (!Source->End() && Source->Peek(next)) {
                    if (next == '"') { 
                        Source->Get(next);
                        right += '"';
                    } else {
                        quotes = !quotes;
                    }
                } else {
                    quotes = !quotes; 
                }
            } else if (c == Delimiter && !quotes) {
                row.push_back(std::move(right));
                right.clear();
            } else if ((c == '\n' || c == '\r') && !quotes) {
                if (!right.empty() || !row.empty()) {
                    row.push_back(std::move(right)); 
                }
                
                if (c == '\r' && !Source->End()) {  
                    char next;
                    if (Source->Peek(next) && next == '\n') {
                        Source->Get(next);
                    }
                }
                return true;
            } else {
                right += c; 
            }
        }
        
        if (!right.empty() || !row.empty()) {
            row.push_back(std::move(right));
        }
        return data; 
    }
};


CDSVReader::CDSVReader(std::shared_ptr<CDataSource> src, char delimiter)
    : DImplementation(std::make_unique<SImplementation>(src, delimiter)) {}


CDSVReader::~CDSVReader() = default;


bool CDSVReader::End() const {
    return DImplementation->Source->End();
}


bool CDSVReader::ReadRow(std::vector<std::string> &row) {
    return DImplementation->ReadRow(row);
}