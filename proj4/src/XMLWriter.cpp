#include "XMLWriter.h"
#include <iostream> 

//Implementation class for CXMLWriter
struct CXMLWriter::SImplementation {
     //the data sink for writing data
    std::shared_ptr<CDataSink> Sink; 
    //leftovers string for XML output as a simple string
    std::string Leftovers;            

    SImplementation(std::shared_ptr<CDataSink> sink) : Sink(sink) {}

    bool LeftoversToSink() {
        //convert Leftovers to a vector of characters
        std::vector<char> leftover(Leftovers.begin(), Leftovers.end());
        // Write the leftover string to the sink
        return Sink->Write(leftover);  
    }
};

CXMLWriter::CXMLWriter(std::shared_ptr<CDataSink> sink): DImplementation(std::make_unique<SImplementation>(sink)) 
{
 
}

CXMLWriter::~CXMLWriter() = default;

bool CXMLWriter::Flush() {
    //write leftovers string to the sink
    return DImplementation->LeftoversToSink();  
}

bool CXMLWriter::WriteEntity(const SXMLEntity &entity) {
    if (entity.DType == SXMLEntity::EType::StartElement) {
        // write the first element
        DImplementation->Leftovers += "<" + entity.DNameData;
        
        //write the middle attibutes of the leftovers
        for (const auto &attr : entity.DAttributes) {
            DImplementation->Leftovers += " " + std::get<0>(attr) + "=\"" + std::get<1>(attr) + "\"";
        }
         
        DImplementation->Leftovers += ">";
    }
    else if (entity.DType == SXMLEntity::EType::EndElement) {
        //write the last element in the leftovers
        DImplementation->Leftovers += "</" + entity.DNameData + ">";
    }
    else if (entity.DType == SXMLEntity::EType::CharData) {
        //write the data of the xml name
        DImplementation->Leftovers += entity.DNameData;
    }
    else if (entity.DType == SXMLEntity::EType::CompleteElement) {
        // write the entire leftovers string
        DImplementation->Leftovers += "<" + entity.DNameData;
        
        //write the attributes of the ends of the leftovers
        for (const auto &attr : entity.DAttributes) {
            DImplementation->Leftovers += " " + std::get<0>(attr) + "=\"" + std::get<1>(attr) + "\"";
        }
        
        DImplementation->Leftovers += "/>";
    }
    
    //write the leftovers to the sink after all entities and attributes are added
    return DImplementation->LeftoversToSink();
}