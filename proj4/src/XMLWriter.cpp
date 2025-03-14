#include "XMLWriter.h"
#include <stack>
#include <string>

struct CXMLWriter::SImplementation {
    std::shared_ptr<CDataSink> Sink;  
    std::stack<std::string> Stack;    


    explicit SImplementation(std::shared_ptr<CDataSink> sink) 
        : Sink(std::move(sink)) {}


    bool WriteText(const std::string &str, bool escape) {
        for (char c : str) {
            if (escape) {
          
                switch (c) {
                    case '<':  if (!WriteText("&lt;", false)) return false; break;
                    case '>':  if (!WriteText("&gt;", false)) return false; break;
                    case '&':  if (!WriteText("&amp;", false)) return false; break;
                    case '\'': if (!WriteText("&apos;", false)) return false; break;
                    case '"':  if (!WriteText("&quot;", false)) return false; break;
                    default:   if (!Sink->Put(c)) return false;
                }
            } else {
                if (!Sink->Put(c)) return false;
            }
        }
        return true;
    }

    bool Flush() {
        while (!Stack.empty()) {
            if (!WriteText("</" + Stack.top() + ">", false)) {
                return false;
            }
            Stack.pop();
        }
        return true;
    }


    bool WriteEntity(const SXMLEntity &entity) {
        switch (entity.DType) {
       
            case SXMLEntity::EType::StartElement:
                if (!WriteText("<" + entity.DNameData, false)) return false;
                for (const auto &attr : entity.DAttributes) {
                    if (!WriteText(" " + attr.first + "=\"", false)) return false;
                    if (!WriteText(attr.second, true)) return false;  
                    if (!WriteText("\"", false)) return false;
                }

                if (!WriteText(">", false)) return false;
                Stack.push(entity.DNameData);
                break;
            case SXMLEntity::EType::EndElement:
                if (!WriteText("</" + entity.DNameData + ">", false)) return false;
                if (!Stack.empty()) {
                    Stack.pop();
                }
                break;


            case SXMLEntity::EType::CharData:
                if (!WriteText(entity.DNameData, true)) return false;  
                break;

            case SXMLEntity::EType::CompleteElement:
                if (!WriteText("<" + entity.DNameData, false)) return false;

                for (const auto &attr : entity.DAttributes) {
                    if (!WriteText(" " + attr.first + "=\"", false)) return false;
                    if (!WriteText(attr.second, true)) return false;  
                    if (!WriteText("\"", false)) return false;
                }

                if (!WriteText("/>", false)) return false;
                break;
        }
        return true;
    }
};

CXMLWriter::CXMLWriter(std::shared_ptr<CDataSink> sink)
    : DImplementation(std::make_unique<SImplementation>(std::move(sink))) {}


CXMLWriter::~CXMLWriter() = default;

bool CXMLWriter::Flush() {
    return DImplementation->Flush();
}

bool CXMLWriter::WriteEntity(const SXMLEntity &entity) {
    return DImplementation->WriteEntity(entity);
}
