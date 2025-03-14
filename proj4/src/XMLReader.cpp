#include "XMLReader.h"
#include <expat.h>
#include <queue>
#include <memory>
#include <vector>

struct CXMLReader::SImplementation {
    std::shared_ptr<CDataSource> Source; 
    XML_Parser Parser; 
    std::queue<SXMLEntity> Queue;
    bool Data;
    std::string Buffer; 
    static void ElementHandler(void *userData, const char *name, const char **element, bool isStart) {
        auto *impl = static_cast<SImplementation *>(userData);
        impl->FlushCharData(); 

        SXMLEntity entity;
        entity.DType = isStart ? SXMLEntity::EType::StartElement : SXMLEntity::EType::EndElement;
        entity.DNameData = name;


        if (isStart && element) {
            for (int i = 0; element[i] != nullptr; i += 2) {
                if (element[i + 1] != nullptr) {
                    entity.DAttributes.emplace_back(element[i], element[i + 1]);
                }
            }
        }

        impl->Queue.push(entity);  
    }


    static void StartElementHandler(void *userData, const char *name, const char **element) {
        ElementHandler(userData, name, element, true);
    }

  
    static void EndElementHandler(void *userData, const char *name) {
        ElementHandler(userData, name, nullptr, false);
    }


    static void CharDataHandler(void *userData, const char *j, int len) {
        if (j && len > 0) {
            static_cast<SImplementation *>(userData)->Buffer.append(j, len);  
        }
    }

    
    SImplementation(std::shared_ptr<CDataSource> src) : Source(std::move(src)), Data(false) {
        Parser = XML_ParserCreate(nullptr);
        XML_SetUserData(Parser, this);
        XML_SetElementHandler(Parser, StartElementHandler, EndElementHandler);
        XML_SetCharacterDataHandler(Parser, CharDataHandler);
    }

    
    ~SImplementation() {
        XML_ParserFree(Parser);
    }


    void FlushCharData() {
        if (!Buffer.empty()) {
            Queue.push({SXMLEntity::EType::CharData, Buffer, {}});
            Buffer.clear(); 
        }
    }

   
    bool ReadEntity(SXMLEntity &entity, bool skipcdata) {
        while (Queue.empty() && !Data) {
            std::vector<char> buffer(4096);
            size_t length = 0;
            while (length < buffer.size() && !Source->End()) {
                char ch;
                if (Source->Get(ch)) {
                    buffer[length++] = ch;\
                } else {
                    break;
                }
            }

            if (length == 0) {
                Data = true;
                XML_Parse(Parser, nullptr, 0, 1); 
                break;
            }

            if (XML_Parse(Parser, buffer.data(), length, 0) == XML_STATUS_ERROR) {
                return false;  
            }
        }

        if (!Queue.empty()) {
            entity = Queue.front();
            Queue.pop();
            return !(skipcdata && entity.DType == SXMLEntity::EType::CharData) || ReadEntity(entity, skipcdata);
        }

        return false;  
    }
};


CXMLReader::CXMLReader(std::shared_ptr<CDataSource> src): DImplementation(std::make_unique<SImplementation>(std::move(src))) {}

CXMLReader::~CXMLReader() = default; 

bool CXMLReader::End() const {
    return DImplementation->Data && DImplementation->Queue.empty();
}


bool CXMLReader::ReadEntity(SXMLEntity &entity, bool skipcdata) {
    return DImplementation->ReadEntity(entity, skipcdata);
}
