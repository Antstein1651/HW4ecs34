#include "OpenStreetMap.h"
#include <memory>
#include <unordered_map>
#include <vector>


struct COpenStreetMap::SImplementation {
    std::unordered_map<CStreetMap::TNodeID, std::shared_ptr<CStreetMap::SNode>> Nodes;
    std::unordered_map<CStreetMap::TWayID, std::vector<CStreetMap::TNodeID>> Ways;
    std::unordered_map<CStreetMap::TWayID, std::unordered_map<std::string, std::string>> WayAttributes;
};

COpenStreetMap::COpenStreetMap(std::shared_ptr<CXMLReader> src)
    : DImplementation(std::make_unique<SImplementation>()) {
    // XML Parsing Logic - Populate DImplementation->Nodes and DImplementation->Ways
}

COpenStreetMap::~COpenStreetMap() = default;

std::size_t COpenStreetMap::NodeCount() const noexcept {
    return DImplementation->Nodes.size();
}

std::size_t COpenStreetMap::WayCount() const noexcept {
    return DImplementation->Ways.size();
}

std::shared_ptr<CStreetMap::SNode> COpenStreetMap::NodeByIndex(std::size_t index) const noexcept {
    if (index >= DImplementation->Nodes.size()) {
        return nullptr;
    }
    auto it = DImplementation->Nodes.begin();
    std::advance(it, index);
    return it->second;
}

std::shared_ptr<CStreetMap::SNode> COpenStreetMap::NodeByID(TNodeID id) const noexcept {
    auto it = DImplementation->Nodes.find(id);
    return (it != DImplementation->Nodes.end()) ? it->second : nullptr;
}

std::shared_ptr<CStreetMap::SWay> COpenStreetMap::WayByID(TWayID id) const noexcept {
    auto it = DImplementation->Ways.find(id);
    if (it == DImplementation->Ways.end()) {
        return nullptr;
    }

    class ConcreteSWay : public CStreetMap::SWay {
        CStreetMap::TWayID wayID;
        std::vector<CStreetMap::TNodeID> nodeRefs;
        const std::unordered_map<std::string, std::string>& attributes;

    public:
        ConcreteSWay(CStreetMap::TWayID id, const std::vector<CStreetMap::TNodeID>& nodes,
                     const std::unordered_map<std::string, std::string>& attr)
            : wayID(id), nodeRefs(nodes), attributes(attr) {}

        CStreetMap::TWayID ID() const noexcept override { return wayID; }
        std::size_t NodeCount() const noexcept override { return nodeRefs.size(); }
        CStreetMap::TNodeID GetNodeID(std::size_t index) const noexcept override {
            return (index < nodeRefs.size()) ? nodeRefs[index] : CStreetMap::InvalidNodeID;
        }
        std::size_t AttributeCount() const noexcept override { return attributes.size(); }
        std::string GetAttributeKey(std::size_t index) const noexcept override {
            if (index >= attributes.size()) return "";
            auto it = attributes.begin();
            std::advance(it, index);
            return it->first;
        }
        bool HasAttribute(const std::string &key) const noexcept override {
            return attributes.find(key) != attributes.end();
        }
        std::string GetAttribute(const std::string &key) const noexcept override {
            auto it = attributes.find(key);
            return (it != attributes.end()) ? it->second : "";
        }
    };

    return std::make_shared<ConcreteSWay>(id, it->second, DImplementation->WayAttributes[id]);
}

std::shared_ptr<CStreetMap::SWay> COpenStreetMap::WayByIndex(std::size_t index) const noexcept {
    if (index >= DImplementation->Ways.size()) {
        return nullptr;
    }
    auto it = DImplementation->Ways.begin();
    std::advance(it, index);
    return WayByID(it->first);
}