#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include <unordered_map>
#include <queue>
#include <limits>
#include <cmath>

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<SConfiguration> Config;
    std::shared_ptr<CStreetMap> StreetMap;
    std::shared_ptr<CBusSystem> BusSystem;
    CDijkstraPathRouter PathRouter;
    
    SImplementation(std::shared_ptr<SConfiguration> config) : Config(config) {
        StreetMap = Config->StreetMap();
        BusSystem = Config->BusSystem();
    }
    
    std::size_t NodeCount() const noexcept {
        return StreetMap->NodeCount();
    }
    
    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const noexcept {
        return StreetMap->NodeByIndex(index);
    }
    
    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
        return PathRouter.FindShortestPath(src, dest, path);
    }
    
    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
        std::vector<TNodeID> shortestPath;
        double distance = FindShortestPath(src, dest, shortestPath);
        
        if (distance == CPathRouter::NoPathExists) {
            return CPathRouter::NoPathExists;
        }
        
        path.clear();
        for (TNodeID node : shortestPath) {
            path.emplace_back(ETransportationMode::Walk, node);
        }
        
        return distance / Config->WalkSpeed();
    }
    
    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();
        for (const auto &[mode, node] : path) {
            std::string step = "Mode: " + std::to_string(static_cast<int>(mode)) + " -> Node: " + std::to_string(node);
            desc.push_back(step);
        }
        return true;
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config)
    : DImplementation(std::make_unique<SImplementation>(config)) {}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() = default;

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->NodeCount();
}

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedNodeByIndex(index);
}

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    return DImplementation->FindShortestPath(src, dest, path);
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    return DImplementation->FindFastestPath(src, dest, path);
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    return DImplementation->GetPathDescription(path, desc);
}
