#include "BusSystemIndexer.h"
#include "BusSystem.h"
#include "StreetMap.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

struct CBusSystemIndexer::SImplementation {
    std::shared_ptr<CBusSystem> BusSystem;
    std::vector<std::shared_ptr<SStop>> SortedStops;
    std::vector<std::shared_ptr<SRoute>> SortedRoutes;
    std::unordered_map<TNodeID, std::shared_ptr<SStop>> NodeIDToStop;
    std::unordered_map<TNodeID, std::unordered_set<std::shared_ptr<SRoute>>> NodeIDToRoutes;

    SImplementation(std::shared_ptr<CBusSystem> bussystem) : BusSystem(bussystem) {
        // Populate sorted stops
        for (std::size_t i = 0; i < BusSystem->StopCount(); ++i) {
            auto stop = BusSystem->StopByIndex(i);
            SortedStops.push_back(stop);
            NodeIDToStop[stop->NodeID()] = stop;
        }
        std::sort(SortedStops.begin(), SortedStops.end(), 
                  [](const auto &a, const auto &b) { return a->NodeID() < b->NodeID(); });
        
        // Populate sorted routes
        for (std::size_t i = 0; i < BusSystem->RouteCount(); ++i) {
            auto route = BusSystem->RouteByIndex(i);
            SortedRoutes.push_back(route);
            
            for (std::size_t j = 0; j < route->StopCount(); ++j) {
                auto stop = BusSystem->StopByID(route->GetStopID(j));
                NodeIDToRoutes[stop->NodeID()].insert(route);
            }
        }
        std::sort(SortedRoutes.begin(), SortedRoutes.end(), 
                  [](const auto &a, const auto &b) { return a->Name() < b->Name(); });
    }
};

CBusSystemIndexer::CBusSystemIndexer(std::shared_ptr<CBusSystem> bussystem)
    : DImplementation(std::make_unique<SImplementation>(bussystem)) {}

CBusSystemIndexer::~CBusSystemIndexer() = default;

std::size_t CBusSystemIndexer::StopCount() const noexcept {
    return DImplementation->SortedStops.size();
}

std::size_t CBusSystemIndexer::RouteCount() const noexcept {
    return DImplementation->SortedRoutes.size();
}

std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::SortedStopByIndex(std::size_t index) const noexcept {
    if (index < DImplementation->SortedStops.size()) {
        return DImplementation->SortedStops[index];
    }
    return nullptr;
}

std::shared_ptr<CBusSystem::SRoute> CBusSystemIndexer::SortedRouteByIndex(std::size_t index) const noexcept {
    if (index < DImplementation->SortedRoutes.size()) {
        return DImplementation->SortedRoutes[index];
    }
    return nullptr;
}

std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::StopByNodeID(TNodeID id) const noexcept {
    auto it = DImplementation->NodeIDToStop.find(id);
    return (it != DImplementation->NodeIDToStop.end()) ? it->second : nullptr;
}

bool CBusSystemIndexer::RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<SRoute>> &routes) const noexcept {
    auto it_src = DImplementation->NodeIDToRoutes.find(src);
    auto it_dest = DImplementation->NodeIDToRoutes.find(dest);
    if (it_src == DImplementation->NodeIDToRoutes.end() || it_dest == DImplementation->NodeIDToRoutes.end()) {
        return false;
    }
    
    for (const auto &route : it_src->second) {
        if (it_dest->second.count(route)) {
            routes.insert(route);
        }
    }
    return !routes.empty();
}

bool CBusSystemIndexer::RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const noexcept {
    std::unordered_set<std::shared_ptr<SRoute>> routes;
    return RoutesByNodeIDs(src, dest, routes);
}
