#include "CSVBusSystem.h"
#include <unordered_map>
#include <vector>
#include <sstream>

struct CCSVBusSystem::SImplementation {
    std::unordered_map<TStopID, std::shared_ptr<SStop>> stops; // Stop ID -> Stop object
    std::unordered_map<std::string, std::shared_ptr<SRoute>> routes; // Route name -> Route object
};


// Constructor
CCSVBusSystem::CCSVBusSystem(std::shared_ptr<CDSVReader> stopsrc, std::shared_ptr<CDSVReader> routesrc): DImplementation(std::make_unique<SImplementation>()) {
    
    std::vector<std::string> row;

    // Parse stops.csv
    while (!stopsrc->End()) {
        if (stopsrc->ReadRow(row) && row.size() >= 2) {
            TStopID stopID = std::stoull(row[0]);  // First column is Stop ID
            CStreetMap::TNodeID nodeID = std::stoull(row[1]); // Second column is Node ID

            // Create SStop implementation
            struct SStopImpl : public SStop {
                TStopID stopID;
                CStreetMap::TNodeID nodeID;
                SStopImpl(TStopID s, CStreetMap::TNodeID n) : stopID(s), nodeID(n) {}
                TStopID ID() const noexcept override { return stopID; }
                CStreetMap::TNodeID NodeID() const noexcept override { return nodeID; }
            };

            DImplementation->stops[stopID] = std::make_shared<SStopImpl>(stopID, nodeID);
        }
    }

    // Parse routes.csv
    while (!routesrc->End()) {
        if (routesrc->ReadRow(row) && row.size() >= 2) {
            std::string routeName = row[0];  // First column is Route Name
            std::vector<TStopID> stopIDs;

            // Read stop IDs
            for (size_t i = 1; i < row.size(); ++i) {
                stopIDs.push_back(std::stoull(row[i]));
            }

            // Create SRoute implementation
            struct SRouteImpl : public SRoute {
                std::string routeName;
                std::vector<TStopID> stopIDs;
                SRouteImpl(std::string name, std::vector<TStopID> stops) 
                    : routeName(std::move(name)), stopIDs(std::move(stops)) {}
                std::string Name() const noexcept override { return routeName; }
                std::size_t StopCount() const noexcept override { return stopIDs.size(); }
                TStopID GetStopID(std::size_t index) const noexcept override {
                    return index < stopIDs.size() ? stopIDs[index] : InvalidStopID;
                }
            };

            DImplementation->routes[routeName] = std::make_shared<SRouteImpl>(routeName, stopIDs);
        }
    }
}

// Destructor
CCSVBusSystem::~CCSVBusSystem() {
    // Cleanup if necessary
}

// Return the number of stops
std::size_t CCSVBusSystem::StopCount() const noexcept {
    return DImplementation->stops.size();
}

// Return the number of routes
std::size_t CCSVBusSystem::RouteCount() const noexcept {
    return DImplementation->routes.size();  
}

// Retrieve stop by index
std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByIndex(std::size_t index) const noexcept {
    // Create a vector of stop pointers from the unordered_map
    std::vector<std::shared_ptr<CBusSystem::SStop>> stopVector;
    for (const auto& pair : DImplementation->stops) {
        stopVector.push_back(pair.second);
    }

    // Ensure the index is valid
    if (index >= stopVector.size()) {
        return nullptr;  // Index out of bounds
    }

    // Return the stop at the specified index
    return stopVector[index];
}

// Retrieve stop by ID
std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByID(TStopID id) const noexcept {
    // Search for the stop ID in the unordered_map
    auto it = DImplementation->stops.find(id);

    // If found, return the corresponding shared_ptr to SStop
    if (it != DImplementation->stops.end()) {
        return it->second;
    }

    // If the stop ID is not found, return nullptr
    return nullptr;
}

// Retrieve route by index
std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByIndex(std::size_t index) const noexcept {
    // Create a vector of route pointers from the unordered_map
    std::vector<std::shared_ptr<CBusSystem::SRoute>> routeVector;
    for (const auto& pair : DImplementation->routes) {
        routeVector.push_back(pair.second);
    }

    // Ensure the index is valid
    if (index >= routeVector.size()) {
        return nullptr;  // Index out of bounds
    }

    // Return the route at the specified index
    return routeVector[index];
}
// Retrieve route by name
std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByName(const std::string &name) const noexcept {
    // Search for the route name in the unordered_map
    auto it = DImplementation->routes.find(name);

    // If the route name is found, return the corresponding shared_ptr to SRoute
    if (it != DImplementation->routes.end()) {
        return it->second;
    }

    // If the route name is not found, return nullptr
    return nullptr;
}
