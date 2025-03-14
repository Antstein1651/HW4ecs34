#include "TransportationPlannerCommandLine.h"
#include <iostream>
#include <sstream>

struct CTransportationPlannerCommandLine::SImplementation {
    std::shared_ptr<CDataSource> CommandSource;
    std::shared_ptr<CDataSink> OutputSink;
    std::shared_ptr<CDataSink> ErrorSink;
    std::shared_ptr<CDataFactory> ResultsFactory;
    std::shared_ptr<CTransportationPlanner> Planner;
    
    SImplementation(std::shared_ptr<CDataSource> cmdsrc, std::shared_ptr<CDataSink> outsink,
                    std::shared_ptr<CDataSink> errsink, std::shared_ptr<CDataFactory> results,
                    std::shared_ptr<CTransportationPlanner> planner)
        : CommandSource(cmdsrc), OutputSink(outsink), ErrorSink(errsink), ResultsFactory(results), Planner(planner) {}
    
    bool ProcessCommands() {
        std::vector<char> buffer(1024);
        while (!CommandSource->End()) {
            std::size_t bytesRead = CommandSource->Read(buffer, buffer.size());
            if (bytesRead == 0) {
                continue;
            }
            std::string command(buffer.begin(), buffer.begin() + bytesRead);
            std::istringstream stream(command);
            std::string action;
            stream >> action;
            
            if (action == "shortest_path") {
                CTransportationPlanner::TNodeID src, dest;
                stream >> src >> dest;
                std::vector<CTransportationPlanner::TNodeID> path;
                double distance = Planner->FindShortestPath(src, dest, path);
                
                std::string response = (distance != CPathRouter::NoPathExists) ? "Distance: " + std::to_string(distance) : "No path found.";
                OutputSink->Write(std::vector<char>(response.begin(), response.end()));
            } else if (action == "fastest_path") {
                CTransportationPlanner::TNodeID src, dest;
                stream >> src >> dest;
                std::vector<CTransportationPlanner::TTripStep> path;
                double time = Planner->FindFastestPath(src, dest, path);
                
                std::string response = (time != CPathRouter::NoPathExists) ? "Time: " + std::to_string(time) : "No path found.";
                OutputSink->Write(std::vector<char>(response.begin(), response.end()));
            } else {
                std::string errorMsg = "Unknown command: " + action;
                ErrorSink->Write(std::vector<char>(errorMsg.begin(), errorMsg.end()));
            }
        }
        return true;
    }
};

CTransportationPlannerCommandLine::CTransportationPlannerCommandLine(
    std::shared_ptr<CDataSource> cmdsrc, std::shared_ptr<CDataSink> outsink,
    std::shared_ptr<CDataSink> errsink, std::shared_ptr<CDataFactory> results,
    std::shared_ptr<CTransportationPlanner> planner)
    : DImplementation(std::make_unique<SImplementation>(cmdsrc, outsink, errsink, results, planner)) {}

CTransportationPlannerCommandLine::~CTransportationPlannerCommandLine() = default;

bool CTransportationPlannerCommandLine::ProcessCommands() {
    return DImplementation->ProcessCommands();
}
