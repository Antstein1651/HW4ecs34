#include "DijkstraPathRouter.h"
#include <unordered_map>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <any>
#include <chrono>

struct CDijkstraPathRouter::SImplementation {
    struct Edge {
        TVertexID Dest;
        double Weight;
    };

    std::unordered_map<TVertexID, std::vector<Edge>> AdjacencyList;
    std::unordered_map<TVertexID, std::any> VertexTags;
    TVertexID NextVertexID = 0;

    std::size_t VertexCount() const noexcept {
        return AdjacencyList.size();
    }

    TVertexID AddVertex(std::any tag) noexcept {
        TVertexID id = NextVertexID++;
        AdjacencyList[id] = {};
        VertexTags[id] = std::move(tag);
        return id;
    }

    std::any GetVertexTag(TVertexID id) const noexcept {
        auto it = VertexTags.find(id);
        if (it != VertexTags.end()) {
            return it->second;
        }
        return {};
    }

    bool AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir = false) noexcept {
        if (AdjacencyList.find(src) == AdjacencyList.end() || AdjacencyList.find(dest) == AdjacencyList.end()) {
            return false;
        }
        AdjacencyList[src].push_back({dest, weight});
        if (bidir) {
            AdjacencyList[dest].push_back({src, weight});
        }
        return true;
    }

    bool Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
        // Dijkstra’s algorithm does not require precomputation
        return true;
    }

    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
        const double INF = std::numeric_limits<double>::infinity();
        std::unordered_map<TVertexID, double> Distance;
        std::unordered_map<TVertexID, TVertexID> Previous;
        std::priority_queue<std::pair<double, TVertexID>, std::vector<std::pair<double, TVertexID>>, std::greater<>> MinHeap;

        Distance[src] = 0;
        MinHeap.push({0, src});
        Previous[src] = src; // Mark source with itself

        while (!MinHeap.empty()) {
            auto [currDist, currVertex] = MinHeap.top();
            MinHeap.pop();

            if (currVertex == dest) break;

            for (const auto &edge : AdjacencyList[currVertex]) {
                double newDist = currDist + edge.Weight;
                if (Distance.find(edge.Dest) == Distance.end() || newDist < Distance[edge.Dest]) {
                    Distance[edge.Dest] = newDist;
                    Previous[edge.Dest] = currVertex;
                    MinHeap.push({newDist, edge.Dest});
                }
            }
        }

        // Backtrack from destination to source
        path.clear();
        if (Distance.find(dest) == Distance.end()) {
            return INF; // No path found
        }

        for (TVertexID at = dest; at != src; at = Previous[at]) {
            path.push_back(at);
            if (Previous[at] == at) {
                path.clear(); // No valid path
                return INF;
            }
        }

        path.push_back(src);
        std::reverse(path.begin(), path.end());
        return Distance[dest];
    }
};

CDijkstraPathRouter::CDijkstraPathRouter() : DImplementation(std::make_unique<SImplementation>()) {}

CDijkstraPathRouter::~CDijkstraPathRouter() = default;

std::size_t CDijkstraPathRouter::VertexCount() const noexcept {
    return DImplementation->VertexCount();
}

CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    return DImplementation->AddVertex(std::move(tag));
}

std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    return DImplementation->GetVertexTag(id);
}

bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
    return DImplementation->AddEdge(src, dest, weight, bidir);
}

bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {

    while (std::chrono::steady_clock::now() < deadline) {
       
        if (std::chrono::steady_clock::now() >= deadline) {
            return true;  
        }
    }

    return true;  // In a real scenario, return true if precompute finishes before the deadline.
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
    return DImplementation->FindShortestPath(src, dest, path);
}