#include "DijkstraPathRouter.h"
#include <unordered_map>
#include <vector>
#include <queue>
#include <limits>
#include <optional>

struct CDijkstraPathRouter::SImplementation {
    struct Edge {
        TVertexID Dest;
        double Weight;
    };
    
    struct VertexData {
        std::any Tag;
        std::vector<Edge> Edges;
    };
    
    std::vector<VertexData> DVertices;
    
    std::size_t VertexCount() const noexcept {
        return DVertices.size();
    }
    
    TVertexID AddVertex(std::any tag) noexcept {
        DVertices.push_back({tag, {}});
        return DVertices.size() - 1;
    }
    
    std::any GetVertexTag(TVertexID id) const noexcept {
        if (id < DVertices.size()) {
            return DVertices[id].Tag;
        }
        return {};
    }
    
    bool AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
        if (src >= DVertices.size() || dest >= DVertices.size() || weight < 0) {
            return false;
        }
        DVertices[src].Edges.push_back({dest, weight});
        if (bidir) {
            DVertices[dest].Edges.push_back({src, weight});
        }
        return true;
    }
    
    bool Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
        // No preprocessing needed for standard Dijkstra's algorithm.
        return true;
    }
    
    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
        if (src >= DVertices.size() || dest >= DVertices.size()) {
            return NoPathExists;
        }
        
        std::vector<double> Distances(DVertices.size(), NoPathExists);
        std::vector<std::optional<TVertexID>> Previous(DVertices.size());
        std::priority_queue<std::pair<double, TVertexID>, std::vector<std::pair<double, TVertexID>>, std::greater<>> MinHeap;
        
        Distances[src] = 0;
        MinHeap.emplace(0, src);
        
        while (!MinHeap.empty()) {
            auto [currentDist, current] = MinHeap.top();
            MinHeap.pop();
            
            if (current == dest) break;
            if (currentDist > Distances[current]) continue;
            
            for (const auto &edge : DVertices[current].Edges) {
                double newDist = currentDist + edge.Weight;
                if (newDist < Distances[edge.Dest]) {
                    Distances[edge.Dest] = newDist;
                    Previous[edge.Dest] = current;
                    MinHeap.emplace(newDist, edge.Dest);
                }
            }
        }
        
        if (Distances[dest] == NoPathExists) {
            return NoPathExists;
        }
        
        path.clear();
        for (TVertexID at = dest; at.has_value(); at = Previous[at.value()]) {
            path.push_back(at.value());
        }
        std::reverse(path.begin(), path.end());
        return Distances[dest];
    }
};

CDijkstraPathRouter::CDijkstraPathRouter() : DImplementation(std::make_unique<SImplementation>()) {}
CDijkstraPathRouter::~CDijkstraPathRouter() = default;

std::size_t CDijkstraPathRouter::VertexCount() const noexcept {
    return DImplementation->VertexCount();
}

CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    return DImplementation->AddVertex(tag);
}

std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    return DImplementation->GetVertexTag(id);
}

bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
    return DImplementation->AddEdge(src, dest, weight, bidir);
}

bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    return DImplementation->Precompute(deadline);
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
    return DImplementation->FindShortestPath(src, dest, path);
}
