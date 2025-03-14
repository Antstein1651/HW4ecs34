#include "DijkstraPathRouter.h"
#include <unordered_map>
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

struct CDijkstraPathRouter::SImplementation {
    static constexpr TVertexID InvalidVertexID = std::numeric_limits<TVertexID>::max();
    struct Edge {
        TVertexID Dest;
        double Weight;
    };

    std::unordered_map<TVertexID, std::vector<Edge>> AdjacencyList;
    std::unordered_map<TVertexID, std::any> VertexTags;

    TVertexID AddVertex(std::any tag) {
        TVertexID id = VertexTags.size();
        VertexTags[id] = std::move(tag);
        return id;
    }

    bool AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) {
        if (VertexTags.find(src) == VertexTags.end() || VertexTags.find(dest) == VertexTags.end()) {
            return false;
        }
        AdjacencyList[src].push_back({dest, weight});
        if (bidir) {
            AdjacencyList[dest].push_back({src, weight});
        }
        return true;
    }

    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) {
        std::unordered_map<TVertexID, double> Dist;
        std::unordered_map<TVertexID, TVertexID> Previous;
        
        for (const auto &v : VertexTags) {
            Dist[v.first] = std::numeric_limits<double>::infinity();
            Previous[v.first] = InvalidVertexID;
        }
        Dist[src] = 0;

        using QueueElement = std::pair<double, TVertexID>;
        std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> PQ;
        PQ.push({0, src});

        while (!PQ.empty()) {
            auto [curDist, curVertex] = PQ.top();
            PQ.pop();

            if (curVertex == dest) {
                break;
            }

            if (curDist > Dist[curVertex]) {
                continue;
            }

            for (const auto &edge : AdjacencyList[curVertex]) {
                double newDist = curDist + edge.Weight;
                if (newDist < Dist[edge.Dest]) {
                    Dist[edge.Dest] = newDist;
                    Previous[edge.Dest] = curVertex;
                    PQ.push({newDist, edge.Dest});
                }
            }
        }

        if (Previous[dest] == InvalidVertexID) {
            return std::numeric_limits<double>::infinity();
        }

        for (TVertexID at = dest; at != InvalidVertexID; at = Previous[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        return Dist[dest];
    }
};

CDijkstraPathRouter::CDijkstraPathRouter() : DImplementation(std::make_unique<SImplementation>()) {}
CDijkstraPathRouter::~CDijkstraPathRouter() = default;

std::size_t CDijkstraPathRouter::VertexCount() const noexcept {
    return DImplementation->VertexTags.size();
}

CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    return DImplementation->AddVertex(std::move(tag));
}

std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    return DImplementation->VertexTags.at(id);
}

bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
    return DImplementation->AddEdge(src, dest, weight, bidir);
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
    return DImplementation->FindShortestPath(src, dest, path);
}
