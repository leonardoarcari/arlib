#include <kspwlo_ref/exploration/graph_utils.hpp>

#include <algorithm>
#include <cassert>

RoadNetwork build_AG(const std::vector<Path> &paths, RoadNetwork &g) {
  RoadNetwork ag{};

  ag.adjListInc = std::vector<EdgeList>(g.numNodes);
  ag.adjListOut = std::vector<EdgeList>(g.numNodes);
  ag.numNodes = g.numNodes;
  ag.numEdges = 0;

  for (auto &path : paths) {
    auto path_edges = path.getEdges();
    // Cleaning loops coming from dijkstra algorithm (for no reason)
    remove_self_loops(path_edges.begin(), path_edges.end());

    // for (auto &edge : path_edges) {
    //   std::cerr << "(" << edge.first << ", " << edge.second << ") ";
    // }
    // std::cerr << "\n";

    for (auto &edge : path_edges) {
      auto lnode = edge.first;
      auto rnode = edge.second;
      auto w = g.getEdgeWeight(lnode, rnode);

      ag.adjListOut[lnode].insert(std::make_pair(rnode, w));
      ag.adjListInc[rnode].insert(std::make_pair(lnode, w));

      ag.numEdges += 2;
    }
  }

  return ag;
}