#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include <cassert>
#include <vector>
#include <algorithm>

#include "../model/graph.hpp"

template<typename ForwardIt>
ForwardIt remove_self_loops(ForwardIt first, ForwardIt last) {
  return std::remove_if(first, last, [](const Edge &e) {
    return e.first == e.second;
  });
}

/**
 * @brief      Builds an Alternative Graph (AG) out of a vector of Paths.
 *			   Basically, an AG is the union of several paths from source
 *			   to target.
 *
 * @param[in]  paths  A vector of s-t-paths.
 * @param      g      The original graph G.
 *
 * @return     The AG.
 */
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

#endif