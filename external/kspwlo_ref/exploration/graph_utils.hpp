#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include <vector>

#include "../model/graph.hpp"

template <typename ForwardIt>
ForwardIt remove_self_loops(ForwardIt first, ForwardIt last) {
  return std::remove_if(first, last,
                        [](const Edge &e) { return e.first == e.second; });
}

/**
 * @brief      Builds an Alternative Graph (AG) out of a vector of Paths.
 *			   Basically, an AG is the union of several paths from
 *source to target.
 *
 * @param[in]  paths  A vector of s-t-paths.
 * @param      g      The original graph G.
 *
 * @return     The AG.
 */
RoadNetwork build_AG(const std::vector<Path> &paths, RoadNetwork &g);

#endif