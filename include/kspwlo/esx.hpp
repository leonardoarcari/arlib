#ifndef BOOST_EDGE_SUBSET_EXCLUSION_HPP
#define BOOST_EDGE_SUBSET_EXCLUSION_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/graph_types.hpp"
#include "kspwlo/impl/esx_impl.hpp"

#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>

namespace boost {
template <typename PropertyGraph,
          typename Vertex =
              typename boost::graph_traits<PropertyGraph>::vertex_descriptor>
std::vector<kspwlo::Path<PropertyGraph>> esx(PropertyGraph &G, Vertex s,
                                             Vertex t, int k, double theta) {
  // P_LO set of k paths
  using Length = typename boost::property_traits<typename boost::property_map<
      PropertyGraph, boost::edge_weight_t>::type>::value_type;
  auto resPaths = std::vector<kspwlo::Path<PropertyGraph>>{};
  auto resPathsEdges = std::vector<std::vector<kspwlo::Edge>>{};
  auto resPathsLengths = std::vector<Length>{};

  // Compute lower bounds for AStar
  auto heuristic = kspwlo_impl::distance_heuristic<PropertyGraph, Length>(G, t);

  // Compute shortest path from s to t
  auto sp_path = kspwlo_impl::compute_shortest_path(G, s, t);
  auto &sp = sp_path.graph;

  // P_LO <-- {shortest path p_0(s, t)};
  resPaths.push_back(sp_path);

  // If we need the shortest path only
  if (k == 1) {
    return resPaths;
  }

  // Every max-heap H_i is associated with p_i
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;
  using Priority = std::pair<Edge, int>;
  using EdgePriorityQueue =
      std::priority_queue<Priority, std::vector<Priority>,
                          kspwlo_impl::EdgePriorityComparator<Edge>>;
  auto edge_priorities = std::vector<EdgePriorityQueue>(k);

  // We keep a set of non-removable edges
  auto dnr_edges = std::unordered_set<Edge, hash<Edge>>{};

  // We keep a set of deleted-edges
  auto deleted_edges = std::unordered_set<Edge, hash<Edge>>{};

  // Initialize max-heap H_0 with the priority of each edge of the shortest path
  for (auto it = edges(sp).first; it != edges(sp).second; ++it) {
    auto prio_e_i =
        kspwlo_impl::compute_priority(G, *it, heuristic, deleted_edges);
  }
}
} // namespace boost

#endif