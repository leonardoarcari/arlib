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

  // Compute lower bounds for AStar
  auto heuristic = kspwlo_impl::distance_heuristic<PropertyGraph, Length>(G, t);

  // Initialize max-heap H_0 with the priority of each edge of the shortest path
  for (auto it = edges(sp).first; it != edges(sp).second; ++it) {
    auto u = source(*it, sp);
    auto v = target(*it, sp);
    auto edge_in_G = edge(u, v, G).first;
    auto prio_e_i =
        kspwlo_impl::compute_priority(G, edge_in_G, heuristic, deleted_edges);
    edge_priorities[0].push(std::make_pair(edge_in_G, prio_e_i));
  }

  auto overlaps = std::vector<double>(k, 0.0);
  using PathIndex = typename decltype(overlaps)::size_type;
  overlaps[0] = 1.0; // Set p_c overlap with sp (itself) to 1

  bool still_feasible = true;
  while (resPaths.size() < static_cast<PathIndex>(k) && still_feasible) {
    PathIndex p_max_idx = 0;
    double overlap_ratio = 1.0;

    while (overlap_ratio > theta) {
      // Get the max overlapping path
      auto max_it = std::max_element(std::begin(overlaps), std::end(overlaps));
      p_max_idx = max_it - std::begin(overlaps);
      overlap_ratio = *max_it;

      // Check if finding a result is feasible
      auto valid_overlapping =
          std::find_if(std::begin(overlaps), std::end(overlaps),
                       [](const auto &v) { return v > 0; });
      if (valid_overlapping == std::end(overlaps)) {
        still_feasible = false;
        break;
      }

      // Get e_tmp with higher priority from H_{p_max_idx}
      auto e_tmp = edge_priorities[p_max_idx].top().first;

      // If edge is in DO-NOT-REMOVE edges set, continue
      if (dnr_edges.find(e_tmp) != std::end(dnr_edges)) {
        edge_priorities[p_max_idx].pop();
        // Also, if H_{p_max_idx} queue is empty set its overlapping value to 0
        if (edge_priorities[p_max_idx].empty()) {
          overlaps[p_max_idx] = 0;
        }
        continue;
      } else {
        // Otherwise, add e_tmp to deleted edges
        deleted_edges.insert(e_tmp);
      }

      // Compute p_tmp shortest path
      auto p_tmp = kspwlo_impl::astar_shortest_path(G, s, t, heuristic,
                                                            deleted_edges);

      // If astar shortest path did not find a path
      if (!p_tmp) {
        auto old_size = deleted_edges.size();
        deleted_edges.erase(e_tmp); // Reinsert e_tmp into G
        assert(deleted_edges.size() + 1 == old_size);
        dnr_edges.insert(e_tmp); // Mark e_tmp as do_not_remove
        continue;
      }

      // Remove e_tmp from H_{p_max_idx}
      edge_priorities[p_max_idx].pop();

      // in cases there are no more edges to remove we set overlap to zero to
      // avoid choosing from the same path again. A path the overlap of which is
      // zero can never be chosen to remove its edges.
      if (edge_priorities[p_max_idx].empty()) {
        overlaps[p_max_idx] = 0;
      } else {
        overlaps[p_max_idx] =
            kspwlo_impl::compute_similarity(*p_tmp, resPaths[p_max_idx]);
      }

      // Checking if the resulting path is valid
      bool candidate_is_valid = true;
      for (const auto &alt_path : resPaths) {
        if (kspwlo_impl::compute_similarity(*p_tmp, alt_path) > theta) {
          candidate_is_valid = false;
          break;
        }
      }

      if (candidate_is_valid) {
        // Add p_tmp to P_LO
        resPaths.emplace_back(
            build_graph_from_edges(*p_tmp, G),
            kspwlo_impl::compute_length_from_edges(*p_tmp, G));

        // Set p_c overlap with itself to 1
        PathIndex p_c_idx = resPaths.size() - 1;
        overlaps[p_c_idx] = 1.0;

        for (const auto & [ u, v ] : *p_tmp) {
          auto edge_in_G = edge(u, v, G).first;
          auto prio_e_i = kspwlo_impl::compute_priority(G, edge_in_G, heuristic,
                                                        deleted_edges);
          edge_priorities[p_c_idx].push(std::make_pair(edge_in_G, prio_e_i));
        }
        break; // From inner while loop
      }
    }
  }

  return resPaths;
}
} // namespace boost

#endif