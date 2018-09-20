#ifndef BOOST_EDGE_SUBSET_EXCLUSION_HPP
#define BOOST_EDGE_SUBSET_EXCLUSION_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/graph_types.hpp"
#include <kspwlo/graph_utils.hpp>
#include "kspwlo/impl/esx_impl.hpp"

#include <queue>
#include <unordered_set>
#include <vector>

/**
 * @brief Algorithms and utilities for Boost::Graph
 */
namespace boost {
/**
 * @brief An implementation of ESX k-shortest path with limited overlap for
 *        Boost::Graph.
 *
 * This implementation refers to the following publication:
 * Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper and Ulf Leser,
 * Exact and Approximate Algorithms for Finding k-Shortest Paths with Limited
 * Overlap , In Proc. of the 20th Int. Conf. on Extending Database Technology
 * (EDBT) (2017)
 *
 * @tparam PropertyGraph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam Vertex A vertex of PropertyGraph.
 * @param G The graph.
 * @param s The source node.
 * @param t The target node.
 * @param k The number of alternative paths to compute.
 * @param theta The similarity threshold.
 *
 * @return A list of at maximum @p k alternative paths.
 */
template <typename PropertyGraph,
    typename Vertex =
    typename boost::graph_traits<PropertyGraph>::vertex_descriptor>
std::vector<kspwlo::Path<PropertyGraph>>
esx(const PropertyGraph &G, Vertex s, Vertex t, int k, double theta,
    kspwlo::shortest_path_algorithm algorithm =
    kspwlo::shortest_path_algorithm::astar) {
  // P_LO set of k paths
  using Length = typename boost::property_traits<typename boost::property_map<
      PropertyGraph, boost::edge_weight_t>::type>::value_type;
  auto resPaths = std::vector<kspwlo::Path<PropertyGraph>>{};

  // Compute shortest path from s to t
  auto sp_path = kspwlo_impl::compute_shortest_path(G, s, t);
  auto &sp = sp_path.graph();

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

  // Make shortest path algorithm function
  auto compute_shortest_path = kspwlo_impl::build_shortest_path_fn(
      algorithm, G, s, t, heuristic, deleted_edges);

  // Initialize max-heap H_0 with the priority of each edge of the shortest path
  kspwlo_impl::init_edge_priorities(sp, edge_priorities, 0, G, deleted_edges);

  auto overlaps = std::vector<double>(k, 0.0);
  overlaps[0] = 1.0; // Set p_c overlap with sp (itself) to 1

  bool still_feasible = true;
  while (resPaths.size() < static_cast<std::size_t>(k) && still_feasible) {
    std::ptrdiff_t p_max_idx = 0;
    double overlap_ratio = 1.0;

    while (overlap_ratio > theta) {
      // Get the max overlapping path
      auto max_it = std::max_element(std::begin(overlaps), std::end(overlaps));
      p_max_idx = max_it - std::begin(overlaps);
      overlap_ratio = *max_it;

      // Check if finding a result is feasible
      still_feasible = kspwlo_impl::check_feasibility(overlaps);
      if (!still_feasible) {
        break; // Stop the algorithm
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
      auto p_tmp = compute_shortest_path(G, s, t, heuristic, deleted_edges);

      // If shortest path did not find a path
      if (!p_tmp) {
        kspwlo_impl::move_to_dnr(e_tmp, deleted_edges, dnr_edges);
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
      bool candidate_is_valid =
          kspwlo_impl::check_candidate_validity(*p_tmp, resPaths, theta);
      if (candidate_is_valid) {
        // Add p_tmp to P_LO
        resPaths.emplace_back(
            build_graph_from_edges(*p_tmp, G),
            kspwlo_impl::compute_length_from_edges(*p_tmp, G));

        // Set p_c overlap with itself to 1
        std::ptrdiff_t p_c_idx = resPaths.size() - 1;
        overlaps[p_c_idx] = 1.0;

        // Initialize max-heap H_i with the priority of each edge of new
        // alternative path
        kspwlo_impl::init_edge_priorities(*p_tmp, edge_priorities, p_c_idx, G,
                                          deleted_edges);
        break; // From inner while loop
      }
    }
  }

  return resPaths;
}
} // namespace boost

#endif