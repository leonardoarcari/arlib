#ifndef BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_HPP
#define BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/graph_types.hpp"
#include "kspwlo/impl/penalty_impl.hpp"

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @brief Algorithms and utilities for Boost::Graph
 */
namespace boost {

/**
 * @brief An implementation of Penalty method to compute alternative routes for
 * Boost::Graph.
 *
 * This implementation refers to the following publication:
 * Andreas Paraskevopoulos, Christos Zaroliagis. Improved Alternative Route
 * Planning. Daniele Frigioni and Sebastian Stiller. ATMOS - 13th Workshop on
 * Algorithmic Approaches for Transportation Modelling, Optimization, and
 * Systems - 2013, Sep 2013, Sophia Antipolis, France.
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam Vertex A vertex_descriptor of PropertyGraph.
 * @param G The graph.
 * @param s The source node.
 * @param t The target node.
 * @param k The number of alternative paths to compute
 * @param theta The similarity threshold. A candidate path is accepted as
 *              alternative route iff its similarity wrt all the other
 *              alternative paths is less than @p theta
 * @param p The penalty factor for edges in the candidate path.
 * @param r The penalty factor for edges incoming and outgoing to/from vertices
 *          of the candidate path.
 * @param max_nb_updates The maximum number of times an edge can be penalized.
 * @param max_nb_steps The maximum number of steps of the algorithm (timeout).
 *
 * @return A vector of at maximum @p k alternative paths.
 */
template <typename Graph, typename Vertex = typename boost::graph_traits<
                              Graph>::vertex_descriptor>
std::vector<kspwlo::Path<Graph>>
penalty_ag(const Graph &G, Vertex s, Vertex t, int k, double theta, double p,
           double r, int max_nb_updates, int max_nb_steps) {
  // P_LO set of k paths
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  auto resPaths = std::vector<kspwlo::Path<Graph>>{};

  // Make a local weight map to avoid modifying existing graph.
  auto original_weight = get(edge_weight, G);
  auto [edge_it, edge_last] = edges(G);
  auto penalty =
      kspwlo_impl::penalty_functor{original_weight, edge_it, edge_last};

  // Compute shortest path from s to t
  auto distance_s = std::vector<kspwlo::Length>(num_vertices(G));
  auto distance_t = std::vector<kspwlo::Length>(num_vertices(G));
  auto sp = kspwlo_impl::dijkstra_shortest_path_two_ways(G, s, t, distance_s,
                                                         distance_t);
  assert(sp);

  // P_LO <-- {shortest path p_0(s, t)};
  resPaths.emplace_back(build_graph_from_edges(*sp, G),
                        kspwlo_impl::compute_length_from_edges(*sp, G));

  // If we need the shortest path only
  if (k == 1) {
    return resPaths;
  }

  // Initialize map for penalty bounds
  auto penalty_bounds = std::unordered_map<Edge, int, boost::hash<Edge>>{};

  // Penalize sp edges
  kspwlo_impl::penalize_candidate_path(*sp, G, s, t, p, r, penalty, distance_s,
                                       distance_t, penalty_bounds,
                                       max_nb_updates);

  int step = 0;
  using Index = std::size_t;
  while (resPaths.size() < static_cast<Index>(k) && step < max_nb_steps) {
    auto p_tmp = kspwlo_impl::dijkstra_shortest_path(G, s, t, penalty);

    // Penalize p_tmp edges
    kspwlo_impl::penalize_candidate_path(*p_tmp, G, s, t, p, r, penalty,
                                         distance_s, distance_t, penalty_bounds,
                                         max_nb_updates);
    ++step;

    // If p_tmp is sufficiently dissimilar to other alternative paths, accept it
    bool is_valid_path = true;
    for (const auto &alt_path : resPaths) {
      if (kspwlo_impl::compute_similarity(*p_tmp, alt_path) > theta) {
        is_valid_path = false;
        break;
      }
    }

    if (is_valid_path) {
      resPaths.emplace_back(build_graph_from_edges(*p_tmp, G),
                            kspwlo_impl::compute_length_from_edges(*p_tmp, G));
    }
  }

  return resPaths;
}
} // namespace boost

#endif