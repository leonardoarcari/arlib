#ifndef BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_HPP
#define BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/details/penalty_impl.hpp>
#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @brief Algorithms and utilities for Boost::Graph
 */
namespace arlib {

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
template <
    typename Graph, typename WeightMap, typename MultiPredecessorMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
void penalty_ag(
    const Graph &G, WeightMap const &original_weight,
    MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k, double theta,
    double p, double r, int max_nb_updates, int max_nb_steps,
    shortest_path_algorithm algorithm = shortest_path_algorithm::dijkstra) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  using Length = typename boost::property_traits<typename boost::property_map<
      Graph, boost::edge_weight_t>::type>::value_type;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<WeightMap, Edge>));

  // P_LO set of k paths
  auto resPathsEdges = std::vector<std::vector<Edge>>{};
  auto resEdges = std::vector<std::unordered_set<Edge, boost::hash<Edge>>>{};

  // Make a local weight map to avoid modifying existing graph.
  auto penalty = details::penalty_functor{original_weight};

  // Make shortest path algorithm function
  auto compute_shortest_path =
      details::build_shortest_path_fn(algorithm, G, original_weight);

  // Compute shortest path from s to t
  auto distance_s = std::vector<Length>(num_vertices(G));
  auto distance_t = std::vector<Length>(num_vertices(G));
  auto sp =
      details::dijkstra_shortest_path_two_ways(G, s, t, distance_s, distance_t);
  assert(sp);

  // P_LO <-- {shortest path p_0(s, t)};
  resPathsEdges.push_back(*sp);
  resEdges.emplace_back(sp->begin(), sp->end());

  // If we need the shortest path only
  if (k == 1) {
    details::fill_multi_predecessor(resPathsEdges.begin(), resPathsEdges.end(),
                                    G, predecessors);
    return;
  }

  // Initialize map for penalty bounds
  auto penalty_bounds = std::unordered_map<Edge, int, boost::hash<Edge>>{};

  // Penalize sp edges
  details::penalize_candidate_path(*sp, G, s, t, p, r, penalty, distance_s,
                                   distance_t, penalty_bounds, max_nb_updates);

  int step = 0;
  using Index = std::size_t;
  while (resPathsEdges.size() < static_cast<Index>(k) && step < max_nb_steps) {
    auto p_tmp = compute_shortest_path(G, s, t, penalty);

    // Penalize p_tmp edges
    details::penalize_candidate_path(*p_tmp, G, s, t, p, r, penalty, distance_s,
                                     distance_t, penalty_bounds,
                                     max_nb_updates);
    ++step;

    // If p_tmp is sufficiently dissimilar to other alternative paths, accept it
    bool is_valid_path = true;
    for (const auto &alt_path : resEdges) {
      if (details::compute_similarity(*p_tmp, alt_path, original_weight) >
          theta) {
        is_valid_path = false;
        break;
      }
    }

    if (is_valid_path) {
      resPathsEdges.push_back(*p_tmp);
      resEdges.emplace_back(p_tmp->begin(), p_tmp->end());
    }
  }

  // Beforer returning, populate predecessors map
  details::fill_multi_predecessor(resPathsEdges.begin(), resPathsEdges.end(), G,
                                  predecessors);
}

template <typename PropertyGraph, typename MultiPredecessorMap,
          typename Vertex =
              typename boost::graph_traits<PropertyGraph>::vertex_descriptor>
void penalty_ag(
    const PropertyGraph &G, MultiPredecessorMap &predecessors, Vertex s,
    Vertex t, int k, double theta, double p, double r, int max_nb_updates,
    int max_nb_steps,
    shortest_path_algorithm algorithm = shortest_path_algorithm::dijkstra) {
  using namespace boost;
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;

  BOOST_CONCEPT_ASSERT(
      (PropertyGraphConcept<PropertyGraph, Edge, edge_weight_t>));

  auto weight = get(edge_weight, G);
  penalty_ag(G, weight, predecessors, s, t, k, theta, p, r, max_nb_updates,
             max_nb_steps, algorithm);
}
} // namespace arlib

#endif