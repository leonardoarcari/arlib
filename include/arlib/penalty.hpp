/**
 * @file penalty.hpp
 * @author Leonardo Arcari (leonardo1.arcari@gmail.com)
 * @version 1.0.0
 * @date 2018-10-28
 *
 * @copyright Copyright (c) 2018 Leonardo Arcari
 *
 * MIT Licence
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_HPP
#define BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/details/penalty_impl.hpp>
#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>
#include <arlib/type_traits.hpp>

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {

/**
 * An implementation of Penalty method to compute alternative routes for
 * Boost::Graph.
 *
 * This implementation refers to the following publication:
 * Andreas Paraskevopoulos, Christos Zaroliagis. Improved Alternative Route
 * Planning. Daniele Frigioni and Sebastian Stiller. ATMOS - 13th Workshop on
 * Algorithmic Approaches for Transportation Modelling, Optimization, and
 * Systems - 2013, Sep 2013, Sophia Antipolis, France.
 *
 * @tparam Graph A Boost::VertexAndEdgeListGraph
 * @tparam WeightMap The weight or "length" of each edge in the graph. The
 *         weights must all be non-negative, and the algorithm will throw a
 *         negative_edge exception is one of the edges is negative. The type
 *         WeightMap must be a model of Readable Property Map. The edge
 *         descriptor type of the graph needs to be usable as the key type for
 *         the weight map. The value type for this map must be the same as the
 *         value type of the distance map.
 * @tparam MultiPredecessorMap The multi predecessor map records the edges in
 *         the alternative paths from @p s to @p t. Upon completion of the
 *         algorithm, for any vertex `v` on any alternative path `p`, multi
 *         predecessor map stores the predecessor of node `v` on path `p`. If no
 *         predecessor for a node `v'` is reported, then `v'` is not part of any
 *         alternative path or it is the source node @p s. The type of
 *         `MultiPredecessorMap` must be a model of `Read Property Map`. The
 *         vertex descriptor of the input graph @p G must be usable as a key
 *         type for the multi predecessor map. Whereas the value type must
 *         satisfy the `UnorderedAssociativeContainer` concept, where its key is
 *         an `int`, the index/number of alternative path for which it exists a
 *         predecessor of `v`, and where the value type is a vertex descriptor
 *         of input graph @p G.
 * @tparam Vertex The vertex descriptor.
 * @param G The input graph.
 * @param original_weight The weight map of @p G.
 * @param predecessors The multi predecessor map of @p G.
 * @param s The source node.
 * @param t The target node.
 * @param k The number of alternative paths to compute.
 * @param theta The similarity threshold.
 * @param p The penalty factor for edges in the candidate path.
 * @param r The penalty factor for edges incoming and outgoing to/from vertices
 *          of the candidate path.
 * @param max_nb_updates The maximum number of times an edge can be penalized.
 * @param max_nb_steps The maximum number of steps of the algorithm (timeout).
 * @param algorithm The routing kernel to employ. The default is
 *        routing_kernels::dijkstra
 *
 */
template <typename Graph, typename WeightMap, typename MultiPredecessorMap,
          typename Vertex = vertex_of_t<Graph>>
void penalty(const Graph &G, WeightMap const &original_weight,
             MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k,
             double theta, double p, double r, int max_nb_updates,
             int max_nb_steps,
             routing_kernels algorithm = routing_kernels::dijkstra) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  using Length = typename boost::property_traits<typename boost::property_map<
      Graph, boost::edge_weight_t>::type>::value_type;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<WeightMap, Edge>));

  // P_LO set of k paths
  auto resPathsEdges = std::vector<std::vector<Edge>>{};
  auto resEdges = std::vector<std::unordered_set<Edge, boost::hash<Edge>>>{};

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<WeightMap, Edge>));

  // P_LO set of k paths
  auto resPaths = std::vector<Path<Graph>>{};
  // Make a local weight map to avoid modifying existing graph.
  auto pen_fctor = details::penalty_functor{original_weight};

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
  details::penalize_candidate_path(*sp, G, s, t, p, r, pen_fctor, distance_s,
                                   distance_t, penalty_bounds, max_nb_updates);

  int step = 0;
  using Index = std::size_t;
  while (resPathsEdges.size() < static_cast<Index>(k) && step < max_nb_steps) {
    auto p_tmp = compute_shortest_path(G, s, t, pen_fctor);

    // Penalize p_tmp edges
    details::penalize_candidate_path(*p_tmp, G, s, t, p, r, pen_fctor,
                                     distance_s, distance_t, penalty_bounds,
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

/**
 * An implementation of Penalty method to compute alternative routes for
 * Boost::Graph.
 *
 * This overload takes an input graph modeling `PropertyGraph` concept having at
 * least one edge property with tag `boost::edge_weight_t`. Moreover it does not
 * require an explicit `WeightMap` parameter, because it is directly gathered
 * from the `PropertyGraph`.
 *
 * @see penalty(const Graph &G, WeightMap const &original_weight,
 *              MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k,
 *              double theta, double p, double r, int max_nb_updates, int
 *              max_nb_steps, routing_kernels algorithm)
 *
 * @tparam PropertyGraph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 */
template <typename PropertyGraph, typename MultiPredecessorMap,
          typename Vertex = vertex_of_t<PropertyGraph>>
void penalty(const PropertyGraph &G, MultiPredecessorMap &predecessors,
             Vertex s, Vertex t, int k, double theta, double p, double r,
             int max_nb_updates, int max_nb_steps,
             routing_kernels algorithm = routing_kernels::dijkstra) {
  using namespace boost;
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;

  BOOST_CONCEPT_ASSERT(
      (PropertyGraphConcept<PropertyGraph, Edge, edge_weight_t>));

  auto weight = get(edge_weight, G);
  penalty(G, weight, predecessors, s, t, k, theta, p, r, max_nb_updates,
          max_nb_steps, algorithm);
}
} // namespace arlib

#endif