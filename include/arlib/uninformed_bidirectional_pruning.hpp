/**
 * @file uninformed_bidirectional_pruning.hpp
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

#ifndef BOOST_GRAPH_PRUNING_ALGORITHMS_HPP
#define BOOST_GRAPH_PRUNING_ALGORITHMS_HPP

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/details/arlib_utils.hpp>
#include <arlib/routing_kernels/bidirectional_dijkstra.hpp>
#include <arlib/routing_kernels/visitor.hpp>
#include <arlib/type_traits.hpp>

#include <arlib/details/ubp_impl.hpp>

#include <limits>
#include <unordered_set>
#include <vector>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
template <typename Graph>
using PrunedGraph =
    boost::filtered_graph<Graph, details::pruned_edges<edge_of_t<Graph>>>;
/**
 * An implementation of Uninformed Bidirectional Pruning for Boost::Graph.
 *
 * This implementation refers to the following publication:
 *
 * Andreas Paraskevopoulos, Christos Zaroliagis. Improved Alternative Route
 * Planning. Daniele Frigioni and Sebastian Stiller. ATMOS - 13th Workshop on
 * Algorithmic Approaches for Transportation Modelling, Optimization, and
 * Systems - 2013, Sep 2013, Sophia Antipolis, France. Schloss
 * Dagstuhl–Leibniz-Zentrum fuer Informatik, 33, pp.108–122, 2013, OpenAccess
 * Series in Informatics (OASIcs).
 *
 * @tparam Graph A Boost::VertexAndEdgeListGraph
 * @tparam WeightMap The weight or "length" of each edge in the graph. The
 *         weights must all be non-negative, and the algorithm will throw a
 *         negative_edge exception is one of the edges is negative. The type
 *         WeightMap must be a model of Readable Property Map. The edge
 *         descriptor type of the graph needs to be usable as the key type for
 *         the weight map. The value type for this map must be the same as the
 *         value type of the distance map.
 * @tparam RevWeightMap Same as `WeightMap`, but for
 *         `boost::reverse_graph<Graph>`
 * @param G The input graph.
 * @param weight_f The weight map of `G`.
 * @param rev_G A `boost::reverse_graph` of `G`
 * @param weight_b The weight map of `rev_G`.
 * @param s The source node.
 * @param t The target node.
 * @param tau The pruning factor.
 * @return A pruned copy of `G`.
 */
template <typename Graph, typename WeightMap, typename RevWeightMap,
          typename Vertex = vertex_of_t<Graph>>
PrunedGraph<Graph>
uninformed_bidirectional_pruner(const Graph &G, WeightMap const &weight_f,
                                boost::reverse_graph<Graph> const &rev_G,
                                RevWeightMap const &weight_b, Vertex s,
                                Vertex t, double tau) {
  using namespace boost;
  using Length = typename boost::property_traits<WeightMap>::value_type;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  using RevEdge =
      typename graph_traits<boost::reverse_graph<Graph>>::edge_descriptor;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT(
      (VertexAndEdgeListGraphConcept<boost::reverse_graph<Graph>>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<WeightMap, Edge>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<RevWeightMap, RevEdge>));

  // Instantiate data structures
  auto distance_f_vec = std::vector<Length>(num_vertices(G), 6);
  auto predecessor_f_vec =
      std::vector<Vertex>(vertices(G).first, vertices(G).second);
  auto vertex_id = get(vertex_index, G);
  auto distance_f =
      make_iterator_property_map(std::begin(distance_f_vec), vertex_id);
  auto predecessor_f =
      make_iterator_property_map(std::begin(predecessor_f_vec), vertex_id);

  // Reverse tree
  auto vertex_id_b = get(vertex_index, G);
  auto distance_b_vec = std::vector<Length>(num_vertices(rev_G));
  auto predecessor_b_vec =
      std::vector<Vertex>(vertices(rev_G).first, vertices(rev_G).second);
  auto predecessor_b =
      make_iterator_property_map(std::begin(predecessor_b_vec), vertex_id_b);

  auto distance_b =
      make_iterator_property_map(std::begin(distance_b_vec), vertex_id_b);

  // Pruning visitor
  auto pruning_visitor = UninformedBiPrunerVisitor<Vertex>{tau};

  bidirectional_dijkstra(G, s, t, predecessor_f, distance_f, weight_f, rev_G,
                         predecessor_b, distance_b, weight_b, pruning_visitor);

  // auto pruned_G = Graph{G};
  auto prd_edges = std::unordered_set<Edge, boost::hash<Edge>>{};

  auto sp = details::build_edge_list_from_dijkstra(G, s, t, predecessor_f);
  auto final_distance =
      details::compute_length_from_edges(sp.begin(), sp.end(), weight_f);
  for (auto [v_it, v_end] = pruning_visitor.get_pruned_vertices();
       v_it != v_end; ++v_it) {
    bool should_prune =
        details::pruning_policy(s, t, *v_it, predecessor_f, predecessor_b,
                                distance_f, distance_b, tau, final_distance);
    if (should_prune) {
      for (auto [adj_it, adj_end] = adjacent_vertices(*v_it, G);
           adj_it != adj_end; ++adj_it) {
        auto u = *adj_it;
        auto v = *v_it;

        auto [uv_e, uv_exists] = edge(u, v, G);
        if (uv_exists) {
          prd_edges.insert(uv_e);
        }

        auto [vu_e, vu_exists] = edge(v, u, G);
        if (vu_exists) {
          prd_edges.insert(vu_e);
        }
      }
    }
  }

  const auto pruned_G =
      filtered_graph(G, details::pruned_edges{std::move(prd_edges)});
  return pruned_G;
}

/**
 * An implementation of Uninformed Bidirectional Pruning for Boost::Graph.
 *
 * This overload takes an input graph modeling `PropertyGraph` concept having at
 * least one edge property with tag `boost::edge_weight_t`. Moreover it does not
 * require an explicit `WeightMap` parameter, because it is directly gathered
 * from the `PropertyGraph`.
 *
 * @see uninformed_bidirectional_pruner(const Graph &G, WeightMap const
 *                                      &weight_f, boost::reverse_graph<Graph>
 *                                      const &rev_G, RevWeightMap const
 *                                      &weight_b, Vertex s, Vertex t, double
 *                                      tau)
 *
 * @tparam PropertyGraph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 *
 * @return A pruned copy of `G`.
 */
template <typename PropertyGraph, typename Vertex = vertex_of_t<PropertyGraph>>
PrunedGraph<PropertyGraph>
uninformed_bidirectional_pruner(const PropertyGraph &G, Vertex s, Vertex t,
                                double tau) {
  using namespace boost;
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;

  BOOST_CONCEPT_ASSERT(
      (PropertyGraphConcept<PropertyGraph, Edge, edge_weight_t>));

  auto weight = get(edge_weight, G);
  auto rev = make_reverse_graph(G);
  auto weight_b = get(edge_weight, rev);
  return uninformed_bidirectional_pruner(G, weight, rev, weight_b, s, t, tau);
}
} // namespace arlib

#endif
