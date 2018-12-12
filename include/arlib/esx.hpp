/**
 * @file esx.hpp
 * @author Leonardo Arcari (leonardo1.arcari@gmail.com)
 * @version 1.0.0
 * @date 2018-10-28
 *
 * @copyright Copyright (c) 2018 Leonardo Arcari
 *
 * MIT License
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

#ifndef BOOST_ESX_ALTERNATIVE_ROUTING_HPP
#define BOOST_ESX_ALTERNATIVE_ROUTING_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/terminators.hpp>
#include <arlib/type_traits.hpp>

#include <arlib/details/esx_impl.hpp>

#include <queue>
#include <unordered_set>
#include <vector>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
/**
 * An implementation of ESX k-shortest path with limited overlap for
 * Boost::Graph.
 *
 * This implementation refers to the following publication:
 *
 * Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper and Ulf Leser,
 * Exact and Approximate Algorithms for Finding k-Shortest Paths with Limited
 * Overlap , In Proc. of the 20th Int. Conf. on Extending Database Technology
 * (EDBT) (2017)
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
 * @param weight The weight map of @p G.
 * @param predecessors The multi predecessor map of @p G.
 * @param s The source node.
 * @param t The target node.
 * @param k The number of alternative paths to compute.
 * @param theta The similarity threshold.
 * @param algorithm The routing kernel to employ. The default is
 *        routing_kernels::astar
 *
 */
template <typename Graph, typename WeightMap, typename MultiPredecessorMap,
          typename Terminator = arlib::always_continue,
          typename Vertex = vertex_of_t<Graph>,
          typename = std::enable_if_t<std::is_same_v<
              typename boost::property_traits<MultiPredecessorMap>::key_type,
              Vertex>>>
void esx(const Graph &G, WeightMap const &weight,
         MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k,
         double theta, routing_kernels algorithm = routing_kernels::astar,
         Terminator &&terminator = Terminator{}) {
  details::esx_dispatch(G, weight, predecessors, s, t, k, theta, algorithm,
                        std::forward<Terminator>(terminator));
}

template <typename Graph, typename WeightMap, typename MultiPredecessorMap,
          typename EdgeCentralityMap,
          typename Terminator = arlib::always_continue,
          typename Vertex = vertex_of_t<Graph>,
          typename = std::enable_if_t<std::is_same_v<
              typename boost::property_traits<MultiPredecessorMap>::key_type,
              Vertex>>>
void esx(const Graph &G, WeightMap const &weight,
         MultiPredecessorMap &predecessors,
         EdgeCentralityMap const &edge_centrality, Vertex s, Vertex t, int k,
         double theta, routing_kernels algorithm = routing_kernels::astar,
         Terminator &&terminator = Terminator{}) {
  details::esx_dispatch(G, weight, predecessors, edge_centrality, s, t, k,
                        theta, algorithm, std::forward<Terminator>(terminator));
}

/**
 * An implementation of `ESX` k-shortest path with limited overlap for
 * `Boost::Graph`.
 *
 * This overload takes an input graph modeling `PropertyGraph` concept having at
 * least one edge property with tag `boost::edge_weight_t`. Moreover it does not
 * require an explicit `WeightMap` parameter, because it is directly gathered
 * from the `PropertyGraph`.
 *
 * @see esx(const Graph &G, WeightMap const &weight,
 *          MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k,
 *          double theta,
 *          routing_kernels algorithm)
 *
 * @tparam PropertyGraph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 *
 */
template <typename PropertyGraph, typename MultiPredecessorMap,
          typename Terminator = arlib::always_continue,
          typename Vertex = vertex_of_t<PropertyGraph>,
          typename = std::enable_if_t<std::is_same_v<
              typename boost::property_traits<MultiPredecessorMap>::key_type,
              Vertex>>>
void esx(const PropertyGraph &G, MultiPredecessorMap &predecessors, Vertex s,
         Vertex t, int k, double theta,
         routing_kernels algorithm = routing_kernels::astar,
         Terminator &&terminator = Terminator{}) {
  using namespace boost;
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;

  BOOST_CONCEPT_ASSERT(
      (PropertyGraphConcept<PropertyGraph, Edge, edge_weight_t>));

  auto weight = get(edge_weight, G);
  details::esx_dispatch(G, weight, predecessors, s, t, k, theta, algorithm,
                        std::forward<Terminator>(terminator));
}

template <typename PropertyGraph, typename MultiPredecessorMap,
          typename EdgeCentralityMap,
          typename Terminator = arlib::always_continue,
          typename Vertex = vertex_of_t<PropertyGraph>,
          typename = std::enable_if_t<std::is_same_v<
              typename boost::property_traits<MultiPredecessorMap>::key_type,
              Vertex>>>
void esx(const PropertyGraph &G, MultiPredecessorMap &predecessors,
         EdgeCentralityMap const &edge_centrality, Vertex s, Vertex t, int k,
         double theta, routing_kernels algorithm = routing_kernels::astar,
         Terminator &&terminator = Terminator{}) {
  using namespace boost;
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;

  BOOST_CONCEPT_ASSERT(
      (PropertyGraphConcept<PropertyGraph, Edge, edge_weight_t>));

  auto weight = get(edge_weight, G);
  esx(G, weight, predecessors, edge_centrality, s, t, k, theta, algorithm,
      std::forward<Terminator>(terminator));
}
} // namespace arlib

#endif
