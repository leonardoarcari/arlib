/**
 * @file bidirectional_dijkstra.hpp
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

#ifndef BOOST_BIDIRECTIONAL_DIJKSTRA_HPP
#define BOOST_BIDIRECTIONAL_DIJKSTRA_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>

#include <arlib/details/arlib_utils.hpp>
#include <arlib/routing_kernels/details/bidirectional_dijkstra_impl.hpp>
#include <arlib/routing_kernels/visitor.hpp>
#include <arlib/type_traits.hpp>

#include <deque>
#include <limits>
#include <vector>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
//===----------------------------------------------------------------------===//
//                   Bidirectional Dijkstra algorithm
//===----------------------------------------------------------------------===//
/**
 * Implementation of Bidirectional Dijkstra method for Boost::Graph to
 *        compute the shortest path between two vertices.
 *
 * This implementation follows the one from nx.graph library:
 * https://networkx.github.io/documentation/networkx-1.10/_modules/networkx/algorithms/shortest_paths/weighted.html#bidirectional_dijkstra
 *
 * You can find the original paper by Nicholson (1996)
 * https://academic.oup.com/comjnl/article/9/3/275/406281
 *
 * @tparam Graph A Boost::VertexAndEdgeListGraph
 * @tparam PredecessorMap The predecessor map records the edges in the shortest
 *         path tree, the tree computed by the traversal of the graph. Upon
 *         completion of the algorithm, the edges (p[u],u) for all u in V are in
 *         the tree. The shortest path from vertex s to each vertex v in the
 *         graph consists of the vertices v, p[v], p[p[v]], and so on until s is
 *         reached, in reverse order. The tree is not guaranteed to be a minimum
 *         spanning tree. If p[u] = u then u is either the source vertex or a
 *         vertex that is not reachable from the source. The PredecessorMap type
 *         must be a Read/Write Property Map whose key and value types are the
 *         same as the vertex descriptor type of the graph.
 * @tparam DistanceMap The shortest path weight from the source vertex s to each
 *         vertex in the graph g is recorded in this property map. The shortest
 *         path weight is the sum of the edge weights along the shortest path.
 *         The type DistanceMap must be a model of Read/Write Property Map. The
 *         vertex descriptor type of the graph needs to be usable as the key
 *         type of the distance map.
 * @tparam WeightMap The weight or ``length'' of each edge in the graph. The
 *         weights must all be non-negative, and the algorithm will throw a
 *         negative_edge exception is one of the edges is negative. The type
 *         WeightMap must be a model of Readable Property Map. The edge
 *         descriptor type of the graph needs to be usable as the key type for
 *         the weight map. The value type for this map must be the same as the
 *         value type of the distance map.
 * @tparam BackGraph A boost::reverse_graph<Graph>
 * @tparam BackPredecessorMap A PredecessorMap of a boost::reverse_graph<Graph>
 * @tparam BackDistanceMap A DistanceMap of a boost::reverse_graph<Graph>
 * @tparam BackWeightMap A WeightMap of a boost::reverse_graph<Graph>
 * @tparam BiDijkstraVisitorImpl An implementation of a BiDijkstraVisitor.
 * @tparam Vertex a vertex_descriptor.
 * @param G The graph.
 * @param s The source vertex.
 * @param t The target vertex.
 * @param predecessor The PredecessorMap for the forward step.
 * @param distance The DistanceMap for the forward step.
 * @param weight The WeightMap for the forward step.
 * @param G_b A boost::reverse_graph<Graph> of @p G.
 * @param predecessor_b The PredecessorMap for the backward step.
 * @param distance_b The DistanceMap for the backward step.
 * @param weight_b The WeightMap for the backward step.
 * @param visitor An implementation of a BiDijkstraVisitor.
 */
template <typename Graph, typename PredecessorMap, typename DistanceMap,
          typename WeightMap, typename BackGraph, typename BackPredecessorMap,
          typename BackDistanceMap, typename BackWeightMap,
          typename BiDijkstraVisitorImpl, typename Vertex = vertex_of_t<Graph>>
void bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
                            PredecessorMap predecessor, DistanceMap distance,
                            WeightMap weight, const BackGraph &G_b,
                            BackPredecessorMap predecessor_b,
                            BackDistanceMap distance_b, BackWeightMap weight_b,
                            BiDijkstraVisitor<BiDijkstraVisitorImpl> &visitor) {
  using namespace boost;
  using Length = typename property_traits<DistanceMap>::value_type;
  using Edge = edge_of_t<Graph>;
  using RevEdge = edge_of_t<boost::reverse_graph<Graph>>;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<BackGraph>));
  BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<WeightMap, Edge>));
  BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<BackWeightMap, RevEdge>));

  constexpr Length inf = std::numeric_limits<Length>::max();

  // Initialize distance structures
  details::init_distance_vector(G, distance);
  details::init_distance_vector(G_b, distance_b);

  // Initialize shortest path trees (forward and backward)
  predecessor[s] = s;
  predecessor_b[t] = t;

  // Init fringe structures
  auto fringe = details::Fringe<Vertex, Length>{};
  auto fringe_b = details::Fringe<Vertex, Length>{};
  fringe.push(std::make_pair(s, 0));
  fringe_b.push(std::make_pair(t, 0));

  // Init distances to nodes seen
  auto seen = details::Seen<Vertex, Length>{};
  auto seen_b = details::Seen<Vertex, Length>{};
  seen.insert({s, 0});
  seen_b.insert({t, 0});

  using details::BiDijkStepRes;
  using details::Direction;

  Length final_distance = inf;
  auto final_path = std::deque<Vertex>{};
  auto direction = Direction::backward;
  while (!fringe.empty() && !fringe_b.empty()) {
    direction = details::switch_direction(direction);
    auto result = BiDijkStepRes::next;

    // Run a step
    if (direction == Direction::forward) {
      result = details::bi_dijkstra_step(
          G, s, t, predecessor, distance, weight, fringe, seen, predecessor_b,
          distance_b, fringe_b, seen_b, direction, final_distance, final_path,
          visitor);
    } else {
      result = details::bi_dijkstra_step(
          G_b, s, t, predecessor_b, distance_b, weight_b, fringe_b, seen_b,
          predecessor, distance, fringe, seen, direction, final_distance,
          final_path, visitor);
    }

    // Handle the outcome of this step
    switch (result) {
    case BiDijkStepRes::next:
      // Go next
      break;
    case BiDijkStepRes::end:
      // Fill predecessor map
      details::fill_predecessor(predecessor, final_path);
      return;
    case BiDijkStepRes::negative_weights:
      throw std::domain_error{"Contradictory paths found: negative weights?"};
    default:
      // Unexpected result
      throw std::invalid_argument{"Unexpected result. Exiting..."};
    }
  }

  if (fringe.empty() || fringe_b.empty()) {
    if (final_path.empty()) {
      throw details::target_not_found{"No path found!"};
    } else {
      // Clean exit
      details::fill_predecessor(predecessor, final_path);
      return;
    }
  } else {
    // Otherwise
    throw details::target_not_found{"No path found!"};
  }
}

/**
 * Implementation of Bidirectional Dijkstra method for Boost::Graph to
 *        compute the shortest path between two vertices.
 *
 * This overload does not need a PredecessorMap nor a DistanceMap for the
 * backward step.
 *
 * @see bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
 *                             PredecessorMap predecessor, DistanceMap distance,
 *                             WeightMap weight, const BackGraph &G_b,
 *                             BackPredecessorMap predecessor_b,
 *                             BackDistanceMap distance_b,
 *                             BackWeightMap weight_b,
 *                             BiDijkstraVisitor<BiDijkstraVisitorImpl>
 *                                 &visitor)
 *
 * @tparam BackIndexMap This maps each vertex to an integer in the range [0,
 *         num_vertices(g)). This is necessary for efficient updates of the heap
 *         data structure [61] when an edge is relaxed. The type VertexIndexMap
 *         must be a model of Readable Property Map. The value type of the map
 *         must be an integer type. The vertex descriptor type of the graph
 *         needs to be usable as the key type of the map.
 */
template <typename Graph, typename PredecessorMap, typename DistanceMap,
          typename WeightMap, typename BackGraph, typename BackWeightMap,
          typename BackIndexMap, typename BiDijkstraVisitorImpl,
          typename Vertex = vertex_of_t<Graph>>
void bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
                            PredecessorMap predecessor, DistanceMap distance,
                            WeightMap weight, const BackGraph &G_b,
                            BackWeightMap weight_b, BackIndexMap index_map_b,
                            BiDijkstraVisitor<BiDijkstraVisitorImpl> &visitor) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  using RevEdge =
      typename graph_traits<boost::reverse_graph<Graph>>::edge_descriptor;
  using Length = typename property_traits<DistanceMap>::value_type;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<BackGraph>));
  BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<WeightMap, Edge>));
  BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<BackWeightMap, RevEdge>));

  auto predecessor_b_vec =
      std::vector<Vertex>(vertices(G_b).first, vertices(G_b).second);
  auto predecessor_b =
      make_iterator_property_map(std::begin(predecessor_b_vec), index_map_b);

  auto distance_b_vec = std::vector<Length>(num_vertices(G_b));
  auto distance_b =
      make_iterator_property_map(std::begin(distance_b_vec), index_map_b);

  bidirectional_dijkstra(G, s, t, predecessor, distance, weight, G_b,
                         predecessor_b, distance_b, weight_b, visitor);
}

/**
 * Implementation of Bidirectional Dijkstra method for Boost::Graph to
 * compute the shortest path between two vertices.
 *
 * This overload does not need a PredecessorMap nor a DistanceMap for the
 * backward step nor a BiDijkstraVisitor. This simply runs Bidirectional
 * Dijkstra.
 *
 * @see bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
 *                             PredecessorMap predecessor, DistanceMap distance,
 *                             WeightMap weight, const BackGraph &G_b,
 *                             BackPredecessorMap predecessor_b,
 *                             BackDistanceMap distance_b,
 *                             BackWeightMap weight_b,
 *                             BiDijkstraVisitor<BiDijkstraVisitorImpl>
 *                                 &visitor)
 *
 * @tparam BackIndexMap This maps each vertex to an integer in the range [0,
 *         num_vertices(g)). This is necessary for efficient updates of the heap
 *         data structure [61] when an edge is relaxed. The type VertexIndexMap
 *         must be a model of Readable Property Map. The value type of the map
 *         must be an integer type. The vertex descriptor type of the graph
 *         needs to be usable as the key type of the map.
 */
template <typename Graph, typename PredecessorMap, typename DistanceMap,
          typename WeightMap, typename BackGraph, typename BackWeightMap,
          typename BackIndexMap, typename Vertex = vertex_of_t<Graph>>
void bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
                            PredecessorMap predecessor, DistanceMap distance,
                            WeightMap weight, const BackGraph &G_b,
                            BackWeightMap weight_b, BackIndexMap index_map_b) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  using RevEdge =
      typename graph_traits<boost::reverse_graph<Graph>>::edge_descriptor;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<BackGraph>));
  BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<WeightMap, Edge>));
  BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<BackWeightMap, RevEdge>));

  auto visitor = IdentityBiDijkstraVisitor{};
  bidirectional_dijkstra(G, s, t, predecessor, distance, weight, G_b, weight_b,
                         index_map_b, visitor);
}
} // namespace arlib

#endif