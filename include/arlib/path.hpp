/**
 * @file path.hpp
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

#ifndef ALTERNATIVE_ROUTING_LIB_PATH_HPP
#define ALTERNATIVE_ROUTING_LIB_PATH_HPP

#include <arlib/details/path_impl.hpp>
#include <arlib/type_traits.hpp>

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <utility>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
/**
 * A *view* of a graph to a simple path. A (simple) Path from `s` to `t` is a
 * cycle-free connected sequence of edges \f$p_{st} = \langle	(s, v_1), (v_1,
 * v_x), \dots , (v_y, t) \rangle\f$.
 *
 * This class wraps a `boost::filtered_graph<Graph>` to expose only vertices and
 * edges that make up a simple path. Moreover it provides the `length` of the
 * path, which is defined as the sum of the weights of all the edges. In
 * formula: \f$\ell(p) = \sum_{\forall (u, v) \in p} w_{uv}\f$.
 *
 * Just like `boost::filtered_graph`, Path models the same concepts as the
 * underlying graph type. If the underlying `Graph` type models
 * `VertexAndEdgeListGraph` and `PropertyGraph` then so does the Path.
 * If the underlying `Graph` type models fewer or smaller concepts than these,
 * then so does the Path.
 *
 * @tparam Graph The graph type
 */
template <typename Graph, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>,
          typename Length = length_of_t<Graph>>
class Path {
public:
  /**
   * The inner boost::filtered_graph type
   */
  using FilteredGraph =
      boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                            details::alternative_path_vertices<Vertex>>;
  /**
   * Construct a new Path object from a FilteredGraph and the Path length.
   *
   * @param g The FilteredGraph to wrap.
   * @param length The path length.
   */
  Path(std::shared_ptr<FilteredGraph> g, Length length)
      : graph_{std::move(g)}, length_{length} {};
  /**
   * Copy-construct a new Path.
   *
   * Copy operations on Path are always **shallow**. That means
   * that multiple copies always refer to the **same memory location**.
   *
   * @param other The Path to copy from.
   */
  Path(Path const &other) = default;
  /**
   * Move-construct a new Path.
   *
   * @param other The Path to move.
   */
  Path(Path &&other) noexcept = default;
  /**
   * Copy-assign a Path to `this`.
   *
   * Copy operations on Path are always **shallow**. That means
   * that multiple copies always refer to the **same memory location**.
   *
   * @param other The Path to copy from.
   * @return `this`.
   */
  Path &operator=(Path const &other) = default;
  /**
   * Move-assign a Path to `this`.
   *
   * @param other The Path to move.
   * @return `this`.
   */
  Path &operator=(Path &&other) noexcept = default;
  /**
   * Swaps `this` with another Path.
   *
   * @param other The Path to swap with.
   */
  void swap(Path &other) {
    using std::swap;
    swap(graph_, other.graph_);
    swap(length_, length_);
  }
  /**
   * @return the length of the path.
   */
  Length length() const { return length_; }

  //===-------------------------------------------------------------------===//
  //                           Graph concept
  //===-------------------------------------------------------------------===//
  /**
   * The type for the vertex descriptors associated with the Path,
   * which is the same type as the `vertex_descriptor` for the original `Graph`.
   */
  using vertex_descriptor =
      typename boost::graph_traits<FilteredGraph>::vertex_descriptor;
  /**
   * The type for the edge descriptors associated with the Path,
   * which is the same type as the `edge_descriptor` for the original `Graph`.
   */
  using edge_descriptor =
      typename boost::graph_traits<FilteredGraph>::edge_descriptor;
  /**
   * Provides information about whether the graph is directed (`directed_tag`)
   * or undirected (`undirected_tag`).
   */
  using directed_category =
      typename boost::graph_traits<FilteredGraph>::directed_category;
  /**
   * This describes whether the graph class allows the insertion of parallel
   * edges (edges with the same source and target). The two tags are
   * `allow_parallel_edge_tag` and `disallow_parallel_edge_tag`.
   */
  using edge_parallel_category =
      typename boost::graph_traits<FilteredGraph>::edge_parallel_category;
  /**
   * The ways in which the vertices and edges of the graph can be visited
   */
  using traversal_category =
      typename boost::graph_traits<FilteredGraph>::traversal_category;
  /**
   * @return A special `boost::graph_traits<Path<Graph>>::vertex_descriptor`
   * object which does not refer to any vertex of graph object which type is
   * `Graph`.
   */
  static vertex_descriptor null_vertex() {
    return boost::graph_traits<FilteredGraph>::null_vertex();
  }

  //===-------------------------------------------------------------------===//
  //                         IncidenceGraph concept
  //===-------------------------------------------------------------------===//
  /**
   * The type for the iterators returned by out_edges().
   * The iterator is a model of `MultiPassInputIterator`.
   */
  using out_edge_iterator =
      typename boost::graph_traits<FilteredGraph>::out_edge_iterator;
  /**
   * The type used for dealing with the number of edges incident to a vertex in
   * the graph
   */
  using degree_size_type =
      typename boost::graph_traits<FilteredGraph>::degree_size_type;

  template <typename G>
  friend std::pair<typename Path<G>::out_edge_iterator,
                   typename Path<G>::out_edge_iterator>
  out_edges(typename Path<G>::vertex_descriptor u, Path<G> const &g);

  template <typename G>
  friend typename Path<G>::vertex_descriptor
  source(typename Path<G>::edge_descriptor e, Path<G> const &g);

  template <typename G>
  friend typename Path<G>::vertex_descriptor
  target(typename Path<G>::edge_descriptor e, Path<G> const &g);

  template <typename G>
  friend typename Path<G>::degree_size_type
  out_degree(typename Path<G>::vertex_descriptor v, Path<G> const &g);

  //===-------------------------------------------------------------------===//
  //                       BidirectionalGraph concept
  //===-------------------------------------------------------------------===//
  /**
   * The type for the iterators returned by in_edges().
   * The iterator is a model of `MultiPassInputIterator`.
   */
  using in_edge_iterator =
      typename boost::graph_traits<FilteredGraph>::in_edge_iterator;

  template <typename G>
  friend std::pair<typename Path<G>::in_edge_iterator,
                   typename Path<G>::in_edge_iterator>
  in_edges(typename Path<G>::vertex_descriptor v, Path<G> const &g);

  template <typename G>
  friend typename Path<G>::degree_size_type
  in_degree(typename Path<G>::vertex_descriptor v, Path<G> const &g);

  template <typename G>
  friend typename Path<G>::degree_size_type
  degree(typename Path<G>::vertex_descriptor v, Path<G> const &g);

  //===-------------------------------------------------------------------===//
  //                        AdjacencyGraph concept
  //===-------------------------------------------------------------------===//
  /**
   * The type for the iterators returned by adjacent_vertices().
   * The iterator is a model of `MultiPassInputIterator`.
   */
  using adjacency_iterator =
      typename boost::graph_traits<FilteredGraph>::adjacency_iterator;

  template <typename G>
  friend std::pair<typename Path<G>::adjacency_iterator,
                   typename Path<G>::adjacency_iterator>
  adjacent_vertices(typename Path<G>::vertex_descriptor v, Path<G> const &g);

  //===-------------------------------------------------------------------===//
  //                        VertexListGraph concept
  //===-------------------------------------------------------------------===//
  /**
   * The type for the iterators returned by vertices().
   * The iterator is a model of `MultiPassInputIterator`.
   */
  using vertex_iterator =
      typename boost::graph_traits<FilteredGraph>::vertex_iterator;
  /**
   * The type used for dealing with the number of vertices in the graph.
   */
  using vertices_size_type =
      typename boost::graph_traits<FilteredGraph>::vertices_size_type;

  template <typename G>
  friend std::pair<typename Path<G>::vertex_iterator,
                   typename Path<G>::vertex_iterator>
  vertices(Path<G> const &g);

  template <typename G>
  friend typename Path<G>::vertices_size_type num_vertices(Path<G> const &g);

  //===-------------------------------------------------------------------===//
  //                        VertexListGraph concept
  //===-------------------------------------------------------------------===//
  /**
   * The type for the iterators returned by edges().
   * The iterator is a model of `MultiPassInputIterator`.
   */
  using edge_iterator =
      typename boost::graph_traits<FilteredGraph>::edge_iterator;
  /**
   * The type used for dealing with the number of edges in the graph.
   */
  using edges_size_type =
      typename boost::graph_traits<FilteredGraph>::edges_size_type;

  template <typename G>
  friend std::pair<typename Path<G>::edge_iterator,
                   typename Path<G>::edge_iterator>
  edges(Path<G> const &g);

  template <typename G>
  friend typename Path<G>::edges_size_type num_edges(Path<G> const &g);

  template <typename G>
  friend std::pair<typename Path<G>::edge_descriptor, bool>
  edge(typename Path<G>::vertex_descriptor u,
       typename Path<G>::vertex_descriptor v, Path<G> const &g);

  //===-------------------------------------------------------------------===//
  //                          PropertyMap concept
  //===-------------------------------------------------------------------===//

  template <typename G, typename Property>
  friend typename boost::property_map<typename Path<G>::FilteredGraph,
                                      Property>::type
  get(Property p, Path<G> &g);

  template <typename G, typename Property>
  friend typename boost::property_map<typename Path<G>::FilteredGraph,
                                      Property>::const_type
  get(Property p, const Path<G> &g);

  template <typename G, typename Property, typename Key>
  friend typename boost::property_map_value<typename Path<G>::FilteredGraph,
                                            Property>::type
  get(Property p, const Path<G> &g, const Key &k);

  template <typename G, typename Property, typename Key, typename Value>
  friend void put(Property p, const Path<G> &g, const Key &k, const Value &val);

private:
  std::shared_ptr<FilteredGraph> graph_ = {};
  Length length_ = {};
};
/**
 * Swap between two Path.
 *
 * @tparam Graph The graph type.
 * @param v1 The first Path.
 * @param v2 The second Path.
 */
template <typename Graph> void swap(Path<Graph> &v1, Path<Graph> &v2) {
  v1.swap(v2);
}

//===----------------------------------------------------------------------===//
//                Non-member functions for the Path Incidence Graph
//===----------------------------------------------------------------------===//
/**
 * @return An iterator-range providing access to the out-edges of vertex `u` in
 * graph `g`. If the graph is undirected, this iterator-range provides access to
 * all edges incident on vertex `u`. For both directed and undirected graphs,
 * for an out-edge `e`, `source(e, g) == u` and `target(e, g) == v` where `v` is
 * a vertex adjacent to `u`.
 */
template <typename G>
std::pair<typename Path<G>::out_edge_iterator,
          typename Path<G>::out_edge_iterator>
out_edges(typename Path<G>::vertex_descriptor u, Path<G> const &g) {
  using boost::out_edges;
  return out_edges(u, *g.graph_);
}
/**
 * @return The source vertex of edge `e`.
 */
template <typename G>
typename Path<G>::vertex_descriptor source(typename Path<G>::edge_descriptor e,
                                           Path<G> const &g) {
  using boost::source;
  return source(e, *g.graph_);
}
/**
 * @return The target vertex of edge `e`.
 */
template <typename G>
typename Path<G>::vertex_descriptor target(typename Path<G>::edge_descriptor e,
                                           Path<G> const &g) {
  using boost::target;
  return target(e, *g.graph_);
}
/**
 * @return The number of edges leaving vertex `u`.
 */
template <typename G>
typename Path<G>::degree_size_type
out_degree(typename Path<G>::vertex_descriptor u, Path<G> const &g) {
  using boost::out_degree;
  return out_degree(u, *g.graph_);
}

//===----------------------------------------------------------------------===//
//              Non-member functions for the Path Bidirectional Graph
//===----------------------------------------------------------------------===//
/**
 * @return An iterator-range providing access to the in-edges of vertex
 * `v` in graph `g`. For an in-edge `e`, `target(e, g) == v` and `source(e, g)
 * == u` for some vertex `u` that is adjacent to `v`, whether the graph is
 * directed or undirected.
 */
template <typename G>
std::pair<typename Path<G>::in_edge_iterator,
          typename Path<G>::in_edge_iterator>
in_edges(typename Path<G>::vertex_descriptor v, Path<G> const &g) {
  using boost::in_edges;
  return in_edges(v, *g.graph_);
}
/**
 * @return The number of edges entering vertex `v`.
 */
template <typename G>
typename Path<G>::degree_size_type
in_degree(typename Path<G>::vertex_descriptor v, Path<G> const &g) {
  using boost::in_degree;
  return in_degree(v, *g.graph_);
}
/**
 * @return The number of in-edges plus out-edges (for directed graphs)
 * or the number of incident edges (for undirected graphs) of vertex `v` in
 * graph `g`.
 */
template <typename G>
typename Path<G>::degree_size_type degree(typename Path<G>::vertex_descriptor v,
                                          Path<G> const &g) {
  using boost::degree;
  return degree(v, *g.graph_);
}

//===----------------------------------------------------------------------===//
//              Non-member function for the Path Adjacency Graph
//===----------------------------------------------------------------------===//
/**
 * @return An iterator-range providing access to the vertices adjacent
 * to vertex `v` in graph `g`.
 */
template <typename G>
std::pair<typename Path<G>::adjacency_iterator,
          typename Path<G>::adjacency_iterator>
adjacent_vertices(typename Path<G>::vertex_descriptor v, Path<G> const &g) {
  using boost::adjacent_vertices;
  return adjacent_vertices(v, *g.graph_);
}

//===----------------------------------------------------------------------===//
//             Non-member function for the Path Vertex List Graph
//===----------------------------------------------------------------------===//
/**
 * @return An iterator-range providing access to the vertex set of graph `g`.
 */
template <typename G>
std::pair<typename Path<G>::vertex_iterator, typename Path<G>::vertex_iterator>
vertices(Path<G> const &g) {
  using boost::vertices;
  return vertices(*g.graph_);
}
/**
 * @return The number of vertices in the underlying graph.
 */
template <typename G>
typename Path<G>::vertices_size_type num_vertices(Path<G> const &g) {
  using boost::num_vertices;
  return num_vertices(*g.graph_);
}

//===----------------------------------------------------------------------===//
//              Non-member functions for the Edge List Graph
//===----------------------------------------------------------------------===//
/**
 * @return An iterator-range providing access to the edge set of graph `g`.
 */
template <typename G>
std::pair<typename Path<G>::edge_iterator, typename Path<G>::edge_iterator>
edges(Path<G> const &g) {
  using boost::edges;
  return edges(*g.graph_);
}
/**
 * @return The number of edges in the underlying graph.
 */
template <typename G>
typename Path<G>::edges_size_type num_edges(Path<G> const &g) {
  using boost::num_edges;
  return num_edges(*g.graph_);
}
/**
 * @return The edge connecting vertex `u` to vertex `v` in graph `g`.
 */
template <typename G>
std::pair<typename Path<G>::edge_descriptor, bool>
edge(typename Path<G>::vertex_descriptor u,
     typename Path<G>::vertex_descriptor v, Path<G> const &g) {
  using boost::edge;
  return edge(u, v, *g.graph_);
}

//===----------------------------------------------------------------------===//
//                              Property Map
//===----------------------------------------------------------------------===//
/**
 * @return The property map object for the vertex property specified by
 * `Property`. The `Property` must match one of the properties specified in
 * the graph's `VertexProperty` template argument.
 */
template <typename G, typename Property>
typename boost::property_map<typename Path<G>::FilteredGraph, Property>::type
get(Property p, Path<G> &g) {
  return get(p, const_cast<typename Path<G>::FilteredGraph &>(*g.graph_));
}
/**
 * @return The property map object for the vertex property specified by
 * `Property`. The `Property` must match one of the properties specified in
 * the graph's `VertexProperty` template argument.
 */
template <typename G, typename Property>
typename boost::property_map<typename Path<G>::FilteredGraph,
                             Property>::const_type
get(Property p, const Path<G> &g) {
  return get(p, (const typename Path<G>::FilteredGraph &)*(g.graph_));
}
/**
 * @return The property value for `k`, where `k` is either a vertex or
 * edge descriptor
 */
template <typename G, typename Property, typename Key>
typename boost::property_map_value<typename Path<G>::FilteredGraph,
                                   Property>::type
get(Property p, const Path<G> &g, const Key &k) {
  return get(p, (const typename Path<G>::FilteredGraph &)*(g.graph_), k);
}
/**
 * This sets the property value for `k` to `val`. `k` is either a vertex or edge
 * descriptor. `Value` must be convertible to `typename
 * property_traits<property_map<Path<G>, Property>::type>::value_type`
 */
template <typename G, typename Property, typename Key, typename Value>
void put(Property p, const Path<G> &g, const Key &k, const Value &val) {
  put(p, const_cast<typename Path<G>::FilteredGraph &>(*g.graph_), k, val);
}

//===----------------------------------------------------------------------===//
//              Non-member functions for the Path instantiation
//===----------------------------------------------------------------------===//

template <typename Graph, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
std::shared_ptr<
    boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                          details::alternative_path_vertices<Vertex>>>
make_path_filtered_graph(
    Graph const &G, std::unordered_set<Edge, boost::hash<Edge>> const &edges,
    std::unordered_set<Vertex, boost::hash<Vertex>> const &vertices) {
  using FilteredGraph =
      boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                            details::alternative_path_vertices<Vertex>>;
  auto ap_edges = details::alternative_path_edges{edges};
  auto ap_vertices = details::alternative_path_vertices{vertices};
  return std::make_shared<FilteredGraph>(G, ap_edges, ap_vertices);
}

template <typename Graph, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
std::shared_ptr<
    boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                          details::alternative_path_vertices<Vertex>>>
make_path_filtered_graph(
    Graph const &G, std::unordered_set<Edge, boost::hash<Edge>> &&edges,
    std::unordered_set<Vertex, boost::hash<Vertex>> &&vertices) {
  using FilteredGraph =
      boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                            details::alternative_path_vertices<Vertex>>;
  auto ap_edges = details::alternative_path_edges{std::move(edges)};
  auto ap_vertices = details::alternative_path_vertices{std::move(vertices)};
  return std::make_shared<FilteredGraph>(G, ap_edges, ap_vertices);
}
} // namespace arlib

/**
 * A collection of free, peer-reviewed C++ libraries.
 */
namespace boost {
/**
 * The property map type for vertex or edge properties in the graph. The same
 * property maps from the wrapped `filtered_graph` graph are available in the
 * Path.
 */
template <typename G, typename Property>
struct property_map<arlib::Path<G>, Property>
    : property_map<typename arlib::Path<G>::FilteredGraph, Property> {};
} // namespace boost

#endif // ALTERNATIVE_ROUTING_LIB_PATH_HPP
