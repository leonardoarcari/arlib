/**
 * @file arlib_utils.hpp
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

#ifndef KSPWLO_IMPL_HPP
#define KSPWLO_IMPL_HPP

#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>

#include <arlib/path.hpp>
#include <arlib/type_traits.hpp>

#include <iterator>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace arlib {
namespace details {
//===----------------------------------------------------------------------===//
//                      kSPwLO algorithms support classes
//===----------------------------------------------------------------------===//

struct target_found {};
struct target_not_found : public std::logic_error {
  explicit target_not_found(const std::string &what_arg)
      : std::logic_error(what_arg) {}
  explicit target_not_found(const char *what_arg)
      : std::logic_error(what_arg) {}
};

template <typename Vertex, typename Tag>
class target_visitor : public boost::base_visitor<target_visitor<Vertex, Tag>> {
public:
  using event_filter = Tag;

  explicit target_visitor(Vertex t) : t{t} {}
  template <typename Graph> void operator()(Vertex u, Graph &) {
    if (u == t) {
      throw target_found{};
    }
  }

private:
  Vertex t;
};
template <typename Vertex, typename Tag>
target_visitor<Vertex, Tag> make_target_visitor(Vertex t, Tag) {
  return target_visitor<Vertex, Tag>{t};
}

/**
 * An A* visitor to stop the algorithm when a target vertex is found.
 *
 * This visitor is required when you are interested just in finding a route from
 * a source to a target, while generally speaking A* Star search terminates when
 * a shortest path to all the nodes are found.
 *
 * @tparam Vertex A Boost::Graph vertex descriptor
 */
template <typename Vertex>
class astar_target_visitor : public boost::default_astar_visitor {
public:
  /**
   * Construct a new astar_target_visitor object that ends the search
   * when Vertex @p t is found.
   *
   * @param t The target Vertex
   */
  explicit astar_target_visitor(Vertex t) : t{t} {}
  /**
   * When Vertex <tt>u == t</tt> (i.e. we found target node) A* search stops
   * throwing a target_found exception.
   *
   * @tparam Graph A Boost::PropertyGraph having at least one edge
   *         property with tag boost::edge_weight_t.
   * @param u examined Vertex
   * @throws target_found when target Vertex is found.
   */
  template <typename Graph> void examine_vertex(Vertex u, Graph &) {
    if (u == t) {
      throw target_found{};
    }
  }

private:
  Vertex t;
};

//===----------------------------------------------------------------------===//
//                      kSPwLO algorithms routines
//===----------------------------------------------------------------------===//

template <typename CostType, typename Vertex, typename DistMap>
bool exists_path_to(Vertex v, const DistMap &dist) {
  auto inf = std::numeric_limits<CostType>::max();
  return dist[v] != inf;
}

/**
 * Computes the length of the path in @p candidate using weights from @p
 * G
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @param candidate The candidate path
 * @param G The graph
 * @return The length of @p candidate, i.e. @f$\sum_{e \in candidate} weight(e,
 *         G)@f$
 */
template <typename ForwardIt, typename GWeightMap,
          typename Length = value_of_t<GWeightMap>>
Length compute_length_from_edges(ForwardIt first, ForwardIt last,
                                 GWeightMap const &weight) {
  Length length = 0;

  for (auto it = first; it != last; ++it) {
    length += weight[*it];
  }

  return length;
}

template <typename GWeightMap, typename Edge,
          typename Length = value_of_t<GWeightMap>>
Length compute_shared_length(
    const std::vector<Edge> &candidate,
    std::unordered_set<Edge, boost::hash<Edge>> const &alt_path,
    GWeightMap const &weight) {
  using namespace boost;
  Length length = 0;

  for (const auto &e : candidate) {
    if (auto search = alt_path.find(e); search != alt_path.end()) {
      length += weight[e];
    }
  }

  return length;
}

template <typename Graph, typename GWeightMap,
          typename Length = value_of_t<GWeightMap>>
Length compute_shared_length(const Graph &candidate, const Graph &G,
                             GWeightMap const &weight) {
  using namespace boost;
  Length length = 0;

  for (auto [it, last] = edges(candidate); it != last; ++it) {
    auto u = source(*it, candidate);
    auto v = target(*it, candidate);
    auto egde_in_G = edge(u, v, G);
    bool edge_is_shared = egde_in_G.second;

    if (edge_is_shared) {
      length += weight[egde_in_G.first];
    }
  }

  return length;
}

template <typename Edge, typename AltEdgeWeightMap>
double
compute_similarity(const std::vector<Edge> &candidate,
                   const std::unordered_set<Edge, boost::hash<Edge>> &alt_path,
                   AltEdgeWeightMap const &weight) {
  double shared_length =
      static_cast<double>(compute_shared_length(candidate, alt_path, weight));
  double alt_length = static_cast<double>(
      compute_length_from_edges(alt_path.begin(), alt_path.end(), weight));
  return shared_length / alt_length;
}

template <typename Graph, typename AltEdgeWeightMap>
double compute_similarity(const Path<Graph> &candidate,
                          const Path<Graph> &alt_path,
                          AltEdgeWeightMap const &weight) {
  double shared_length =
      static_cast<double>(compute_shared_length(candidate, alt_path, weight));

  return shared_length / alt_path.length();
}

template <typename Length, typename Graph, typename Vertex = vertex_of_t<Graph>>
std::vector<Length> distance_from_target(const Graph &G, Vertex t) {
  // Reverse graph
  auto G_rev = boost::make_reverse_graph(G);
  auto distance = std::vector<Length>(boost::num_vertices(G_rev));

  // Run dijkstra_shortest_paths and return distance vector
  boost::dijkstra_shortest_paths(G_rev, t, boost::distance_map(&distance[0]));
  return distance;
}

/**
 * An A* heuristic using <em>distance from target</em> lower bound.
 *
 * In order to derive tight h(n, t) lower bounds, we first reverse the edges of
 * the road network and then run Dijkstra’s algorithm from target t to every
 * node n of the network
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam CostType The value type of an edge weight of Graph.
 */
template <typename Graph, typename CostType>
class distance_heuristic : public boost::astar_heuristic<Graph, CostType> {
public:
  /**
   * Graph vertex descriptor.
   */
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
  /**
   * Construct a new distance heuristic object. Upon construction a
   * Dijkstra’s algorithm is run on @c G' from @p t to all nodes. @c G' is equal
   * to @p G except that all the edges are reversed.
   *
   * @param G The Graph on which to search
   * @param t The target vertex
   */
  distance_heuristic(const Graph &G, Vertex t)
      : lower_bounds{distance_from_target<CostType>(G, t)} {}
  /**
   * @param u The Vertex
   * @return The heuristic of the cost of Vertex @p u.
   */
  CostType operator()(Vertex u) const { return lower_bounds[u]; }

private:
  std::vector<CostType> lower_bounds;
};

template <typename Graph, typename EdgeWeightMap, typename PredecessorMap,
          typename Vertex>
Path<Graph>
build_path_from_dijkstra(const Graph &G, EdgeWeightMap const &weight,
                         const PredecessorMap &p, Vertex s, Vertex t) {
  using boost::edge;
  using Length = typename boost::property_traits<EdgeWeightMap>::value_type;
  using Edge = typename boost::graph_traits<Graph>::edge_descriptor;

  auto length = Length{};
  auto path_es = std::unordered_set<Edge, boost::hash<Edge>>{};
  auto path_vs = std::unordered_set<Vertex, boost::hash<Vertex>>{};

  path_vs.insert(t);
  auto current = t;
  while (current != s) {
    auto u = p[current];
    path_vs.insert(u);

    auto edge_in_G = edge(u, current, G).first;
    length += weight[edge_in_G];
    path_es.insert(edge_in_G);
    current = u;
  }

  using FilteredGraph =
      boost::filtered_graph<Graph, alternative_path_edges<Edge>,
                            alternative_path_vertices<Vertex>>;
  std::shared_ptr<FilteredGraph> fg =
      make_path_filtered_graph(G, std::move(path_es), std::move(path_vs));
  return Path{fg, length};
}

/**
 * Builds a vector of kspwlo::Edge out of a PredecessorMap computed by a
 * shortest path algorithm from @p s to @p t
 *
 * @tparam Vertex
 * @tparam PredecessorMap
 * @param s The source vertex
 * @param t The target vertex
 * @param p The PredecessorMap
 * @return A vector of the shortest path edges from @p s to @p t.
 */
template <typename Graph, typename PredecessorMap,
          typename Edge = edge_of_t<Graph>,
          typename Vertex = vertex_of_t<Graph>>
std::vector<Edge> build_edge_list_from_dijkstra(Graph const &G, Vertex s,
                                                Vertex t,
                                                const PredecessorMap &p) {
  using namespace boost;
  auto edge_list = std::vector<Edge>{};

  auto current = t;
  while (current != s) {
    auto u = p[current];
    if (u == current) {
      // Vertex 'current' is not reachable from source vertex
      break;
    }
    auto [e, is_ok] = edge(u, current, G);
    assert(is_ok &&
           "[arlib::details::build_edge_list_from_dijkstra] Edge not found.");
    edge_list.push_back(e);
    current = u;
  }

  return edge_list;
}

template <typename Graph, typename EdgeWeightMap,
          typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>,
          typename Length = value_of_t<EdgeWeightMap>>
std::optional<std::vector<Edge>>
compute_shortest_path(const Graph &G, EdgeWeightMap const &weight, Vertex s,
                      Vertex t) {
  using namespace boost;
  auto sp_distances = std::vector<Length>(num_vertices(G));
  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);
  try {
    dijkstra_shortest_paths(
        G, s,
        distance_map(&sp_distances[0])
            .predecessor_map(make_iterator_property_map(std::begin(predecessor),
                                                        vertex_id, s))
            .weight_map(weight)
            .visitor(make_dijkstra_visitor(
                make_target_visitor(t, on_examine_vertex{}))));
  } catch (target_found &tf) {
    auto path = build_edge_list_from_dijkstra(G, s, t, predecessor);
    return {path};
  }

  // If target was not found, t is unreachable from s
  return {};
}

template <typename ForwardIt, typename Graph, typename MultiPredecessorMap>
void fill_multi_predecessor(ForwardIt first, ForwardIt last, Graph const &G,
                            MultiPredecessorMap &pmap) {
  using namespace boost;
  for (auto it = first; it != last; ++it) {
    auto n = std::distance(first, it);
    auto const &edge_list = *it;
    for (auto const &e : edge_list) {
      auto u = source(e, G);
      auto v = target(e, G);
      auto &preds = get(pmap, v);
      preds.insert({n, u});
    }
  }
}
} // namespace details
} // namespace arlib

#endif