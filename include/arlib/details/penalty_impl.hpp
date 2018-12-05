/**
 * @file penalty_impl.hpp
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

#ifndef BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_IMPL_HPP
#define BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_IMPL_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/property_map/property_map.hpp>

#include <arlib/details/arlib_utils.hpp>
#include <arlib/routing_kernels/bidirectional_dijkstra.hpp>
#include <arlib/routing_kernels/types.hpp>
#include <arlib/terminators.hpp>
#include <arlib/type_traits.hpp>

#include <functional>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arlib {
/**
 * Implementations details of kSPwLO algorithms
 */
namespace details {
//===----------------------------------------------------------------------===//
//                      Penalty algorithm types
//===----------------------------------------------------------------------===//
/**
 * A map from edges to the number of time they have been penalized so
 * far.
 *
 * @tparam Edge An edge_descriptor
 */
template <typename Edge>
using PenBoundsMap = std::unordered_map<Edge, int, boost::hash<Edge>>;

/**
 * A map from edges to their weights
 *
 * @tparam Edge An edge_descriptor
 * @tparam Length The edge weight type
 */
template <typename Edge, typename Length>
using WeightMap = std::unordered_map<Edge, Length, boost::hash<Edge>>;

/**
 * A vector for tracking distance of a vertex from the source.
 *
 * The vector must be indexable by the vertex_descriptor, thus its size should
 * be equal to the number of vertices in the graph.
 *
 * @tparam Length The edge weight type.
 */
template <typename Length> using DistanceMap = std::vector<Length>;

//===----------------------------------------------------------------------===//
//                      Penalty algorithm classes
//===----------------------------------------------------------------------===//

/**
 * A functor to return the penalized weight of an edge, to avoid changing
 * the original graph weights.
 *
 * @tparam PMap A Weight Property Map.
 */
template <typename PMap> class penalty_functor {
public:
  using Edge = typename boost::property_traits<PMap>::key_type;
  using Length = double;

  /**
   * Construct a new penalty functor object.
   *
   * @tparam EdgeIterator An iterator of the graph edges.
   * @param weight The weight property map.
   */
  penalty_functor(PMap weight)
      : weight{weight}, penalties{std::make_shared<WeightMap<Edge, Length>>()} {
  }

  /**
   * Copy constructor.
   *
   * @param other The penalty functor to copy from
   */
  penalty_functor(const penalty_functor<PMap> &other)
      : weight{other.weight}, penalties{other.penalties} {}

  /**
   * Returns the penalized weight of an edge.
   *
   * @param e The query edge.
   * @return The penalized weight for @p e.
   */
  const Length &operator()(const Edge &e) const { return get_or_insert(e); }

  /**
   * Returns the penalized weight of an edge.
   *
   * @param e The query edge.
   * @return The penalized weight for @p e.
   */
  Length &operator[](const Edge &e) { return get_or_insert(e); }

  /**
   * Returns the penalized weight of an edge.
   *
   * @param e The query edge.
   * @return The penalized weight for @p e.
   */
  const Length &operator[](const Edge &e) const { return get_or_insert(e); }

  penalty_functor clone() const {
    auto pf = *this;
    pf.penalties = std::make_shared<WeightMap<Edge, Length>>(*penalties);
    return pf;
  }

private:
  PMap weight;
  mutable std::shared_ptr<WeightMap<Edge, Length>> penalties;

  Length &get_or_insert(const Edge &e) const {
    if (auto search = penalties->find(e); search != penalties->end()) {
      return search->second;
    } else {
      auto [it, ok] = penalties->insert({e, weight[e]});
      assert(ok && "[kspwlo::penalty_functor] Could not insert edge weight");
      return it->second;
    }
  }
};

template <typename PMap, typename Graph> class reverse_penalty_functor {
public:
  using Edge = typename boost::graph_traits<
      boost::reverse_graph<Graph>>::edge_descriptor;
  using Length = double;

  reverse_penalty_functor(penalty_functor<PMap> &penalty, const Graph &G,
                          const boost::reverse_graph<Graph> &rev_G)
      : inner_pf{penalty}, G{G}, rev_G{rev_G} {}

  reverse_penalty_functor(reverse_penalty_functor const &other)
      : inner_pf{other.inner_pf}, G{other.G}, rev_G{other.rev_G} {}

  const Length &operator()(const Edge &e) const {
    auto forward_edge = get_forward_edge(e);

    return inner_pf(forward_edge);
  }

  Length &operator[](const Edge &e) {
    auto forward_edge = get_forward_edge(e);

    return inner_pf[forward_edge];
  }

  const Length &operator[](const Edge &e) const {
    auto forward_edge = get_forward_edge(e);

    return inner_pf[forward_edge];
  }

private:
  auto get_forward_edge(const Edge &e) const {
    using namespace boost;
    auto u = source(e, rev_G);
    auto v = target(e, rev_G);

    auto [forward_edge, is_valid] = edge(v, u, G);
    assert(is_valid);
    return forward_edge;
  }

  penalty_functor<PMap> &inner_pf;
  const Graph &G;
  const boost::reverse_graph<Graph> &rev_G;
};

//===----------------------------------------------------------------------===//
//                     Penalty algorithm support routines
//===----------------------------------------------------------------------===//
/**
 * Computes the shortest path between two vertices s and t, first
 * from s to t and then from t to s. The distances of each node in the
 * shortest paths are stored in distance_s and distance_t.
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *               property with tag boost::edge_weight_t.
 * @tparam Vertex A vertex_descriptor
 * @tparam Length The edge weight type.
 * @param G The graph.
 * @param s The source vertex.
 * @param t The target vertex.
 * @param distance_s A map from Vertex to its distance from @p s
 * @param distance_t A map from Vertex to its distance from @p t
 * @return A vector of the edges of the shortest path from s to t.
 *         An empty optional if t is not reachable from s.
 */
template <typename Graph, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>,
          typename Length = length_of_t<Graph>>
std::optional<std::vector<Edge>>
dijkstra_shortest_path_two_ways(const Graph &G, Vertex s, Vertex t,
                                DistanceMap<Length> &distance_s,
                                DistanceMap<Length> &distance_t) {
  using namespace boost;

  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);

  // Forward step
  dijkstra_shortest_paths(G, s,
                          distance_map(&distance_s[0])
                              .predecessor_map(make_iterator_property_map(
                                  std::begin(predecessor), vertex_id, s)));
  auto edge_list = build_edge_list_from_dijkstra(G, s, t, predecessor);

  // Backward step
  auto rev_G = make_reverse_graph(G);
  dijkstra_shortest_paths(rev_G, t, distance_map(&distance_t[0]));

  if (exists_path_to<Length>(t, distance_s)) {
    return std::make_optional(edge_list);
  } else {
    // In case t could not be found from astar_search and target_found is not
    // thrown, return empty optional
    return std::optional<std::vector<Edge>>{};
  }

} // namespace kspwlo_impl

/**
 * Computes the Dijkstra shortest path from s to t using a
 *        penalty_functor to gather edges weight instead of the Graph's weight
 *        property map
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *               property with tag boost::edge_weight_t.
 * @tparam PMap The graph's weight property map.
 * @tparam Vertex A vertex_descriptor.
 * @tparam Edge An edge_descriptor.
 * @tparam Length The edge weight type.
 * @param G The graph
 * @param s The source vertex
 * @param t The target vertex
 * @param penalty A penalty_functor
 * @return A vector of the edges of the shortest path from s to t.
 *         An empty optional if t is not reachable from s.
 */
template <typename Graph, typename PMap, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>,
          typename Length = length_of_t<Graph>>
std::optional<std::vector<Edge>>
dijkstra_shortest_path(const Graph &G, Vertex s, Vertex t,
                       penalty_functor<PMap> &penalty) {
  using namespace boost;

  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);

  auto weight = make_function_property_map<Edge>(penalty);
  try {
    dijkstra_shortest_paths(
        G, s,
        weight_map(weight)
            .predecessor_map(make_iterator_property_map(std::begin(predecessor),
                                                        vertex_id, s))
            .visitor(make_dijkstra_visitor(
                make_target_visitor(t, on_examine_vertex{}))));
  } catch (target_found tf) {
    auto edge_list = build_edge_list_from_dijkstra(G, s, t, predecessor);
    return std::make_optional(edge_list);
  }

  // In case t could not be found from astar_search and target_found is not
  // thrown, return empty optional
  return std::optional<std::vector<Edge>>{};
}

template <typename Graph, typename PMap, typename AStarHeuristic,
          typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>,
          typename Length = length_of_t<Graph>>
std::optional<std::vector<Edge>>
astar_shortest_path(const Graph &G, Vertex s, Vertex t,
                    penalty_functor<PMap> &penalty,
                    const AStarHeuristic &heuristic) {
  using namespace boost;
  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);

  auto weight = make_function_property_map<Edge>(penalty);
  try {
    astar_search(G, s, heuristic,
                 predecessor_map(make_iterator_property_map(
                                     std::begin(predecessor), vertex_id, s))
                     .visitor(astar_target_visitor{t})
                     .weight_map(weight));
  } catch (target_found &tf) {
    auto edge_list = build_edge_list_from_dijkstra(G, s, t, predecessor);
    return std::make_optional(edge_list);
  }
  // In case t could not be found from astar_search and target_found is not
  // thrown, return empty optional
  return std::optional<std::vector<Edge>>{};
}

template <typename Graph, typename PMap, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>,
          typename Length = length_of_t<Graph>>
std::optional<std::vector<Edge>>
bidirectional_dijkstra_shortest_path(const Graph &G, Vertex s, Vertex t,
                                     penalty_functor<PMap> &penalty) {
  using namespace boost;

  auto index = get(vertex_index, G);
  auto predecessor_vec = std::vector<Vertex>(num_vertices(G), s);
  auto predecessor = make_iterator_property_map(predecessor_vec.begin(), index);
  auto distance_vec = std::vector<Length>(num_vertices(G));
  auto distance = make_iterator_property_map(distance_vec.begin(), index);
  auto weight = make_function_property_map<Edge>(penalty);

  auto rev_G = make_reverse_graph(G);
  auto rev_weight_ = reverse_penalty_functor(penalty, G, rev_G);
  using RevEdge = typename boost::graph_traits<
      boost::reverse_graph<Graph>>::edge_descriptor;
  auto rev_weight = make_function_property_map<RevEdge>(rev_weight_);
  auto rev_index = get(vertex_index, rev_G);

  try {
    bidirectional_dijkstra(G, s, t, predecessor, distance, weight, rev_G,
                           rev_weight, rev_index);
  } catch (details::target_not_found &) {
    // In case t could not be found return empty optional
    return std::optional<std::vector<Edge>>{};
  }

  auto edge_list = build_edge_list_from_dijkstra(G, s, t, predecessor);
  return std::make_optional(edge_list);
}

template <typename Graph, typename PMap, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
constexpr std::function<std::optional<std::vector<Edge>>(
    const Graph &, Vertex, Vertex, penalty_functor<PMap> &)>
build_shortest_path_fn(routing_kernels algorithm, const Graph &, const PMap &) {
  switch (algorithm) {
  case routing_kernels::dijkstra:
    return [](const auto &G, auto s, auto t, auto &penalty) {
      return dijkstra_shortest_path(G, s, t, penalty);
    };
  case routing_kernels::bidirectional_dijkstra:
    return [](const auto &G, auto s, auto t, auto &penalty) {
      return bidirectional_dijkstra_shortest_path(G, s, t, penalty);
    };
  default:
    throw std::invalid_argument{
        "Invalid algorithm. Only [dijkstra|bidirectional_dijkstra] allowed."};
  }
}

template <typename Graph, typename PMap, typename AStarHeuristic,
          typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
constexpr std::function<std::optional<std::vector<Edge>>(
    const Graph &, Vertex, Vertex, penalty_functor<PMap> &)>
build_shortest_path_fn(routing_kernels algorithm, const Graph &, const PMap &,
                       const AStarHeuristic &heuristic) {
  switch (algorithm) {
  case routing_kernels::astar:
    return [&heuristic](const auto &G, auto s, auto t, auto &penalty) {
      return astar_shortest_path(G, s, t, penalty, heuristic);
    };
  default:
    throw std::invalid_argument{"Invalid algorithm. Only [astar] allowed."};
  }
}

/**
 * Apply penalization step to the candidate path.
 *
 * @pre For each vertex @c v in @p candidate @p distance_s contains the shortest
 *      path distance of @c v from @p s
 * @pre For each vertex @c v in @p candidate @p distance_t contains the shortest
 *      path distance of @c v from @p t
 * @post For each edge @c e in @p candidate, if <tt>penalty_bounds[e] <
 *       bound_limit</tt>, @c e is penalized in @p penalty according to the
 *       following formula: <tt>w(e)_new = w(e) + @p p * w(e)</tt>
 * @post For each edge @c e incoming to a vertex @c u in @p candidate, if
 *       <tt>penalty_bounds[e] < bound_limit</tt>, @c e is penalized in @p
 *       penalty according to the following formula:
 *       <tt>w(e)_new = w(e) + w(e) * (0.1 + @p r * @p distance_t[u] / @p
 *       distance_t[s])</tt>
 * @post For each edge @c e outgoing from a vertex @c v in @p candidate, if
 *       <tt>penalty_bounds[e] < bound_limit</tt>, @c e is penalized in @p
 *       penalty according to the following formula:
 *       <tt>w(e)_new = w(e) + w(e) * (0.1 + @p r * @p distance_s[v] / @p
 *       distance_s[t])</tt>
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *               property with tag boost::edge_weight_t.
 * @tparam DistanceMap A DistanceMap.
 * @tparam PMap The graph's weight property map.
 * @tparam Vertex A vertex_descriptor.
 * @tparam Edge An edge_descriptor.
 * @tparam Length The edge weight type.
 * @param candidate The candidate path.
 * @param G The graph.
 * @param s The source vertex.
 * @param t The target vertex.
 * @param p The penalty factor for edges in the candidate path.
 * @param r The penalty factor for edges incoming and outgoing to/from vertices
 *          of the candidate path.
 * @param penalty A penalty_functor.
 * @param distance_s A DistanceMap from s.
 * @param distance_t A DistanceMap from t.
 * @param penalty_bounds A PenBoundsMap.
 * @param bound_limit The maximum number of times an edge can be penalized.
 */
template <typename Graph, typename DistanceMap, typename PMap,
          typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>,
          typename Length = length_of_t<Graph>>
void penalize_candidate_path(const std::vector<Edge> &candidate, const Graph &G,
                             Vertex s, Vertex t, double p, double r,
                             penalty_functor<PMap> &penalty,
                             const DistanceMap &distance_s,
                             const DistanceMap &distance_t,
                             PenBoundsMap<Edge> &penalty_bounds,
                             int bound_limit) {
  using namespace boost;

  // Keep track of candidate vertices to exclude them from incoming/outgoing
  // edges update
  auto candidate_vertices = std::unordered_set<Vertex, boost::hash<Vertex>>{};
  auto candidate_edges = std::unordered_set<Edge, boost::hash<Edge>>{};

  for (const auto &e : candidate) {
    candidate_edges.insert(e);

    auto u = source(e, G);
    auto v = target(e, G);
    candidate_vertices.insert(u);
    candidate_vertices.insert(v);
  }

  for (auto &e : candidate_edges) {
    auto u = source(e, G);
    auto v = target(e, G);

    // Check if 'e' is already part of the alternative graph
    if (auto search = penalty_bounds.find(e);
        search != std::end(penalty_bounds)) {
      // If so, penalize only if limit isnt reached
      auto n_updates = search->second;
      if (n_updates < bound_limit) {
        penalty[e] += p * penalty[e];
        ++penalty_bounds[e];
      }
    } else { // Penalize and create a nb_updates counter for e
      penalty[e] += p * penalty[e];
      penalty_bounds.insert({e, 1});
    }

    // Update incoming edges
    for (auto [it, end] = in_edges(u, G); it != end; ++it) {
      auto a = source(*it, G);

      // Incoming edge (a, u) is updated only if 'a' is not part of candidate
      // path
      if (candidate_vertices.find(a) == std::end(candidate_vertices)) {
        // Check if '*it' is already part of the alternative graph
        if (auto search = penalty_bounds.find(*it);
            search != std::end(penalty_bounds)) {
          auto n_updates = search->second;
          // penalize only if limit isnt reached
          if (n_updates < bound_limit) {
            auto closeness = distance_t[u] / distance_t[s];
            auto pen_factor = 0.1 + r * closeness;
            penalty[*it] += pen_factor * penalty[*it];
            ++penalty_bounds[*it];
          }
        } else {
          // Else, just update it
          auto closeness = distance_t[u] / distance_t[s];
          auto pen_factor = 0.1 + r * closeness;
          penalty[*it] += pen_factor * penalty[*it];
        }
      }
    }

    // Update outgoing edges
    for (auto [it, end] = out_edges(v, G); it != end; ++it) {
      auto b = target(*it, G);

      // Outgoing edge (v, b) is updated only if 'b' is not part of candidate
      // path
      if (candidate_vertices.find(b) == std::end(candidate_vertices)) {
        // Check if '*it' is already part of the alternative graph
        if (auto search = penalty_bounds.find(*it);
            search != std::end(penalty_bounds)) {
          auto n_updates = search->second;
          // penalize only if limit isnt reached
          if (n_updates < bound_limit) {
            auto closeness = distance_s[v] / distance_s[t];
            auto pen_factor = 0.1 + r * closeness;
            penalty[*it] += pen_factor * penalty[*it];
            ++penalty_bounds[*it];
          }
        } else {
          // Else, just update it
          auto closeness = distance_s[v] / distance_s[t];
          auto pen_factor = 0.1 + r * closeness;
          penalty[*it] += pen_factor * penalty[*it];
        }
      }
    }
  }
}

template <typename Graph, typename WeightMap, typename MultiPredecessorMap,
          typename RoutingKernel, typename Terminator,
          typename Vertex = vertex_of_t<Graph>>
void penalty(const Graph &G, WeightMap const &original_weight,
             MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k,
             double theta, double p, double r, int max_nb_updates,
             int max_nb_steps, RoutingKernel &routing_kernel,
             Terminator &&terminator) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  using Length = typename boost::property_traits<typename boost::property_map<
      Graph, boost::edge_weight_t>::type>::value_type;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<WeightMap, Edge>));

  // P_LO set of k paths
  auto resPathsEdges = std::vector<std::vector<Edge>>{};
  auto resEdges = std::vector<std::unordered_set<Edge, boost::hash<Edge>>>{};

  // P_LO set of k paths
  auto resPaths = std::vector<Path<Graph>>{};
  // Make a local weight map to avoid modifying existing graph.
  auto pen_fctor = details::penalty_functor{original_weight};

  // Compute shortest path from s to t
  auto distance_s = std::vector<Length>(num_vertices(G));
  auto distance_t = std::vector<Length>(num_vertices(G));
  auto sp = dijkstra_shortest_path_two_ways(G, s, t, distance_s, distance_t);
  if (!sp) {
    auto oss = std::ostringstream{};
    oss << "Vertex " << t << " is unreachable from " << s;
    throw details::target_not_found{oss.str()};
  }

  // P_LO <-- {shortest path p_0(s, t)};
  resPathsEdges.push_back(*sp);
  resEdges.emplace_back(sp->begin(), sp->end());

  // If we need the shortest path only
  if (k == 1) {
    fill_multi_predecessor(resPathsEdges.begin(), resPathsEdges.end(), G,
                           predecessors);
    return;
  }

  // Initialize map for penalty bounds
  auto penalty_bounds = std::unordered_map<Edge, int, boost::hash<Edge>>{};

  // Penalize sp edges
  penalize_candidate_path(*sp, G, s, t, p, r, pen_fctor, distance_s, distance_t,
                          penalty_bounds, max_nb_updates);

  int step = 0;
  using Index = std::size_t;
  while (resPathsEdges.size() < static_cast<Index>(k) && step < max_nb_steps) {
    // The remainder code is the hot part of the algorithm. So we check here
    // if the algorithm should terminate
    if (terminator.should_stop()) {
      throw terminator_stop_error{
          "Penalty terminated before completing due to a Terminator. Please "
          "discard partial output."};
    }

    auto p_tmp = routing_kernel(G, s, t, pen_fctor);

    // Penalize p_tmp edges
    penalize_candidate_path(*p_tmp, G, s, t, p, r, pen_fctor, distance_s,
                            distance_t, penalty_bounds, max_nb_updates);
    ++step;

    // If p_tmp is sufficiently dissimilar to other alternative paths, accept it
    bool is_valid_path = true;
    for (const auto &alt_path : resEdges) {
      if (compute_similarity(*p_tmp, alt_path, original_weight) > theta) {
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
  fill_multi_predecessor(resPathsEdges.begin(), resPathsEdges.end(), G,
                         predecessors);
}
} // namespace details
} // namespace arlib

#endif