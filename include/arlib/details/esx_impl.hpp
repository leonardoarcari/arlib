/**
 * @file esx_impl.hpp
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

#ifndef BOOST_EDGE_SUBSET_EXCLUSION_IMPL_HPP
#define BOOST_EDGE_SUBSET_EXCLUSION_IMPL_HPP

#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/details/arlib_utils.hpp>
#include <arlib/routing_kernels/bidirectional_dijkstra.hpp>
#include <arlib/routing_kernels/types.hpp>
#include <arlib/terminators.hpp>
#include <arlib/type_traits.hpp>

#include <limits>
#include <unordered_set>
#include <utility>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
/**
 * Implementations details of kSPwLO algorithms
 */
namespace details {

//===----------------------------------------------------------------------===//
//                      ESX algorithm support classes
//===----------------------------------------------------------------------===//

/**
 * Comparator for edges priority queue.
 *
 * Given two edges @c e1 and @c e2, if <tt>prio(e1) > prio(e2)</tt> then @c e1
 * must be popped before @c e2.
 *
 * @tparam Edge A Boost::Graph edge descriptor.
 */
template <typename Edge> struct EdgePriorityComparator {
  /**
   * The priority of Edge
   */
  using Priority = std::pair<Edge, int>;
  /**
   * Given two edges @c e1 and @c e2, if <tt>prio(e1) > prio(e2)</tt>
   * then @c e1 must be popped before @c e2.
   *
   * @param lhs first edge and its priority.
   * @param rhs second edge and its priority.
   * @return true if @p lhs has lower priority then @p rhs
   * @return false otherwise.
   */
  bool operator()(Priority lhs, Priority rhs) const {
    return lhs.second < rhs.second;
  }
};

/**
 * A filter functor for Boost::filtered_graph to hide edges deleted by
 * ESX.
 *
 * @tparam Edge A Boost::Graph edge descriptor
 */
template <typename Edge> class edge_deleted_filter {
public:
  /**
   * The deleted edges map.
   */
  using DeletedEdgeMap = std::unordered_set<Edge, boost::hash<Edge>>;
  /**
   * Empty constructor, required by Boost::Graph
   */
  edge_deleted_filter() : deleted_edge_map{nullptr} {}
  /**
   * Construct a new edge_deleted_filter object filtering edges contained
   * in @p deleted_edge_map.
   *
   * @param deleted_edge_map The set of deleted edges.
   */
  edge_deleted_filter(const DeletedEdgeMap &deleted_edge_map)
      : deleted_edge_map{std::addressof(deleted_edge_map)} {}
  /**
   * @param e Edge to check.
   * @return true if @p e is marked as deleted
   * @return false otherwise.
   */
  bool operator()(const Edge &e) const {
    return deleted_edge_map->find(e) == std::end(*deleted_edge_map);
  }

private:
  const DeletedEdgeMap *deleted_edge_map;
};

template <typename PMap, class Graph> class reverse_weight_functor {
public:
  using Edge = typename boost::graph_traits<
      boost::reverse_graph<Graph>>::edge_descriptor;
  using Length = typename boost::property_traits<PMap>::value_type;

  reverse_weight_functor(PMap &pmap, Graph const &G,
                         const boost::reverse_graph<Graph> &rev_G)
      : inner_weight{pmap}, G{G}, rev_G{rev_G} {}

  reverse_weight_functor(reverse_weight_functor const &other)
      : inner_weight{other.inner_weight}, G{other.G}, rev_G{other.rev_G} {}

  const Length &operator()(const Edge &e) const {
    auto forward_edge = get_forward_edge(e);

    return inner_weight[forward_edge];
  }

  Length &operator[](const Edge &e) {
    auto forward_edge = get_forward_edge(e);

    return inner_weight[forward_edge];
  }

  const Length &operator[](const Edge &e) const {
    auto forward_edge = get_forward_edge(e);

    return inner_weight[forward_edge];
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

  PMap &inner_weight;
  const Graph &G;
  const boost::reverse_graph<Graph> &rev_G;
};

//===----------------------------------------------------------------------===//
//                          ESX algorithm routines
//===----------------------------------------------------------------------===//
/**
 * Checks whether the path from @p s to @p t contains edge @p e or not.
 *
 * @pre @p predecessor is the PredecessorMap filled by a Boost::Graph
 * shortest path algorithm such that <tt>predecessor[s] == s</tt>
 *
 * @tparam Graph A Boost::EdgeList graph
 * @tparam PredMap A Boost::PredecessorMap
 * @param s The source vertex
 * @param t The target vertex
 * @param e The edge we want to check if present in <tt>path(s -> t)</tt>
 * @param G The graph containing edge @p e
 * @param predecessor The predecessor map
 * @return true If @p e is in the computed path from @p s to @p t
 * @return false otherwise.
 */
template <typename Graph, typename PredMap,
          typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
bool shortest_path_contains_edge(Vertex s, Vertex t, Edge e, const Graph &G,
                                 const PredMap &predecessor) {
  auto e_s = boost::source(e, G);
  auto e_t = boost::target(e, G);

  auto v = t;
  auto u = predecessor[v];
  while (u != s) {
    if (u == e_s && v == e_t) {
      return true;
    }
    v = u;
    u = predecessor[v];
  }
  return false;
}

template <typename Graph, typename WeightMap, typename DeletedEdgeMap,
          typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
std::optional<std::vector<Edge>>
dijkstra_shortest_path(const Graph &G, Vertex s, Vertex t,
                       const WeightMap &weight,
                       DeletedEdgeMap &deleted_edge_map) {
  using namespace boost;

  // Get a graph with deleted edges filtered out
  auto filter = edge_deleted_filter{deleted_edge_map};
  auto filtered_G = filtered_graph(G, filter);

  auto predecessor = std::vector<Vertex>(num_vertices(filtered_G), s);
  auto vertex_id = get(vertex_index, filtered_G);

  try {
    dijkstra_shortest_paths(
        filtered_G, s,
        weight_map(weight)
            .predecessor_map(make_iterator_property_map(std::begin(predecessor),
                                                        vertex_id, s))
            .visitor(make_dijkstra_visitor(
                make_target_visitor(t, on_examine_vertex{}))));
  } catch (target_found &) {
    auto edge_list = build_edge_list_from_dijkstra(G, s, t, predecessor);
    return std::make_optional(edge_list);
  }

  // In case t could not be found from astar_search and target_found is not
  // thrown, return empty optional
  return std::optional<std::vector<Edge>>{};
}

/**
 * Compute a shortest path between two vertices on a filtered graph using
 * an A* approach, provided the set of edges to filter and the heuristic.
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam AStarHeuristic
 * @tparam DeletedEdgeMap an edge_deleted_filter::DeletedEdgeMap
 * @param G The graph
 * @param s The source vertex
 * @param t The target vertex
 * @param heuristic The A* heuristic.
 * @param deleted_edge_map The set of edges to filter from @p G
 * @return A std::optional of the list of edges from @p s to @p t if a path
 * could be found. An empty optional otherwise.
 */
template <typename Graph, typename WeightMap, typename AStarHeuristic,
          typename DeletedEdgeMap, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
std::optional<std::vector<Edge>>
astar_shortest_path(const Graph &G, Vertex s, Vertex t, const WeightMap &weight,
                    const AStarHeuristic &heuristic,
                    DeletedEdgeMap &deleted_edge_map) {
  using namespace boost;

  // Get a graph with deleted edges filtered out
  auto filter = edge_deleted_filter{deleted_edge_map};
  auto filtered_G = filtered_graph(G, filter);

  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, filtered_G);

  try {
    astar_search(filtered_G, s, heuristic,
                 predecessor_map(make_iterator_property_map(
                                     std::begin(predecessor), vertex_id, s))
                     .visitor(astar_target_visitor{t})
                     .weight_map(weight));
  } catch (target_found &) {
    auto edge_list = build_edge_list_from_dijkstra(G, s, t, predecessor);
    return std::make_optional(edge_list);
  }
  // In case t could not be found from astar_search and target_found is not
  // thrown, return empty optional
  return std::optional<std::vector<Edge>>{};
}

template <typename Graph, typename WeightMap, typename DeletedEdgeMap,
          typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>,
          typename Length = length_of_t<Graph>>
std::optional<std::vector<Edge>>
bidirectional_dijkstra_shortest_path(const Graph &G, Vertex s, Vertex t,
                                     const WeightMap &weight,
                                     DeletedEdgeMap &deleted_edge_map) {
  using namespace boost;

  // Get a graph with deleted edges filtered out
  using FilteredGraph = boost::filtered_graph<Graph, edge_deleted_filter<Edge>>;
  auto filter = edge_deleted_filter{deleted_edge_map};
  const auto filtered_G = filtered_graph(G, filter);

  auto index = get(vertex_index, filtered_G);
  auto predecessor_vec = std::vector<Vertex>(num_vertices(G), s);
  auto predecessor = make_iterator_property_map(predecessor_vec.begin(), index);
  auto distance_vec = std::vector<Length>(num_vertices(G));
  auto distance = make_iterator_property_map(distance_vec.begin(), index);

  auto rev_G = make_reverse_graph(filtered_G);
  using RevEdge =
      typename graph_traits<reverse_graph<FilteredGraph>>::edge_descriptor;
  auto rev_weight = make_function_property_map<RevEdge, Length>(
      reverse_weight_functor{weight, filtered_G, rev_G});
  auto rev_index = get(vertex_index, rev_G);

  try {
    bidirectional_dijkstra(filtered_G, s, t, predecessor, distance, weight,
                           rev_G, rev_weight, rev_index);
  } catch (target_not_found &) {
    // In case t could not be found return empty optional
    return std::optional<std::vector<Edge>>{};
  }

  auto edge_list = build_edge_list_from_dijkstra(G, s, t, predecessor);
  return std::make_optional(edge_list);
}

template <typename Graph, typename WeightMap, typename AStarHeuristic,
          typename DeletedEdgeMap, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
constexpr std::function<std::optional<std::vector<Edge>>(
    const Graph &, Vertex, Vertex, const WeightMap &, DeletedEdgeMap &)>
build_shortest_path_fn(routing_kernels algorithm, const Graph &, Vertex, Vertex,
                       const WeightMap &, const AStarHeuristic &heuristic,
                       DeletedEdgeMap &) {
  switch (algorithm) {
  case routing_kernels::astar:
    return [&heuristic](const auto &G, auto s, auto t, const auto &weight,
                        auto &deleted_edge_map) {
      return astar_shortest_path(G, s, t, weight, heuristic, deleted_edge_map);
    };
  default:
    throw std::invalid_argument{"Invalid algorithm. Only [astar] "
                                "allowed."};
  }
}

template <typename Graph, typename WeightMap, typename DeletedEdgeMap,
          typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
constexpr std::function<std::optional<std::vector<Edge>>(
    const Graph &, Vertex, Vertex, const WeightMap &, DeletedEdgeMap &)>
build_shortest_path_fn(routing_kernels algorithm, const Graph &, Vertex, Vertex,
                       const WeightMap &, DeletedEdgeMap &) {
  switch (algorithm) {
  case routing_kernels::dijkstra:
    return [](const auto &G, auto s, auto t, const auto &weight,
              auto &deleted_edge_map) {
      return dijkstra_shortest_path(G, s, t, weight, deleted_edge_map);
    };
  case routing_kernels::bidirectional_dijkstra:
    return [](const auto &G, auto s, auto t, const auto &weight,
              auto &deleted_edge_map) {
      return bidirectional_dijkstra_shortest_path(G, s, t, weight,
                                                  deleted_edge_map);
    };
  default:
    throw std::invalid_argument{
        "Invalid algorithm. Only [dijkstra|bidirectional_dijkstra] "
        "allowed."};
  }
}

/**
 * Computes the ESX priority of an edge. Quoting the reference paper:
 *
 * <blockquote>Given an edge e(a, b) on some alternative path p, let E_inc(a) be
 * the set of all incoming edges e(n_i, a) to a from some node n_i in N\{b} and
 * E_out(b) be the set of all outgoing edges e(b, n_j) from b to some node n_j
 * in N\{a}. First, ESX computes the set P_s which contains the shortest paths
 * from every node n_i in E_inc(a) to every node n_j in E_out(b). Then, ESX
 * defines the set P'_s which contains all paths p in P'_s that cross edge e.
 * Finally, ESX assigns a priority to edge e, denoted by prio(e), which is set
 * to cardinality of P'_s.</blockquote>
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam AStarHeuristic
 * @tparam DeletedEdgeMap  an edge_deleted_filter::DeletedEdgeMap
 * @param G The graph
 * @param e An edge of @p G
 * @param heuristic An A* heuristic to use in performing shortest paths search.
 * @param deleted_edge_map The set of edges to filter from @p G
 * @return The priority of @p e.
 */
template <typename Graph, typename WeightMap, typename DeletedEdgeMap,
          typename Edge = edge_of_t<Graph>,
          typename Length = length_of_t<Graph>>
int compute_priority(const Graph &G, const Edge &e, WeightMap &weight,
                     const DeletedEdgeMap &deleted_edge_map) {
  using namespace boost;
  using Vertex = typename graph_traits<Graph>::vertex_descriptor;
  int priority = 0;
  auto sources = std::vector<Vertex>{};
  auto targets = std::vector<Vertex>{};

  auto a = source(e, G);
  auto b = target(e, G);

  // Compute all the n_i nodes s.t. (n_i, a) is an incoming edge of a, with n_i
  // != b
  for (auto in_it = in_edges(a, G).first; in_it != in_edges(a, G).second;
       ++in_it) {
    auto n_i = source(*in_it, G);
    if (n_i != b) {
      sources.push_back(n_i);
    }
  }

  // Compute all the n_j nodes s.t. (b, n_j) is an outgoing edge of b, with n_j
  // != a
  for (auto out_it = out_edges(b, G).first; out_it != out_edges(b, G).second;
       ++out_it) {
    auto n_j = target(*out_it, G);
    if (n_j != a) {
      targets.push_back(n_j);
    }
  }

  // Get a graph with deleted edges filtered out
  auto filter = edge_deleted_filter{deleted_edge_map};
  const auto filtered_G = filtered_graph(G, filter);

  for (auto s_i : sources) {
    for (auto t_i : targets) {
      // Compute the shortest path from s_i to t_i
      auto index = get(vertex_index, filtered_G);
      auto predecessor_vec = std::vector<Vertex>(num_vertices(G), s_i);
      auto predecessor =
          make_iterator_property_map(predecessor_vec.begin(), index);
      auto distance_vec = std::vector<Length>(num_vertices(G));
      auto distance = make_iterator_property_map(distance_vec.begin(), index);

      auto rev_G = make_reverse_graph(filtered_G);
      using RevEdge =
          typename graph_traits<reverse_graph<Graph>>::edge_descriptor;
      auto rev_weight = make_function_property_map<RevEdge>(
          reverse_weight_functor{weight, filtered_G, rev_G});
      auto rev_index = get(vertex_index, rev_G);

      try {
        bidirectional_dijkstra(filtered_G, s_i, t_i, predecessor, distance,
                               weight, rev_G, rev_weight, rev_index);
        if (shortest_path_contains_edge(s_i, t_i, e, filtered_G, predecessor)) {
          ++priority;
        }
      } catch (target_not_found &ex) {
        // In case t could not be found do nothing
        std::cout << "Caught exception: " << ex.what() << "\n";
      }
    }
  }

  return priority;
}

/**
 * Computes the edge priorities of an alternative path.
 *
 * @pre @p edge_priorities is a vector of std::priority_queue of size al least
 * @p alt_index + 1.
 *
 * @post @c edge_priorities[alt_index] queue is filled with pairs <tt>(e,
 * priority(e))</tt>
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam PrioritiesVector a std::vector<std::priority_queue<std::pair<Edge,
 * Priority>>>
 * @tparam AStarHeuristic
 * @tparam EdgeMap an edge_deleted_filter::DeletedEdgeMap
 * @param alternative The alternative path
 * @param edge_priorities The edge priorities vector
 * @param alt_index The index of @p edge_priorities where to store priorities
 * at.
 * @param G The graph
 * @param heuristic An A* heuristic to use in performing shortest paths search.
 * @param deleted_edges The set of edges to filter from @p G
 */
template <typename Graph, typename PrioritiesVector, typename WeightMap,
          typename EdgeMap,
          typename Index = typename PrioritiesVector::size_type>
void init_edge_priorities(const Graph &alternative,
                          PrioritiesVector &edge_priorities, Index alt_index,
                          const Graph &G, const WeightMap &weight,
                          const EdgeMap &deleted_edges) {
  using namespace boost;
  for (auto it = edges(alternative).first; it != edges(alternative).second;
       ++it) {
    // Get a reference to (u, v) in G
    auto u = source(*it, alternative);
    auto v = target(*it, alternative);
    auto edge_in_G = edge(u, v, G);
    assert(edge_in_G.second); // (u, v) must exist in G
    auto prio_e_i = compute_priority(G, edge_in_G.first, weight, deleted_edges);
    edge_priorities[alt_index].push(std::make_pair(edge_in_G.first, prio_e_i));
  }
}

/**
 * Computes the edge priorities of an alternative path.
 *
 * @pre @p edge_priorities is a vector of std::priority_queue of size al least
 * @p alt_index + 1.
 *
 * @post @c edge_priorities[alt_index] queue is filled with pairs <tt>(e,
 * priority(e))</tt>
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam PrioritiesVector a std::vector<std::priority_queue<std::pair<Edge,
 * Priority>>>
 * @tparam AStarHeuristic
 * @tparam EdgeMap an edge_deleted_filter::DeletedEdgeMap
 * @param alternative The alternative path
 * @param edge_priorities The edge priorities vector
 * @param alt_index The index of @p edge_priorities where to store priorities
 * at.
 * @param G The graph
 * @param heuristic An A* heuristic to use in performing shortest paths search.
 * @param deleted_edges The set of edges to filter from @p G
 */
template <typename PrioritiesVector, typename Graph, typename WeightMap,
          typename EdgeMap, typename Edge = edge_of_t<Graph>,
          typename Index = typename PrioritiesVector::size_type>
void init_edge_priorities(const std::vector<Edge> &alternative,
                          PrioritiesVector &edge_priorities, Index alt_index,
                          const Graph &G, const WeightMap &weight,
                          const EdgeMap &deleted_edges) {
  for (const auto &e : alternative) {
    auto prio_e_i = compute_priority(G, e, weight, deleted_edges);
    edge_priorities[alt_index].push(std::make_pair(e, prio_e_i));
  }
}

/**
 * Checks whether another alternative path can be found by ESX.
 *
 * @param overlaps The vector of overlapping factors between @c path_tmp and the
 * alternative paths.
 * @return true if a solution can still be found.
 * @return false otherwise.
 */
bool check_feasibility(const std::vector<double> &overlaps);

/**
 * Checks whether a candidate path satisfies the condition to add it the
 * the alternative paths set. That is: @f$Sim(candidate, p_i) < \theta, \forall
 * p_i \in AlternativePaths@f$
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @param candidate The candidate path.
 * @param alternatives The list of alternative paths.
 * @param theta The similarity threshold.
 * @return true If @p candidate is sufficiently dissimilar to all the other
 * alternative paths.
 * @return false Otherwise.
 */
template <typename Edge, typename WeightMap>
bool check_candidate_validity(
    const std::vector<Edge> &candidate,
    const std::vector<std::unordered_set<Edge, boost::hash<Edge>>>
        &alternatives,
    WeightMap const &weight, double theta) {
  bool candidate_is_valid = true;
  for (const auto &alt_path : alternatives) {
    if (compute_similarity(candidate, alt_path, weight) > theta) {
      candidate_is_valid = false;
      break;
    }
  }
  return candidate_is_valid;
}

template <typename Edge>
void move_to_dnr(Edge e,
                 std::unordered_set<Edge, boost::hash<Edge>> &deleted_edges,
                 std::unordered_set<Edge, boost::hash<Edge>> &dnr_edges) {
#ifndef NDEBUG
  auto old_size = deleted_edges.size();
#endif
  deleted_edges.erase(e); // Reinsert e_tmp into G
#ifndef NDEBUG
  assert(deleted_edges.size() + 1 == old_size);
#endif
  dnr_edges.insert(e); // Mark e_tmp as do_not_remove
}

//===----------------------------------------------------------------------===//
//                             ESX implementation
//===----------------------------------------------------------------------===//
template <typename Graph, typename WeightMap, typename MultiPredecessorMap,
          typename PriorityFunc, typename RoutingKernel, typename Terminator,
          typename Vertex = vertex_of_t<Graph>>
void esx(const Graph &G, WeightMap const &weight,
         MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k,
         double theta, PriorityFunc &&priority_fn,
         RoutingKernel &routing_kernel, Terminator &&terminator) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<WeightMap, Edge>));

  // P_LO set of k paths
  auto resPathsEdges = std::vector<std::vector<Edge>>{};
  auto resEdges = std::vector<std::unordered_set<Edge, boost::hash<Edge>>>{};

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<WeightMap, Edge>));

  // P_LO set of k paths
  auto resPaths = std::vector<Path<Graph>>{};
  // Compute shortest path from s to t
  auto sp = details::compute_shortest_path(G, weight, s, t);
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

  // Every max-heap H_i is associated with p_i
  using Priority = std::pair<Edge, int>;
  using EdgePriorityQueue =
      std::priority_queue<Priority, std::vector<Priority>,
                          details::EdgePriorityComparator<Edge>>;
  auto edge_priorities = std::vector<EdgePriorityQueue>(k);

  // We keep a set of non-removable edges
  auto dnr_edges = std::unordered_set<Edge, boost::hash<Edge>>{};

  // We keep a set of deleted-edges
  auto deleted_edges = std::unordered_set<Edge, boost::hash<Edge>>{};

  // Compute lower bounds for AStar
  // auto heuristic = details::distance_heuristic<Graph, Length>(G, t);

  // Initialize max-heap H_0 with the priority of each edge of the shortest path
  priority_fn(*sp, edge_priorities, 0, G, weight, deleted_edges);

  auto overlaps = std::vector<double>(k, 0.0);
  overlaps[0] = 1.0; // Set p_c overlap with sp (itself) to 1

  bool still_feasible = true;
  while (resPathsEdges.size() < static_cast<std::size_t>(k) && still_feasible) {
    std::ptrdiff_t p_max_idx = 0;
    double overlap_ratio = 1.0;

    while (overlap_ratio >= theta) {
      // Get the max overlapping path
      auto max_it = std::max_element(std::begin(overlaps), std::end(overlaps));
      p_max_idx = max_it - std::begin(overlaps);
      overlap_ratio = *max_it;

      // Check if finding a result is feasible
      still_feasible = details::check_feasibility(overlaps);
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

      // The remainder code is the hot part of the algorithm. So we check here
      // if the algorithm should terminate
      if (terminator.should_stop()) {
        throw terminator_stop_error{
            "ESX terminated before completing due to a Terminator. Please "
            "discard partial output."};
      }

      // Compute p_tmp shortest path
      auto p_tmp = routing_kernel(G, s, t, weight, deleted_edges);

      // If shortest path did not find a path
      if (!p_tmp) {
        move_to_dnr(e_tmp, deleted_edges, dnr_edges);
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
        const auto &alt_path = resEdges[p_max_idx];
        overlaps[p_max_idx] = compute_similarity(*p_tmp, alt_path, weight);
      }

      // Checking if the resulting path is valid
      bool candidate_is_valid =
          check_candidate_validity(*p_tmp, resEdges, weight, theta);
      if (candidate_is_valid) {
        // Add p_tmp to P_LO
        resPathsEdges.emplace_back(*p_tmp);
        resEdges.emplace_back(p_tmp->begin(), p_tmp->end());

        // Set p_c overlap with itself to 1
        std::ptrdiff_t p_c_idx = resPathsEdges.size() - 1;
        overlaps[p_c_idx] = 1.0;

        // Initialize max-heap H_i with the priority of each edge of new
        // alternative path
        priority_fn(*p_tmp, edge_priorities, p_c_idx, G, weight, deleted_edges);
        break; // From inner while loop
      }
    }
  }

  // Beforer returning, populate predecessors map
  fill_multi_predecessor(resPathsEdges.begin(), resPathsEdges.end(), G,
                         predecessors);
}

template <typename Graph, typename WeightMap, typename MultiPredecessorMap,
          typename PriorityFunc, typename Terminator,
          typename Vertex = vertex_of_t<Graph>>
void esx_dispatch2(const Graph &G, WeightMap const &weight,
                   MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k,
                   double theta, PriorityFunc &&priority_fn,
                   routing_kernels algorithm, Terminator &&terminator) {
  using Edge = edge_of_t<Graph>;
  using Length = length_of_t<Graph>;

  auto deleted_edges = std::unordered_set<Edge, boost::hash<Edge>>{};
  if (algorithm == routing_kernels::astar) {
    auto heuristic = details::distance_heuristic<Graph, Length>(G, t);
    auto routing_kernel = details::build_shortest_path_fn(
        algorithm, G, s, t, weight, heuristic, deleted_edges);
    esx(G, weight, predecessors, s, t, k, theta,
        std::forward<PriorityFunc>(priority_fn), routing_kernel,
        std::forward<Terminator>(terminator));
  } else {
    auto routing_kernel = details::build_shortest_path_fn(
        algorithm, G, s, t, weight, deleted_edges);
    esx(G, weight, predecessors, s, t, k, theta,
        std::forward<PriorityFunc>(priority_fn), routing_kernel,
        std::forward<Terminator>(terminator));
  }
}

template <typename Graph, typename WeightMap, typename MultiPredecessorMap,
          typename Terminator, typename Vertex = vertex_of_t<Graph>>
void esx_dispatch(const Graph &G, WeightMap const &weight,
                  MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k,
                  double theta, routing_kernels algorithm,
                  Terminator &&terminator) {
  auto priority_fn = [](auto const &alternative, auto &edge_priorities,
                        auto alt_index, auto const &G, auto const &weight,
                        auto const &deleted_edges) {
    init_edge_priorities(alternative, edge_priorities, alt_index, G, weight,
                         deleted_edges);
  };
  esx_dispatch2(G, weight, predecessors, s, t, k, theta, std::move(priority_fn),
                algorithm, std::forward<Terminator>(terminator));
}

template <typename Graph, typename WeightMap, typename MultiPredecessorMap,
          typename EdgeCentralityMap, typename Terminator,
          typename Vertex = vertex_of_t<Graph>>
void esx_dispatch(const Graph &G, WeightMap const &weight,
                  MultiPredecessorMap &predecessors,
                  EdgeCentralityMap const &edge_centrality, Vertex s, Vertex t,
                  int k, double theta, routing_kernels algorithm,
                  Terminator &&terminator) {
  auto priority_fn = [&edge_centrality](auto const &alternative,
                                        auto &edge_priorities, auto alt_index,
                                        auto const &, auto const &,
                                        auto const &) {
    for (const auto &e : alternative) {
      auto prio_e_i = get(edge_centrality, e);
      edge_priorities[alt_index].push(std::make_pair(e, prio_e_i));
    }
  };

  esx_dispatch2(G, weight, predecessors, s, t, k, theta, std::move(priority_fn),
                algorithm, std::forward<Terminator>(terminator));
}
} // namespace details
} // namespace arlib
#endif
