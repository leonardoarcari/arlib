#ifndef BOOST_EDGE_SUBSET_EXCLUSION_IMPL_HPP
#define BOOST_EDGE_SUBSET_EXCLUSION_IMPL_HPP

#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/graph_types.hpp"
#include "kspwlo/impl/kspwlo_impl.hpp"

#include <limits>
#include <utility>

/**
 * @brief Implementations details of kSPwLO algorithms
 */
namespace kspwlo_impl {

//===----------------------------------------------------------------------===//
//                      ESX algorithm support classes
//===----------------------------------------------------------------------===//

/**
 * @brief Comparator for edges priority queue.
 *
 * Given two edges @c e1 and @c e2, if <tt>prio(e1) > prio(e2)</tt> then @c e1
 * must be popped before @c e2.
 *
 * @tparam Edge A Boost::Graph edge descriptor.
 */
template <typename Edge> struct EdgePriorityComparator {
  using Priority = std::pair<Edge, int>;

  /**
   * @brief Given two edges @c e1 and @c e2, if <tt>prio(e1) > prio(e2)</tt>
   * then @c e1 must be popped before @c e2.
   *
   * @param lhs first edge and its priority.
   * @param rhs second edge and its priority.
   * @return true if @p lhs has lower priority then @p rhs
   * @return false otherwise.
   */
  bool operator()(Priority lhs, Priority rhs) {
    return lhs.second < rhs.second;
  }
};

/**
 * @brief A filter functor for Boost::filtered_graph to hide edges deleted by
 * ESX.
 *
 * @tparam Edge A Boost::Graph edge descriptor
 */
template <typename Edge> class edge_deleted_filter {
public:
  /**
   * @brief The deleted edges map.
   */
  using DeletedEdgeMap = std::unordered_set<Edge, boost::hash<Edge>>;
  /**
   * @brief Empty constructor, required by Boost::Graph
   */
  edge_deleted_filter() : deleted_edge_map{nullptr} {}

  /**
   * @brief Construct a new edge_deleted_filter object filtering edges contained
   * in @p deleted_edge_map.
   *
   * @param deleted_edge_map The set of deleted edges.
   */
  edge_deleted_filter(const DeletedEdgeMap &deleted_edge_map)
      : deleted_edge_map{std::addressof(deleted_edge_map)} {};

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

/**
 * @brief An A* heuristic using <em>distance from target</em> lower bound.
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
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;

  /**
   * @brief Construct a new distance heuristic object. Upon construction a
   * Dijkstra’s algorithm is run on @c G' from @p t to all nodes. @c G' is equal
   * to @p G except that all the edges are reversed.
   *
   * @param G The Graph on which to search
   * @param t The target vertex
   */
  distance_heuristic(Graph &G, Vertex t) {
    lower_bounds = kspwlo_impl::distance_from_target(G, t);
  }

  /**
   * @param u The Vertex
   * @return The heuristic of the cost of Vertex @p u.
   */
  CostType operator()(Vertex u) { return lower_bounds[u]; }

private:
  std::vector<CostType> lower_bounds;
};

/**
 * @brief An A* visitor to stop the algorithm when a target vertex is found.
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
   * @brief Construct a new astar_target_visitor object that ends the search
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
//                          ESX algorithm routines
//===----------------------------------------------------------------------===//

/**
 * @brief Checks whether the path from @p s to @p t contains edge @p e or not.
 *
 * @pre @p predecessor is the PredecessorMap filled by a Boost::Graph shortest
 * path algorithm such that <tt>predecessor[s] == s</tt>
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
template <
    typename Graph, typename PredMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Edge = typename boost::graph_traits<Graph>::edge_descriptor>
bool shortest_path_contains_edge(Vertex s, Vertex t, Edge e, Graph &G,
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

/**
 * @brief Builds a vector of kspwlo::Edge out of a PredecessorMap computed by a
 * shortest path algorithm from @p s to @p t
 *
 * @tparam Vertex
 * @tparam PredecessorMap
 * @param s The source vertex
 * @param t The target vertex
 * @param p The PredecessorMap
 * @return A vector of the shortest path edges from @p s to @p t.
 */
template <typename Vertex, typename PredecessorMap>
std::vector<kspwlo::Edge>
build_edge_list_from_dijkstra(Vertex s, Vertex t, const PredecessorMap &p) {
  auto edge_list = std::vector<kspwlo::Edge>{};

  auto current = t;
  while (current != s) {
    auto u = p[current];
    edge_list.emplace_back(u, current);
    current = u;
  }

  return edge_list;
}

/**
 * @brief Compute a shortest path between two vertices on a filtered graph using
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
template <
    typename Graph, typename AStarHeuristic, typename DeletedEdgeMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
std::optional<std::vector<kspwlo::Edge>>
astar_shortest_path(Graph &G, Vertex s, Vertex t,
                    const AStarHeuristic &heuristic,
                    DeletedEdgeMap &deleted_edge_map) {
  using namespace boost;
  using Length = typename property_traits<
      typename property_map<Graph, edge_weight_t>::type>::value_type;

  // Get a graph with deleted edges filtered out
  auto filter = edge_deleted_filter{deleted_edge_map};
  auto filtered_G = filtered_graph(G, filter);

  auto dist = std::vector<Length>(num_vertices(G));
  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, filtered_G);

  try {
    astar_search(filtered_G, s, heuristic,
                 distance_map(&dist[0])
                     .predecessor_map(make_iterator_property_map(
                         std::begin(predecessor), vertex_id, s))
                     .visitor(astar_target_visitor{t}));
  } catch (target_found tf) {
    auto edge_list = build_edge_list_from_dijkstra(s, t, predecessor);
    return std::make_optional(edge_list);
  }
  // In case t could not be found from astar_search and target_found is not
  // thrown, return empty optional
  return std::optional<std::vector<kspwlo::Edge>>{};
}

/**
 * @brief Computes the ESX priority of an edge. Quoting the reference paper:
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
template <typename Graph, typename AStarHeuristic, typename DeletedEdgeMap,
          typename Edge = typename boost::graph_traits<Graph>::edge_descriptor>
int compute_priority(Graph &G, const Edge &e, const AStarHeuristic &heuristic,
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
  auto filtered_G = filtered_graph(G, filter);

  for (auto s_i : sources) {
    for (auto t_i : targets) {
      // Compute the shortest path from s_i to t_i
      auto predecessor = std::vector<Vertex>(num_vertices(G), s_i);
      auto vertex_id = get(vertex_index, filtered_G);

      try {
        astar_search(
            filtered_G, s_i, heuristic,
            predecessor_map(make_iterator_property_map(std::begin(predecessor),
                                                       vertex_id, s_i))
                .visitor(astar_target_visitor{t_i}));
      } catch (target_found tf) {
        if (shortest_path_contains_edge(s_i, t_i, e, filtered_G, predecessor)) {
          ++priority;
        }
      }
    }
  }

  return priority;
}

/**
 * @brief Computes the length of the path in @p candidate using weights from @p
 * G
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @param candidate The candidate path
 * @param G The graph
 * @return The length of @p candidate, i.e. @f$\sum_{e \in candidate} weight(e,
 *         G)@f$
 */
template <typename Graph,
          typename Length =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
Length compute_length_from_edges(const std::vector<kspwlo::Edge> &candidate,
                                 const Graph &G) {
  using namespace boost;
  Length length = 0;
  auto weight = get(edge_weight, G);

  for (const auto & [ u, v ] : candidate) {
    auto egde_in_G = edge(u, v, G);
    bool edge_is_shared = egde_in_G.second;

    if (edge_is_shared) {
      length += weight[egde_in_G.first];
    }
  }

  return length;
}

/**
 * @brief Evaluates the similarity of a candidate path with respect to some
 * alterative path.
 *
 * This measure of similarity is the one from the reference paper, that is: let
 * @c p' be the candidate path and @c p some alternative path, then
 * @f[
 *   Sim(p', p) = \frac{\sum_{\left(n_x,n_y\right) \in p'\cap p} w_xy}{l(p)}
 * @f]
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @param candidate The candidate path @c p'
 * @param alt_path The alternative path @c p
 * @return The similarity between @c p' and @c p
 */
template <typename Graph>
double compute_similarity(const std::vector<kspwlo::Edge> &candidate,
                          const kspwlo::Path<Graph> &alt_path) {
  double shared_length =
      static_cast<double>(compute_length_from_edges(candidate, alt_path.graph));

  return shared_length / alt_path.length;
}

/**
 * @brief Computes the edge priorities of an alternative path.
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
template <typename Graph, typename PrioritiesVector, typename AStarHeuristic,
          typename EdgeMap,
          typename Index = typename PrioritiesVector::size_type>
void init_edge_priorities(const Graph &alternative,
                          PrioritiesVector &edge_priorities, Index alt_index,
                          const Graph &G, const AStarHeuristic &heuristic,
                          const EdgeMap &deleted_edges) {
  using namespace boost;
  for (auto it = edges(alternative).first; it != edges(alternative).second;
       ++it) {
    // Get a reference to (u, v) in G
    auto u = source(*it, alternative);
    auto v = target(*it, alternative);
    auto edge_in_G = edge(u, v, G);
    assert(edge_in_G.second); // (u, v) must exist in G
    auto prio_e_i =
        compute_priority(G, edge_in_G.first, heuristic, deleted_edges);
    edge_priorities[alt_index].push(std::make_pair(edge_in_G.first, prio_e_i));
  }
}

/**
 * @brief Computes the edge priorities of an alternative path.
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
template <typename PrioritiesVector, typename Graph, typename AStarHeuristic,
          typename EdgeMap,
          typename Index = typename PrioritiesVector::size_type>
void init_edge_priorities(const std::vector<kspwlo::Edge> &alternative,
                          PrioritiesVector &edge_priorities, Index alt_index,
                          const Graph &G, const AStarHeuristic &heuristic,
                          const EdgeMap &deleted_edges) {
  for (const auto & [ u, v ] : alternative) {
    auto edge_in_G = edge(u, v, G).first;
    auto prio_e_i = compute_priority(G, edge_in_G, heuristic, deleted_edges);
    edge_priorities[alt_index].push(std::make_pair(edge_in_G, prio_e_i));
  }
}

/**
 * @brief Checks whether another alternative path can be found by ESX.
 *
 * @param overlaps The vector of overlapping factors between @c path_tmp and the
 * alternative paths.
 * @return true if a solution can still be found.
 * @return false otherwise.
 */
bool check_feasibility(const std::vector<double> &overlaps) {
  auto valid_overlapping =
      std::find_if(std::begin(overlaps), std::end(overlaps),
                   [](const auto &v) { return v > 0; });
  if (valid_overlapping == std::end(overlaps)) {
    return false;
  }
  return true;
}

/**
 * @brief Checks whether a candidate path satisfies the condition to add it the
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
template <typename Graph>
bool check_candidate_validity(
    const std::vector<kspwlo::Edge> &candidate,
    const std::vector<kspwlo::Path<Graph>> &alternatives, double theta) {
  bool candidate_is_valid = true;
  for (const auto &alt_path : alternatives) {
    if (compute_similarity(candidate, alt_path) > theta) {
      candidate_is_valid = false;
      break;
    }
  }
  return candidate_is_valid;
}
} // namespace kspwlo_impl
#endif