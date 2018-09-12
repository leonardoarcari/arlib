#ifndef BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_IMPL_HPP
#define BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_IMPL_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/property_map/property_map.hpp>

#include "kspwlo/bidirectional_dijkstra.hpp"
#include "kspwlo/graph_types.hpp"

#include <functional>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @brief Implementations details of kSPwLO algorithms
 */
namespace kspwlo_impl {
//===----------------------------------------------------------------------===//
//                      Penalty algorithm types
//===----------------------------------------------------------------------===//
/**
 * @brief A map from edges to the number of time they have been penalized so
 * far.
 *
 * @tparam Edge An edge_descriptor
 */
template <typename Edge>
using PenBoundsMap = std::unordered_map<Edge, int, boost::hash<Edge>>;

/**
 * @brief A map from edges to their weights
 *
 * @tparam Edge An edge_descriptor
 * @tparam Length The edge weight type
 */
template <typename Edge, typename Length>
using WeightMap = std::unordered_map<Edge, Length, boost::hash<Edge>>;

/**
 * @brief A vector for tracking distance of a vertex from the source.
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
 * @brief A functor to return the penalized weight of an edge, to avoid changing
 * the original graph weights.
 *
 * @tparam PMap A Weight Property Map.
 */
template <typename PMap> class penalty_functor {
public:
  using Edge = typename boost::property_traits<PMap>::key_type;
  using Length = typename boost::property_traits<PMap>::value_type;

  /**
   * @brief Construct a new penalty functor object.
   *
   * @tparam EdgeIterator An iterator of the graph edges.
   * @param weight The weight property map.
   */
  template <typename EdgeIterator>
  penalty_functor(PMap weight, EdgeIterator first, EdgeIterator last)
      : weight{weight}, penalties{} {}

  /**
   * @brief Copy constructor.
   *
   * @param other The penalty functor to copy from
   */
  penalty_functor(const penalty_functor<PMap> &other)
      : weight{other.weight}, penalties{other.penalties} {}

  /**
   * @brief Returns the penalized weight of an edge.
   *
   * @param e The query edge.
   * @return The penalized weight for @p e.
   */
  const Length &operator()(const Edge &e) const { return get_or_insert(e); }

  /**
   * @brief Returns the penalized weight of an edge.
   *
   * @param e The query edge.
   * @return The penalized weight for @p e.
   */
  Length &operator[](const Edge &e) { return get_or_insert(e); }

  /**
   * @brief Returns the penalized weight of an edge.
   *
   * @param e The query edge.
   * @return The penalized weight for @p e.
   */
  const Length &operator[](const Edge &e) const { return get_or_insert(e); }

private:
  PMap weight;
  mutable WeightMap<Edge, Length> penalties;

  Length &get_or_insert(const Edge &e) const {
    if (auto search = penalties.find(e); search != penalties.end()) {
      return search->second;
    } else {
      auto [it, ok] = penalties.insert({e, weight[e]});
      assert(ok && "[kspwlo::penalty_functor] Could not insert edge weight");
      return it->second;
    }
  }
};

template <typename PMap, typename Graph> class reverse_penalty_functor {
public:
  using Edge = typename boost::graph_traits<
      boost::reverse_graph<Graph>>::edge_descriptor;
  using Length = typename boost::property_traits<PMap>::value_type;

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

template <
    typename Graph, typename PMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
constexpr std::function<std::optional<std::vector<kspwlo::Edge>>(
    const Graph &, Vertex, Vertex, penalty_functor<PMap> &)>
build_shortest_path_fn(kspwlo::shortest_path_algorithm algorithm, const Graph &,
                       const PMap &) {
  switch (algorithm) {
  case kspwlo::shortest_path_algorithm::dijkstra:
    return [](const auto &G, auto s, auto t, auto &penalty) {
      return dijkstra_shortest_path(G, s, t, penalty);
    };
  case kspwlo::shortest_path_algorithm::bidirectional_dijkstra:
    return [](const auto &G, auto s, auto t, auto &penalty) {
      return bidirectional_dijkstra_shortest_path(G, s, t, penalty);
    };
  default:
    throw std::invalid_argument{
        "Invalid algorithm. Only [dijkstra|bidirectional_dijkstra] allowed."};
  }
}

/**
 * @brief Computes the shortest path between two vertices s and t, first
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
template <
    typename Graph,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Length =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
std::optional<std::vector<kspwlo::Edge>>
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
  auto edge_list = build_edge_list_from_dijkstra(s, t, predecessor);

  // Backward step
  auto rev_G = make_reverse_graph(G);
  dijkstra_shortest_paths(rev_G, t, distance_map(&distance_t[0]));

  if (exists_path_to<Length>(t, distance_s)) {
    return std::make_optional(edge_list);
  } else {
    // In case t could not be found from astar_search and target_found is not
    // thrown, return empty optional
    return std::optional<std::vector<kspwlo::Edge>>{};
  }

} // namespace kspwlo_impl

/**
 * @brief Computes the Dijkstra shortest path from s to t using a
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
template <
    typename Graph, typename PMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Edge = typename boost::graph_traits<Graph>::edge_descriptor,
    typename Length =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
std::optional<std::vector<kspwlo::Edge>>
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
    auto edge_list = build_edge_list_from_dijkstra(s, t, predecessor);
    return std::make_optional(edge_list);
  }

  // In case t could not be found from astar_search and target_found is not
  // thrown, return empty optional
  return std::optional<std::vector<kspwlo::Edge>>{};
}

template <
    typename Graph, typename PMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Edge = typename boost::graph_traits<Graph>::edge_descriptor,
    typename Length =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
std::optional<std::vector<kspwlo::Edge>>
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
  } catch (kspwlo_impl::target_not_found &) {
    // In case t could not be found return empty optional
    return std::optional<std::vector<kspwlo::Edge>>{};
  }

  auto edge_list = build_edge_list_from_dijkstra(s, t, predecessor);
  return std::make_optional(edge_list);
}

/**
 * @brief Apply penalization step to the candidate path.
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
template <
    typename Graph, typename DistanceMap, typename PMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Edge = typename boost::graph_traits<Graph>::edge_descriptor,
    typename Length =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
void penalize_candidate_path(const std::vector<kspwlo::Edge> &candidate,
                             const Graph &G, Vertex s, Vertex t, double p,
                             double r, penalty_functor<PMap> &penalty,
                             const DistanceMap &distance_s,
                             const DistanceMap &distance_t,
                             PenBoundsMap<Edge> &penalty_bounds,
                             int bound_limit) {
  using namespace boost;

  // Keep track of candidate vertices to exclude them from incoming/outgoing
  // edges update
  auto candidate_vertices = std::unordered_set<Vertex, boost::hash<Vertex>>{};
  auto candidate_edges = std::unordered_set<Edge, boost::hash<Edge>>{};

  for (const auto &[u_c, v_c] : candidate) {
    auto [e, is_valid] = edge(u_c, v_c, G);
    assert(is_valid);
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
} // namespace kspwlo_impl

#endif