#ifndef BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_IMPL_HPP
#define BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_IMPL_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/property_map/property_map.hpp>

#include "kspwlo/graph_types.hpp"

#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace kspwlo_impl {
//===----------------------------------------------------------------------===//
//                      Penalty algorithm types
//===----------------------------------------------------------------------===//
template <typename Edge>
using PenBoundsMap = std::unordered_map<Edge, int, boost::hash<Edge>>;

template <typename Edge, typename Length>
using WeightMap = std::unordered_map<Edge, Length, boost::hash<Edge>>;

template <typename Vertex> using DistanceMap = std::vector<Vertex>;

//===----------------------------------------------------------------------===//
//                      Penalty algorithm classes
//===----------------------------------------------------------------------===//
template <typename PMap> class penalty_functor {
public:
  using Edge = typename boost::property_traits<PMap>::key_type;
  using Length = typename boost::property_traits<PMap>::value_type;

  template <typename EdgeIterator>
  penalty_functor(PMap weight, EdgeIterator first, EdgeIterator last)
      : weight{weight}, penalties{} {
    for (auto it = first; it != last; ++it) {
      penalties.insert({*it, weight[*it]});
    }
  }

  penalty_functor(const penalty_functor<PMap> &other)
      : weight{other.weight}, penalties{other.penalties} {}

  const Length &operator()(const Edge &e) const { return penalties.at(e); }

  Length &operator[](const Edge &e) { return penalties.at(e); }

  const Length &operator[](const Edge &e) const { return penalties.at(e); }

private:
  PMap weight;
  WeightMap<Edge, Length> penalties;
};

// template <typename PMap, typename Graph, typename GraphRef = const Graph &>
// class reverse_penalty_functor {
// public:
//   using Edge = typename boost::property_traits<PMap>::key_type;
//   using Length = typename boost::property_traits<PMap>::value_type;
//   using RevGraph = boost::reverse_graph<Graph, Graph &>;
//   using RevGraphRef = const RevGraph &;

//   reverse_penalty_functor(const penalty_functor<PMap> &pf, const Graph &G,
//                           const RevGraph &rev_G)
//       : pf{pf}, G{G}, rev_G{rev_G} {}

//   Length &operator()(const Edge &e) const {
//     // Get edge in original graph
//     auto e_G = get_underlying_edge(e);

//     // Forward call to underlying penalty_functor
//     return pf(e_G);
//   }

//   Length &operator[](const Edge &e) { // Get edge in original graph
//     auto e_G = get_underlying_edge(e);

//     // Forward call to underlying penalty_functor
//     return pf[e_G];
//   }

//   const Length &operator[](const Edge &e) const {
//     // Get edge in original graph
//     auto e_G = get_underlying_edge(e);

//     // Forward call to underlying penalty_functor
//     return pf[e_G];
//   }

// private:
//   Edge get_underlying_edge(const Edge &e) {
//     using namespace boost;
//     auto u = source(e, rev_G);
//     auto v = target(e, rev_G);
//     return edge(v, u, G).first;
//   }

//   const penalty_functor<PMap> &pf;
//   GraphRef G;
//   RevGraphRef rev_G;
// };
//===----------------------------------------------------------------------===//
//                     Penalty algorithm support routines
//===----------------------------------------------------------------------===//

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

  //auto weight_init = get(edge_weight, G);
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

  for (const auto & [ u_c, v_c ] : candidate) {
    auto[e, is_valid] = edge(u_c, v_c, G);
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
    for (auto[it, end] = in_edges(u, G); it != end; ++it) {
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
    for (auto[it, end] = out_edges(v, G); it != end; ++it) {
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