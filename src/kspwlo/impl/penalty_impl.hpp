#ifndef BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_IMPL_HPP
#define BOOST_PENALTY_BASED_ALTERNATIVE_ROUTING_IMPL_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
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
// template <typename Graph,
//           typename Edge = typename
//           boost::graph_traits<Graph>::edge_descriptor, typename Length =
//               typename boost::property_traits<typename boost::property_map<
//                   Graph, boost::edge_weight_t>::type>::value_type>
//     class reverse_weights_property_map
//     : public boost::put_get_helper <
//       typename WeightMap<Edge, Length>::value_type::second_type &,
//     associative_property_map<WeightMap<Edge, Length>> {
//   using namespace boost;
//   using C = WeightMap<Edge, Length>;
//   using RevG = reverse_graph<Graph, Graph &>

//       public : using key_type = typename C::key_type;
//   using value_type = typename C::value_type::second_type;
//   using reference = value_type &;
//   using category = lvalue_property_map_tag;

//   reverse_associative_property_map() : c{nullptr}, G{nullptr}, rev_g{nullptr}
//   {} reverse_associative_property_map(C &c, Graph &G, RevG &rev_g)
//       : c{std::addreof(c)}, G{std::addresof(G)}, rev_G{std::addressof(rev_g)}
//       {}

//   reference operator[](const key_type &k) const {
//     auto u = source(k, rev_g);
//     auto v = target(k, rev_g);

//   }

// private:
//   C *c;
//   Graph *G;
//   RevG *rev_g;
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
dijkstra_shortest_path_two_ways(Graph &G, Vertex s, Vertex t,
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
    typename Graph,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Edge = typename boost::graph_traits<Graph>::edge_descriptor,
    typename Length =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
std::optional<std::vector<kspwlo::Edge>>
dijkstra_shortest_path(Graph &G, Vertex s, Vertex t,
                       WeightMap<Edge, Length> &weight) {
  using namespace boost;

  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);

  // Forward step
  try {
    dijkstra_shortest_paths(
        G, s,
        weight_map(make_assoc_property_map(weight))
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
    typename Graph, typename DistanceMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Edge = typename boost::graph_traits<Graph>::edge_descriptor,
    typename Length =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
void penalize_candidate_path(const std::vector<kspwlo::Edge> &candidate,
                             const Graph &G, Vertex s, Vertex t, double p,
                             double r, WeightMap<Edge, Length> &weight,
                             DistanceMap &distance_s, DistanceMap &distance_t,
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
        weight[e] += p * weight[e];
        ++penalty_bounds[e];
      }
    } else { // Penalize and create a nb_updates counter for e
      weight[e] += p * weight[e];
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
            weight[*it] += (0.1 + r * closeness) * weight[*it];
            ++penalty_bounds[*it];
          }
        } else {
          // Else, just update it
          auto closeness = distance_t[u] / distance_t[s];
          weight[*it] += (0.1 + r * closeness) * weight[*it];
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
            weight[*it] += (0.1 + r * closeness) * weight[*it];
            ++penalty_bounds[*it];
          }
        } else {
          // Else, just update it
          auto closeness = distance_s[v] / distance_s[t];
          weight[*it] += (0.1 + r * closeness) * weight[*it];
        }
      }
    }
  }
}
} // namespace kspwlo_impl

#endif