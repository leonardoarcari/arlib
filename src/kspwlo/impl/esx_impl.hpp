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

namespace kspwlo_impl {
//===----------------------------------------------------------------------===//
//                      ESX algorithm support classes
//===----------------------------------------------------------------------===//
template <typename Edge> struct EdgePriorityComparator {
  using Priority = std::pair<Edge, int>;

  bool operator()(Priority lhs, Priority rhs) {
    return lhs.second < rhs.second;
  }
};

template <typename DeletedEdgeMap> class edge_deleted_filter {
public:
  edge_deleted_filter() : deleted_edge_map{nullptr} {}
  edge_deleted_filter(DeletedEdgeMap &deleted_edge_map)
      : deleted_edge_map{std::addressof(deleted_edge_map)} {};

  template <typename Edge> bool operator()(const Edge &e) const {
    return deleted_edge_map->find(e) == std::end(*deleted_edge_map);
  }

private:
  DeletedEdgeMap *deleted_edge_map;
};

template <typename Graph, typename CostType>
class distance_heuristic : public boost::astar_heuristic<Graph, CostType> {
public:
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;

  distance_heuristic(Graph &G, Vertex t) {
    lower_bounds = kspwlo_impl::distance_from_target(G, t);
  }

  CostType operator()(Vertex u) { return lower_bounds[u]; }

private:
  std::vector<CostType> lower_bounds;
};

struct target_found {};

template <typename Vertex>
class astar_target_visitor : public boost::default_astar_visitor {
public:
  explicit astar_target_visitor(Vertex t) : t{t} {}
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
template <typename CostType, typename Vertex, typename DistMap>
bool exists_path_to(Vertex v, const DistMap &dist) {
  auto inf = std::numeric_limits<CostType>::max();
  return dist[v] != inf;
}

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

template <typename Graph, typename AStarHeuristic, typename DeletedEdgeMap,
          typename Edge = typename boost::graph_traits<Graph>::edge_descriptor>
int compute_priority(Graph &G, const Edge &e, const AStarHeuristic &heuristic,
                     DeletedEdgeMap &deleted_edge_map) {
  using namespace boost;
  using Vertex = typename graph_traits<Graph>::vertex_descriptor;
  using Length = typename property_traits<
      typename property_map<Graph, edge_weight_t>::type>::value_type;
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

template <typename Graph>
double compute_similarity(const std::vector<kspwlo::Edge> &candidate,
                          const kspwlo::Path<Graph> &alt_path) {
  double shared_length =
      static_cast<double>(compute_length_from_edges(candidate, alt_path.graph));

  return shared_length / alt_path.length;
}
} // namespace kspwlo_impl
#endif