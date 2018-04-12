#ifndef KSPWLO_IMPL_HPP
#define KSPWLO_IMPL_HPP

#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/graph_types.hpp"
#include "kspwlo/impl/kspwlo_impl.hpp"

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
  explicit edge_deleted_filter(const DeletedEdgeMap &deleted_edge_map)
      : deleted_edge_map{deleted_edge_map} {};

  template <typename Edge> bool operator()(const Edge &e) const {
    return deleted_edge_map.find(e) == std::end(deleted_edge_map);
  }

private:
  const DeletedEdgeMap &deleted_edge_map;
};

template <typename Graph, CostType>
class distance_heuristic : public boost::astar_heuristic<Graph, CostType> {
public:
  using Vertex = typename graph_traits<Graph>::vertex_descriptor;

  distance_heuristic(Graph &G, Vertex t) {
    lower_bounds = kspwlo_impl::distance_from_target(G, t);
  }

  CostType operator()(Vertex u) { return lower_bounds[u]; }

private:
  std::vector<CostType> lower_bounds;
};
//===----------------------------------------------------------------------===//
//                          ESX algorithm routines
//===----------------------------------------------------------------------===//
template <typename Graph, typename AStarHeuristic, typename DeletedEdgeMap,
          typename Edge = typename graph_traits<Graph>::edge_descriptor>
int compute_priority(Graph &G, const Edge &e, AStarHeuristic &heuristic,
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
    if (*in_it != b) {
      sources.push_back(*in_it);
    }
  }

  // Compute all the n_j nodes s.t. (b, n_j) is an outgoing edge of b, with n_j
  // != a
  for (auto out_it = out_edges(b, G).first; out_it != out_edges(b, G).second;
       ++out_it) {
    if (*out_it != a) {
      targets.push_back(*out_it);
    }
  }

  for (auto s_i : sources) {
    for (auto t_i : targets) {
      // Compute the shortest path from s_i to t_i
    }
  }
}
} // namespace kspwlo_impl
#endif