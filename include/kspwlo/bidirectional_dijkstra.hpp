#ifndef BOOST_BIDIRECTIONAL_DIJKSTRA_HPP
#define BOOST_BIDIRECTIONAL_DIJKSTRA_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>

#include "kspwlo/impl/bidirectional_dijkstra_impl.hpp"
#include "kspwlo/impl/kspwlo_impl.hpp"
#include "kspwlo/visitor.hpp"

#include <deque>
#include <limits>
#include <vector>

/**
 * @brief Algorithms and utilities for Boost::Graph
 */
namespace boost {
//===----------------------------------------------------------------------===//
//                   Bidirectional Dijkstra algorithm
//===----------------------------------------------------------------------===//
template <typename Vertex>
using PathMap =
    std::unordered_map<Vertex, std::vector<Vertex>, boost::hash<Vertex>>;

template <typename PredecessorMap, typename Vertex>
void fill_predecessor(PredecessorMap predecessor,
                      const std::deque<Vertex> &final_path) {
  for (std::size_t i = 0; i < final_path.size() - 1; ++i) {
    predecessor[final_path[i + 1]] = final_path[i];
  }
}

template <
    typename Graph, typename PredecessorMap, typename DistanceMap,
    typename WeightMap, typename BackGraph, typename BackPredecessorMap,
    typename BackDistanceMap, typename BackWeightMap,
    typename BiDijkstraVisitorImpl,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
void bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
                            PredecessorMap predecessor, DistanceMap distance,
                            WeightMap weight, const BackGraph &G_b,
                            BackPredecessorMap predecessor_b,
                            BackDistanceMap distance_b, BackWeightMap weight_b,
                            BiDijkstraVisitor<BiDijkstraVisitorImpl> &visitor) {
  using namespace boost;
  using Length = typename property_traits<DistanceMap>::value_type;
  constexpr Length inf = std::numeric_limits<Length>::max();

  // Initialize distance structures
  kspwlo_impl::init_distance_vector(G, distance);
  kspwlo_impl::init_distance_vector(G_b, distance_b);

  // Initialize shortest path trees (forward and backward)
  predecessor[s] = s;
  predecessor_b[t] = t;

  // Init fringe structures
  auto fringe = kspwlo_impl::Fringe<Vertex, Length>{};
  auto fringe_b = kspwlo_impl::Fringe<Vertex, Length>{};
  fringe.push(std::make_pair(s, 0));
  fringe_b.push(std::make_pair(t, 0));

  // Init distances to nodes seen
  auto seen = kspwlo_impl::Seen<Vertex, Length>{};
  auto seen_b = kspwlo_impl::Seen<Vertex, Length>{};
  seen.insert({s, 0});
  seen_b.insert({t, 0});

  using kspwlo_impl::BiDijkStepRes;
  using kspwlo_impl::Direction;

  Length final_distance = inf;
  auto final_path = std::deque<Vertex>{};
  auto direction = Direction::backward;
  while (!fringe.empty() && !fringe_b.empty()) {
    direction = kspwlo_impl::switch_direction(direction);
    auto result = BiDijkStepRes::next;

    // Run a step
    if (direction == Direction::forward) {
      result = kspwlo_impl::bi_dijkstra_step(
          G, s, t, predecessor, distance, weight, fringe, seen, predecessor_b,
          distance_b, fringe_b, seen_b, direction, final_distance, final_path,
          visitor);
    } else {
      result = kspwlo_impl::bi_dijkstra_step(
          G_b, s, t, predecessor_b, distance_b, weight_b, fringe_b, seen_b,
          predecessor, distance, fringe, seen, direction, final_distance,
          final_path, visitor);
    }

    // Handle the outcome of this step
    switch (result) {
    case BiDijkStepRes::next:
      // Go next
      break;
    case BiDijkStepRes::end:
      // Fill predecessor map
      fill_predecessor(predecessor, final_path);
      return;
    case BiDijkStepRes::negative_weights:
      throw std::domain_error{"Contradictory paths found: negative weights?"};
    default:
      // Unexpected result
      throw std::invalid_argument{"Unexpected result. Exiting..."};
    }
  }

  if (fringe.empty() || fringe_b.empty()) {
    // Clean exit
    fill_predecessor(predecessor, final_path);
    return;
  } else {
    // Otherwise
    throw kspwlo_impl::target_not_found{"No path found!"};
  }
}

template <
    typename Graph, typename PredecessorMap, typename DistanceMap,
    typename WeightMap, typename BackGraph, typename BackWeightMap,
    typename BackIndexMap, typename BiDijkstraVisitorImpl,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
void bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
                            PredecessorMap predecessor, DistanceMap distance,
                            WeightMap weight, const BackGraph &G_b,
                            BackWeightMap weight_b, BackIndexMap index_map_b,
                            BiDijkstraVisitor<BiDijkstraVisitorImpl> &visitor) {
  using namespace boost;
  using Length = typename property_traits<DistanceMap>::value_type;

  auto predecessor_b_vec =
      std::vector<Vertex>(vertices(G_b).first, vertices(G_b).second);
  auto predecessor_b =
      make_iterator_property_map(std::begin(predecessor_b_vec), index_map_b);

  auto distance_b_vec = std::vector<Length>(num_vertices(G_b));
  auto distance_b =
      make_iterator_property_map(std::begin(distance_b_vec), index_map_b);

  bidirectional_dijkstra(G, s, t, predecessor, distance, weight, G_b,
                         predecessor_b, distance_b, weight_b, visitor);
}

template <
    typename Graph, typename PredecessorMap, typename DistanceMap,
    typename WeightMap, typename BackGraph, typename BackWeightMap,
    typename BackIndexMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
void bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
                            PredecessorMap predecessor, DistanceMap distance,
                            WeightMap weight, const BackGraph &G_b,
                            BackWeightMap weight_b, BackIndexMap index_map_b) {
  auto visitor = IdentityBiDijkstraVisitor{};
  bidirectional_dijkstra(G, s, t, predecessor, distance, weight, G_b, weight_b,
                         index_map_b, visitor);
}
} // namespace boost

#endif