#ifndef BOOST_BIDIRECTIONAL_DIJKSTRA_HPP
#define BOOST_BIDIRECTIONAL_DIJKSTRA_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>

#include "kspwlo/impl/bidirectional_dijkstra_impl.hpp"
#include "kspwlo/impl/kspwlo_impl.hpp"

#include <limits>
#include <vector>

/**
 * @brief Algorithms and utilities for Boost::Graph
 */
namespace boost {
template <
    typename Graph, typename PredecessorMap, typename DistanceMap,
    typename WeightMap, typename VertexIndexMap, typename BackGraph,
    typename BackWeightMap, typename BackIndexMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
void bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
                            PredecessorMap predecessor, DistanceMap distance,
                            WeightMap weight, VertexIndexMap index_map,
                            const BackGraph &G_b, BackWeightMap weight_b,
                            BackIndexMap index_map_b) {
  using namespace boost;
  using Length = typename property_traits<DistanceMap>::value_type;
  constexpr Length inf = std::numeric_limits<Length>::max();

  // Initialize distance structures
  kspwlo_impl::init_distance_vector(G, distance, s);

  auto distance_b_vec = std::vector<Length>(num_vertices(G_b));
  auto distance_b =
      make_iterator_property_map(std::begin(distance_b_vec), index_map_b);
  kspwlo_impl::init_distance_vector(G_b, distance_b, t);

  auto paths = kspwlo_impl::PathMap<Vertex>{};
  auto paths_b = kspwlo_impl::PathMap<Vertex>{};
  paths.insert({s, std::vector<Vertex>{s}});
  paths_b.insert({t, std::vector<Vertex>{t}});

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
  auto final_path = std::vector<Vertex>{};
  auto direction = Direction::backward;
  while (!fringe.empty() && !fringe_b.empty()) {
    direction = kspwlo_impl::switch_direction(direction);
    auto result = BiDijkStepRes::next;

    // Run a step
    if (direction == Direction::forward) {
      result = kspwlo_impl::bi_dijkstra_step(
          G, t, paths, distance, weight, index_map, fringe, seen, paths_b,
          distance_b, index_map_b, fringe_b, seen_b, direction, final_distance,
          final_path);
    } else {
      result = kspwlo_impl::bi_dijkstra_step(
          G_b, t, paths_b, distance_b, weight_b, index_map_b, fringe_b, seen_b,
          paths, distance, index_map, fringe, seen, direction, final_distance,
          final_path);
    }

    // Handle the outcome of this step
    if (result == BiDijkStepRes::next) {
      continue;
    } else if (result == BiDijkStepRes::end) {
      // Update predecessor map for computed shortest path
      for (std::size_t i = 0; i < final_path.size() - 1; ++i) {
        predecessor[final_path[i + 1]] = final_path[i];
      }
      return;
    } else if (result == BiDijkStepRes::negative_weights) {
      throw std::domain_error{"Contradictory paths found: negative weights? "};
    } else {
      break;
    }
  }

  // Otherwise
  throw kspwlo_impl::target_not_found{"No path found!"};
}
} // namespace boost

#endif