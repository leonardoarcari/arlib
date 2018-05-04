#ifndef BOOST_BIDIRECTIONAL_DIJKSTRA_HPP
#define BOOST_BIDIRECTIONAL_DIJKSTRA_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>

#include "kspwlo/impl/bidirectional_dijkstra_impl.hpp"

#include <limits>
#include <vector>

/**
 * @brief Algorithms and utilities for Boost::Graph
 */
namespace boost {
template <
    typename Graph, typename PredecessorMap, typename DistanceMap,
    typename WeightMap, typename VertexIndexMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
void bidirectional_dijkstra(const Graph &G, Vertex s, Vertex t,
                            PredecessorMap predecessor, DistanceMap distance,
                            WeightMap weight, VertexIndexMap index_map) {
  using namespace boost;
  using Length = typename property_traits<DistanceMap>::value_type;
  constexpr Length inf = std::numeric_limits<Length>::max();

  // Initialize data structures
  kspwlo_impl::init_distance_vector(distance, s, index_map);

  // Backwards data structures
  auto rev_G = make_reverse_graph(G);
  auto index_map_b = get(vertex_index, rev_G);
  auto distance_b_vec = std::vector<Length>(num_vertices(G));
  kspwlo_impl::init_distance_vector(distance_b_vec, t, index_map_b);
  auto distance_b = make_iterator_property_map(std::begin(distance_b_vec), index_map_b);
  auto weight_b = kspwlo_impl::make_reverse_edge_property_map(weight, G, rev_G);
}
} // namespace boost

#endif