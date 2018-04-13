#ifndef KSPWLO_IMPL_HPP
#define KSPWLO_IMPL_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>

#include "kspwlo/graph_types.hpp"

#include <vector>

namespace kspwlo_impl {
//===----------------------------------------------------------------------===//
//                      kSPwLO algorithms routines
//===----------------------------------------------------------------------===//

template <
    typename Graph,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename length_type =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
std::vector<length_type> distance_from_target(Graph &G, Vertex t) {
  // Reverse graph
  auto G_rev = boost::make_reverse_graph(G);
  auto distance = std::vector<length_type>(boost::num_vertices(G_rev));

  // Run dijkstra_shortest_paths and return distance vector
  boost::dijkstra_shortest_paths(G_rev, t, boost::distance_map(&distance[0]));
  return distance;
}

template <typename Graph, typename PredecessorMap, typename Vertex,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
kspwlo::Path<Graph> build_path_from_dijkstra(Graph &G, const PredecessorMap &p,
                                             Vertex s, Vertex t) {
  length_type length = 0;
  auto weight = boost::get(boost::edge_weight, G);
  auto edge_list = std::vector<kspwlo::Edge>{};

  auto current = t;
  while (current != s) {
    auto u = p[current];
    edge_list.emplace_back(u, current);

    auto edge_in_G = boost::edge(u, current, G).first;

    length += weight[edge_in_G];
    current = u;
  }

  return {build_graph_from_edges(edge_list, G), length};
}

template <
    typename Graph,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Length =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
kspwlo::Path<Graph> compute_shortest_path(Graph &G, Vertex s, Vertex t) {
  using namespace boost;
  auto sp_distances = std::vector<Length>(num_vertices(G));
  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);
  dijkstra_shortest_paths(G, s,
                          distance_map(&sp_distances[0])
                              .predecessor_map(make_iterator_property_map(
                                  std::begin(predecessor), vertex_id, s)));
  return build_path_from_dijkstra(G, predecessor, s, t);
}
} // namespace kspwlo_impl

#endif