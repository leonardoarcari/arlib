#ifndef KSPWLO_IMPL_HPP
#define KSPWLO_IMPL_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>

#include "kspwlo/graph_types.hpp"

#include <sstream>
#include <stdexcept>
#include <vector>

namespace kspwlo_impl {
//===----------------------------------------------------------------------===//
//                      kSPwLO algorithms support classes
//===----------------------------------------------------------------------===//

struct target_found {};
struct target_not_found : public std::logic_error {
  explicit target_not_found(const std::string &what_arg)
      : std::logic_error(what_arg) {}
  explicit target_not_found(const char *what_arg)
      : std::logic_error(what_arg) {}
};

template <typename Vertex, typename Tag>
class target_visitor : public boost::base_visitor<target_visitor<Vertex, Tag>> {
public:
  using event_filter = Tag;

  explicit target_visitor(Vertex t) : t{t} {}
  template <typename Graph> void operator()(Vertex u, Graph &) {
    if (u == t) {
      throw target_found{};
    }
  }

private:
  Vertex t;
};
template <typename Vertex, typename Tag>
target_visitor<Vertex, Tag> make_target_visitor(Vertex t, Tag) {
  return target_visitor<Vertex, Tag>{t};
}

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
  try {
    dijkstra_shortest_paths(
        G, s,
        distance_map(&sp_distances[0])
            .predecessor_map(make_iterator_property_map(std::begin(predecessor),
                                                        vertex_id, s))
            .visitor(make_dijkstra_visitor(
                make_target_visitor(t, on_examine_vertex{}))));
  } catch (target_found tf) {
    return build_path_from_dijkstra(G, predecessor, s, t);
  }
  // If target was not found, t is unreachable from s
  auto oss = std::ostringstream{};
  oss << "Vertex " << t << " is unreachable from " << s;
  throw target_not_found{oss.str()};
}
} // namespace kspwlo_impl

#endif