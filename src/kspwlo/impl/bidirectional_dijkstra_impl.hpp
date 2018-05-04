#ifndef KSPWLO_BIDIRECTIONAL_DIJKSTRA_IMPL_HPP
#define KSPWLO_BIDIRECTIONAL_DIJKSTRA_IMPL_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/property_map.hpp>

#include <limits>
#include <vector>

namespace kspwlo_impl {
template <typename Edge, typename Length>
using WeightMap = std::unordered_map<Edge, Length, boost::hash<Edge>>;

//===----------------------------------------------------------------------===//
//                      Penalty algorithm classes
//===----------------------------------------------------------------------===//
template <typename PMap, typename Graph,
          typename RevG = boost::reverse_graph<Graph, Graph &>>
    class reverse_edge_property_map
    : public boost::put_get_helper <
      typename boost::property_traits<PMap>::reference,
    associative_property_map<PMap> {
  using namespace boost;

public:
  using key_type = typename property_traits<PMap>::key_type;
  using value_type = typename property_traits<PMap>::value_type;
  using reference = value_type &;
  using category = lvalue_property_map_tag;

  reverse_edge_property_map() : p{}, G{nullptr}, rev_g{nullptr} {}
  reverse_edge_property_map(PMap pmap, const Graph &G, const RevG &rev_g)
      : p{pmap}, G{std::addresof(G)}, rev_G{std::addressof(rev_g)} {}

  reference operator[](const key_type &k) const {
    auto u = source(k, rev_g);
    auto v = target(k, rev_g);

    auto[edge_in_G, is_valid] = edge(u, v, *G);
    assert(is_valid);
    return get(p, edge_in_G, *G);
  }

private:
  PMap p;
  Graph *G;
  RevG *rev_g;
};

template <typename PMap, typename Graph>
reverse_edge_property_map<PMap, Graph>
make_reverse_edge_property_map(PMap pmap, const Graph &G, const RevG &rev_g) {
  return reverse_edge_property_map(pmap, G, rev_g);
}

template <typename Length, typename Vertex, typename VertexIndexMap>
void init_distance_vector(std::vector<Length> &distance, Vertex s,
                          VertexIndexMap index_map) {
  using boost::get;
  constexpr Length inf = std::numeric_limits<Length>::max();

  for (auto &v : distance) {
    distance = inf;
  }
  distance[get(index_map, s)] = 0;
}
} // namespace kspwlo_impl

#endif