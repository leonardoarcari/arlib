#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>

#include "test_types.hpp"
#include "utils.hpp"
#include <arlib/details/arlib_utils.hpp>
#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>
#include <arlib/routing_kernels/bidirectional_dijkstra.hpp>

#include <experimental/filesystem>
#include <memory>
#include <string>

#include <string_view>

using namespace arlib::test;

TEST_CASE("Bidirectional Dijkstra finds the same shortest path of "
          "unidirectional dijkstra",
          "[bi_dijkstra") {
  using namespace boost;
  using arlib::VPair;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;

  // Bidirectional Dijkstra
  auto weight_bi = get(edge_weight, G);
  auto distance_bi_vec = std::vector<Length>(num_vertices(G), 6);
  auto predecessor_bi_vec =
      std::vector<Vertex>(vertices(G).first, vertices(G).second);
  auto vertex_id = get(vertex_index, G);
  auto distance_bi =
      make_iterator_property_map(std::begin(distance_bi_vec), vertex_id);
  auto predecessor_bi =
      make_iterator_property_map(std::begin(predecessor_bi_vec), vertex_id);

  auto rev = make_reverse_graph(G);
  auto weight_b = get(edge_weight, rev);
  auto vertex_id_b = get(vertex_index, G);

  // Run Bidirectional dijkstra
  arlib::bidirectional_dijkstra(G, s, t, predecessor_bi, distance_bi, weight_bi,
                                rev, weight_b, vertex_id_b);

  auto sp_bi =
      arlib::details::build_edge_list_from_dijkstra(G, s, t, predecessor_bi);

  // Unidirectional Dijkstra
  auto predecessor_uni =
      std::vector<Vertex>(vertices(G).first, vertices(G).second);
  dijkstra_shortest_paths(G, s,
                          predecessor_map(make_iterator_property_map(
                              std::begin(predecessor_uni), vertex_id, s)));
  auto sp_uni =
      arlib::details::build_edge_list_from_dijkstra(G, s, t, predecessor_uni);

  std::cout << "***** BiDirectional vs Unidirectional solutions*****\n";
  for (auto e : sp_bi) {
    auto u = source(e, G);
    auto v = target(e, G);
    std::cout << "(" << u << ", " << v << ") ";
  }
  std::cout << "\n";

  for (auto e : sp_uni) {
    auto u = source(e, G);
    auto v = target(e, G);
    std::cout << "(" << u << ", " << v << ") ";
  }
  std::cout << "\n";

  REQUIRE(sp_bi == sp_uni);
}