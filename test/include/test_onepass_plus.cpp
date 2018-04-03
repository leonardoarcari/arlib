#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "graph_types.hpp"
#include "graph_utils.hpp"
#include "onepass_plus.hpp"
#include "utils.hpp"

#include <algorithm>
#include <experimental/filesystem>
#include <fstream>
#include <memory>
#include <string_view>

TEST_CASE("OnePassLabel builds a right path back to source", "[onepasslabel]") {
  using kspwlo::OnePassLabel;
  auto s = std::make_shared<OnePassLabel<int, int>>(0, 0, 0, 0);
  auto n1 = std::make_shared<OnePassLabel<int, int>>(1, 1, 1, s, 1);
  auto n2 = std::make_shared<OnePassLabel<int, int>>(2, 2, 2, n1, 1);
  auto n3 = std::make_shared<OnePassLabel<int, int>>(3, 3, 2, n2, 1);

  auto path = n3->getPath<kspwlo::Graph>();
  REQUIRE(boost::num_vertices(path) == 4);
}

TEST_CASE("Computing distance from target", "[distance_from_target]") {
  using namespace boost;
  auto G = read_graph_from_string<kspwlo::Graph>(std::string(graph_gr));

  kspwlo::Vertex target = 6;
  auto weight = get(edge_weight, G);
  using size_type = typename property_traits<decltype(weight)>::value_type;
  auto distance = kspwlo::distance_from_target<size_type>(G, target);

  auto index = get(vertex_index, G);
  REQUIRE(distance[index[1]] == 6);
  REQUIRE(distance[index[2]] == 7);
  REQUIRE(distance[index[3]] == 5);
  REQUIRE(distance[index[4]] == 2);
  REQUIRE(distance[index[5]] == 2);
  REQUIRE(distance[index[6]] == 0);
}

TEST_CASE("Computing path from dijkstra_shortest_paths") {
  using namespace boost;

  auto G = read_graph_from_string<kspwlo::Graph>(std::string(graph_gr));
  auto predecessor = std::vector<kspwlo::Vertex>(num_vertices(G), 0);
  auto vertex_id = get(vertex_index, G);
  dijkstra_shortest_paths(G, vertex_id[0],
                          predecessor_map(make_iterator_property_map(
                              std::begin(predecessor), vertex_id, vertex_id[0])));
  
  auto path = build_graph_from_dijkstra(G, predecessor, 0, 6);

  REQUIRE(num_edges(path) == 3);

  auto weight = get(edge_weight, path);
  REQUIRE(edge(0, 3, path).second);
  REQUIRE(weight[edge(0, 3, path).first] == 3);
  REQUIRE(edge(3, 5, path).second);
  REQUIRE(weight[edge(3, 5, path).first] == 3);
  REQUIRE(edge(5, 6, path).second);
  REQUIRE(weight[edge(5, 6, path).first] == 2);
}

TEST_CASE("onepass_plus kspwlo algorithm runs on Boost::Graph",
          "[boost::graph]") {
  using namespace boost;
  auto G = read_graph_from_string<kspwlo::Graph>(std::string{graph_gr});

  auto res = onepass_plus(G, 0, 6, 3, 0.5);
  REQUIRE(res.empty());
}