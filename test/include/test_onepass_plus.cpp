#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "graph_types.hpp"
#include "graph_utils.hpp"
#include "onepass_plus.hpp"
#include "impl/onepass_plus_impl.hpp"
#include "utils.hpp"

#include "algorithms/kspwlo.hpp"

#include <algorithm>
#include <experimental/filesystem>
#include <fstream>
#include <memory>
#include <string_view>

TEST_CASE("OnePassLabel builds a right path back to source", "[onepasslabel]") {
  using Label = kspwlo_impl::OnePassLabel<kspwlo::Graph>;
  auto s = std::make_shared<Label>(0, 0, 0, 0, 0);
  auto n1 = std::make_shared<Label>(1, 1, 1, s, 1, 1);
  auto n2 = std::make_shared<Label>(2, 2, 2, n1, 2, 1);
  auto n3 = std::make_shared<Label>(3, 3, 2, n2, 3, 1);

  auto path = n3->getPath();
  REQUIRE(boost::num_vertices(path) == 4);

  REQUIRE(edge(0, 1, path).second);
  REQUIRE(edge(1, 2, path).second);
  REQUIRE(edge(2, 3, path).second);
}

TEST_CASE("Computing distance from target", "[distance_from_target]") {
  using namespace boost;
  auto G = read_graph_from_string<kspwlo::Graph>(std::string(graph_gr));

  kspwlo::Vertex target = 6;
  auto distance = kspwlo_impl::distance_from_target(G, target);

  auto index = get(vertex_index, G);
  REQUIRE(distance[index[1]] == 6);
  REQUIRE(distance[index[2]] == 8);
  REQUIRE(distance[index[3]] == 5);
  REQUIRE(distance[index[4]] == 3);
  REQUIRE(distance[index[5]] == 2);
  REQUIRE(distance[index[6]] == 0);
}

TEST_CASE("Computing path from dijkstra_shortest_paths") {
  using namespace boost;

  auto G = read_graph_from_string<kspwlo::Graph>(std::string(graph_gr));
  auto predecessor = std::vector<kspwlo::Vertex>(num_vertices(G), 0);
  auto vertex_id = get(vertex_index, G);
  dijkstra_shortest_paths(
      G, vertex_id[0],
      predecessor_map(make_iterator_property_map(std::begin(predecessor),
                                                 vertex_id, vertex_id[0])));

  auto path = kspwlo_impl::build_path_from_dijkstra(G, predecessor, 0, 6).graph;

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
  auto G = boost::read_graph_from_string<kspwlo::Graph>(std::string{graph_gr});
  kspwlo::Vertex s = 0, t = 6;
  auto res = boost::onepass_plus(G, s, t, 3, 0.5);

  // Create a new tmp file out of graph_gr
  namespace fs = std::experimental::filesystem;
  auto path = fs::temp_directory_path() / std::string("graph_gr_file.gr");
  auto of = std::ofstream(path.string());
  of << graph_gr;
  of.close();

  auto G_regr = std::make_unique<RoadNetwork>(path.c_str());
  auto res_regression = onepass_plus(G_regr.get(), 0, 6, 3, 0.5);

  std::cout << "boost::graph result:\n";
  for (auto &resPath : res) {
    auto &p = resPath.graph;
    for (auto it = edges(p).first; it != edges(p).second; ++it) {
      std::cout << "(" << source(*it, p) << ", " << target(*it, p) << ") ";
    } std::cout << "\n";
  }

  std::cout << "regression result:\n";
  for (auto &regPath : res_regression) {
    auto edges = regPath.getEdges();

    for (auto edge : edges) {
      std::cout << "(" << edge.first << ", " << edge.second << ") ";
    } std::cout << "\n";
  }

  REQUIRE(res.size() == res_regression.size());
}