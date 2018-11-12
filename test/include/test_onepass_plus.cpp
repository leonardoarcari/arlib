#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/details/onepass_plus_impl.hpp>
#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>
#include <arlib/onepass_plus.hpp>
#include <arlib/terminators.hpp>

#include "cittastudi_graph.hpp"
#include "test_types.hpp"
#include "utils.hpp"

#include <kspwlo_ref/algorithms/kspwlo.hpp>
#include <kspwlo_ref/exploration/graph_utils.hpp>

#include <algorithm>
#include <chrono>
#include <experimental/filesystem>
#include <fstream>
#include <memory>
#include <string_view>

using namespace arlib::test;

//===----------------------------------------------------------------------===//
//                                Test cases
//===----------------------------------------------------------------------===//

TEST_CASE("OnePassLabel builds a right path back to source", "[onepassplus]") {
  using arlib::VPair;
  using namespace boost;
  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr));
  using Label = arlib::details::OnePassLabel<Graph, Length>;

  auto s = std::make_unique<Label>(0, 0, 0, 0, 0);
  auto n1 = std::make_unique<Label>(3, 1, 1, s.get(), 1, 1);
  auto n2 = std::make_unique<Label>(5, 2, 2, n1.get(), 2, 1);
  auto n3 = std::make_unique<Label>(6, 3, 2, n2.get(), 3, 1);

  auto path = n3->get_path(G);

  REQUIRE(std::find(std::begin(path), std::end(path), edge(0, 3, G).first) !=
          std::end(path));
  REQUIRE(std::find(std::begin(path), std::end(path), edge(3, 5, G).first) !=
          std::end(path));
  REQUIRE(std::find(std::begin(path), std::end(path), edge(5, 6, G).first) !=
          std::end(path));
}

TEST_CASE("Computing distance from target", "[onepassplus]") {
  using namespace boost;
  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr));

  Vertex target = 6;
  auto distance = arlib::details::distance_from_target<Length>(G, target);

  auto index = get(vertex_index, G);
  REQUIRE(distance[index[1]] == 6);
  REQUIRE(distance[index[2]] == 8);
  REQUIRE(distance[index[3]] == 5);
  REQUIRE(distance[index[4]] == 3);
  REQUIRE(distance[index[5]] == 2);
  REQUIRE(distance[index[6]] == 0);
}

TEST_CASE("Computing path from dijkstra_shortest_paths", "[onepassplus]") {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr));
  auto predecessor = std::vector<Vertex>(num_vertices(G), 0);
  auto vertex_id = get(vertex_index, G);
  dijkstra_shortest_paths(
      G, vertex_id[0],
      predecessor_map(make_iterator_property_map(std::begin(predecessor),
                                                 vertex_id, vertex_id[0])));
  auto weight_G = get(edge_weight, G);
  auto path = arlib::details::build_path_from_dijkstra(G, weight_G, predecessor,
                                                       0lu, 6lu);

  auto weight = get(edge_weight, path);
  REQUIRE(edge(0, 3, path).second);
  REQUIRE(weight[edge(0, 3, path).first] == 3);
  REQUIRE(edge(3, 5, path).second);
  REQUIRE(weight[edge(3, 5, path).first] == 3);
  REQUIRE(edge(5, 6, path).second);
  REQUIRE(weight[edge(5, 6, path).first] == 2);
}

TEST_CASE("onepass_plus kspwlo algorithm runs on Boost::Graph",
          "[onepassplus]") {
  auto G = arlib::read_graph_from_string<Graph>(std::string{graph_gr});
  Vertex s = 0, t = 6;
  auto predecessors = arlib::multi_predecessor_map<Vertex>{};
  arlib::onepass_plus(G, predecessors, s, t, 3, 0.5);
  auto res = arlib::to_paths(G, predecessors, s, t);

  // Create a new tmp file out of graph_gr
  namespace fs = std::experimental::filesystem;
  auto path = fs::temp_directory_path() / std::string("graph_gr_file.gr");
  auto of = std::ofstream(path.string());
  of << graph_gr;
  of.close();

  auto G_regr = std::make_unique<RoadNetwork>(path.c_str());
  auto res_regression = onepass_plus(G_regr.get(), 0, 6, 3, 0.5);

  using boost::edges;
  using boost::source;
  using boost::target;
  std::cout << "boost::graph result:\n";
  for (auto &p : res) {
    for (auto it = edges(p).first; it != edges(p).second; ++it) {
      std::cout << "(" << source(*it, p) << ", " << target(*it, p) << ") ";
    }
    std::cout << "\n";
  }

  std::cout << "regression result:\n";
  for (auto &regPath : res_regression) {
    auto es = regPath.getEdges();
    // Cleaning loops coming from dijkstra algorithm (for no reason)
    remove_self_loops(es.begin(), es.end());

    for (auto edge : es) {
      std::cout << "(" << edge.first << ", " << edge.second << ") ";
    }
    std::cout << "\n";
  }

  // Same number of paths are computed
  REQUIRE(res.size() == res_regression.size());

  // For each k-spwlo check if its edges are in a solution of the regression
  // test
  for (auto &p : res) {
    REQUIRE(one_regression_path_have_edges(res_regression, p));
  }

  using boost::edge_weight;
  using boost::get;
  REQUIRE(alternative_paths_are_dissimilar(res, get(edge_weight, G), 0.5));
}

TEST_CASE("OnePass+ times-out on large graph", "[onepassplus]") {
  using namespace boost;
  using namespace std::chrono_literals;

  auto G = arlib::read_graph_from_string<Graph>(std::string(cittastudi_gr));

  Vertex s = 0, t = 20;
  auto k = 3;
  auto theta = 0.5;
  auto predecessors = arlib::multi_predecessor_map<Vertex>{};

  REQUIRE_THROWS_AS(
      arlib::onepass_plus(G, predecessors, s, t, k, theta, arlib::timer{1us}),
      arlib::terminator_stop_error);
}