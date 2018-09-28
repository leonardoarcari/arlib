#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "utils.hpp"
#include <arlib/details/arlib_utils.hpp>
#include <arlib/details/esx_impl.hpp>
#include <arlib/esx.hpp>
#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>

#include <kspwlo_ref/algorithms/kspwlo.hpp>
#include <kspwlo_ref/exploration/graph_utils.hpp>

#include <experimental/filesystem>
#include <memory>
#include <string>

#include <string_view>

TEST_CASE("Edge priority computation", "[esx]") {
  using namespace boost;
  using arlib::Vertex;
  using arlib::details::compute_priority;

  auto G =
      arlib::read_graph_from_string<arlib::Graph>(std::string(graph_gr_esx));
  Vertex s = 0, t = 6;

  // Compute shortest path from s to t
  auto sp_path = arlib::details::compute_shortest_path(G, s, t);
  auto &sp = sp_path.graph();

  // Compute lower bounds for AStar
  auto heuristic =
      arlib::details::distance_heuristic<arlib::Graph, arlib::Length>(G, t);

  // We keep a set of deleted-edges
  using Edge = typename graph_traits<arlib::Graph>::edge_descriptor;
  auto deleted_edges = std::unordered_set<Edge, boost::hash<Edge>>{};

  // Check that (0, 3) was computed in shortest path
  REQUIRE(edge(0, 3, sp).second);
  auto s_n3 = edge(0, 3, G).first;

  int prio_s_n3 = compute_priority(G, s_n3, deleted_edges);
  REQUIRE(prio_s_n3 == 0);

  // Check that (3, 5) was computed in shortest path
  REQUIRE(edge(3, 5, sp).second);
  auto n3_n5 = edge(3, 5, G).first;

  int prio_n3_n5 = compute_priority(G, n3_n5, deleted_edges);
  REQUIRE(prio_n3_n5 == 3);

  // Check that (5, 6) was computed in shortest path
  REQUIRE(edge(5, 6, sp).second);
  auto n5_t = edge(5, 6, G).first;

  int prio_n5_t = compute_priority(G, n5_t, deleted_edges);
  REQUIRE(prio_n5_t == 0);
}

TEST_CASE("esx kspwlo algorithm runs on Boost::Graph", "[esx]") {
  auto G =
      arlib::read_graph_from_string<arlib::Graph>(std::string{graph_gr_esx});
  arlib::Vertex s = 0, t = 6;
  auto res = arlib::esx(G, s, t, 3, 0.5);

  // Create a new tmp file out of graph_gr_esx
  namespace fs = std::experimental::filesystem;
  auto path = fs::temp_directory_path() / std::string("graph_gr_esx_file.gr");
  auto of = std::ofstream(path.string());
  of << graph_gr_esx;
  of.close();

  auto G_regr = std::make_unique<RoadNetwork>(path.c_str());
  auto res_regression = esx(G_regr.get(), 0, 6, 3, 0.5);

  using boost::edges;
  using boost::source;
  using boost::target;
  std::cout << "Esx boost::graph result:\n";
  for (auto &resPath : res) {
    auto &p = resPath.graph();
    for (auto it = edges(p).first; it != edges(p).second; ++it) {
      std::cout << "(" << source(*it, p) << ", " << target(*it, p) << ") ";
    }
    std::cout << "\n";
  }

  std::cout << "Esx regression result:\n";
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
  for (auto &resPath : res) {
    auto &p = resPath.graph();
    REQUIRE(one_regression_path_have_edges(res_regression, p));
  }
}

TEST_CASE("ESX running with bidirectional dijkstra returns same result as "
          "unidirectional dijkstra",
          "[esx]") {
  using namespace boost;
  using arlib::Vertex;

  auto G =
      arlib::read_graph_from_string<arlib::Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  int k = 3;
  double theta = 0.5;

  auto res_paths_uni = arlib::esx(G, s, t, 3, 0.5);
  auto res_paths_bi = arlib::esx(
      G, s, t, 3, 0.5, arlib::shortest_path_algorithm::bidirectional_dijkstra);

  REQUIRE(res_paths_uni.size() == res_paths_bi.size());

  for (std::size_t i = 0; i < res_paths_uni.size(); ++i) {
    REQUIRE(res_paths_uni[i].length() == res_paths_bi[i].length());
  }
}