#include "catch.hpp"

#include <boost/graph/betweenness_centrality.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/details/arlib_utils.hpp>
#include <arlib/details/esx_impl.hpp>
#include <arlib/esx.hpp>
#include <arlib/graph_utils.hpp>
#include <arlib/routing_kernels/types.hpp>
#include <arlib/terminators.hpp>

#include "cittastudi_graph.hpp"
#include "test_types.hpp"
#include "utils.hpp"

#include <kspwlo_ref/algorithms/kspwlo.hpp>
#include <kspwlo_ref/exploration/graph_utils.hpp>

#include <chrono>
#include <experimental/filesystem>
#include <memory>
#include <string>
#include <string_view>

using namespace arlib::test;

template <typename ForwardIt, typename Edge>
bool contains(ForwardIt first, ForwardIt last, Edge e) {
  if (auto search = std::find(first, last, e); search != last) {
    return true;
  } else {
    return false;
  }
}

TEST_CASE("Edge priority computation", "[esx]") {
  using namespace boost;
  using arlib::details::compute_priority;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));
  Vertex s = 0, t = 6;

  // Compute shortest path from s to t
  auto weight_map = get(edge_weight, G);
  auto sp_path = *arlib::details::compute_shortest_path(G, weight_map, s, t);

  // Compute lower bounds for AStar
  auto heuristic = arlib::details::distance_heuristic<Graph, Length>(G, t);

  // We keep a set of deleted-edges
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  auto deleted_edges = std::unordered_set<Edge, boost::hash<Edge>>{};

  // Check that (0, 3) was computed in shortest path
  REQUIRE(contains(sp_path.begin(), sp_path.end(), edge(0, 3, G).first));
  auto s_n3 = edge(0, 3, G).first;

  int prio_s_n3 = compute_priority(G, s_n3, weight_map, deleted_edges);
  REQUIRE(prio_s_n3 == 0);

  // Check that (3, 5) was computed in shortest path
  REQUIRE(contains(sp_path.begin(), sp_path.end(), edge(3, 5, G).first));
  auto n3_n5 = edge(3, 5, G).first;

  int prio_n3_n5 = compute_priority(G, n3_n5, weight_map, deleted_edges);
  REQUIRE(prio_n3_n5 == 3);

  // Check that (5, 6) was computed in shortest path
  REQUIRE(contains(sp_path.begin(), sp_path.end(), edge(5, 6, G).first));
  auto n5_t = edge(5, 6, G).first;

  int prio_n5_t = compute_priority(G, n5_t, weight_map, deleted_edges);
  REQUIRE(prio_n5_t == 0);
}

TEST_CASE("esx kspwlo algorithm runs on Boost::Graph", "[esx]") {
  auto G = arlib::read_graph_from_string<Graph>(std::string{graph_gr_esx});
  Vertex s = 0, t = 6;
  auto predecessors = arlib::multi_predecessor_map<Vertex>{};
  arlib::esx(G, predecessors, s, t, 3, 0.5);
  auto res = arlib::to_paths(G, predecessors, s, t);

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
  for (auto &p : res) {
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
  for (auto &p : res) {
    REQUIRE(one_regression_path_have_edges(res_regression, p));
  }

  using boost::edge_weight;
  using boost::get;
  REQUIRE(alternative_paths_are_dissimilar(res, get(edge_weight, G), 0.5));
}

TEST_CASE("ESX running with bidirectional dijkstra returns same result as "
          "unidirectional dijkstra",
          "[esx]") {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  int k = 3;
  double theta = 0.5;

  auto predecessors_uni = arlib::multi_predecessor_map<Vertex>{};
  arlib::esx(G, predecessors_uni, s, t, 3, 0.5);
  auto res_paths_uni = arlib::to_paths(G, predecessors_uni, s, t);

  auto predecessors_bi = arlib::multi_predecessor_map<Vertex>{};
  arlib::esx(G, predecessors_bi, s, t, 3, 0.5,
             arlib::routing_kernels::bidirectional_dijkstra);
  auto res_paths_bi = arlib::to_paths(G, predecessors_bi, s, t);

  REQUIRE(res_paths_uni.size() == res_paths_bi.size());

  for (std::size_t i = 0; i < res_paths_uni.size(); ++i) {
    REQUIRE(res_paths_uni[i].length() == res_paths_bi[i].length());
  }
}

TEST_CASE("ESX running with plain dijkstra returns same result as astar",
          "[esx]") {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  int k = 3;
  double theta = 0.5;

  auto predecessors_uni = arlib::multi_predecessor_map<Vertex>{};
  arlib::esx(G, predecessors_uni, s, t, 3, 0.5);
  auto res_paths_uni = arlib::to_paths(G, predecessors_uni, s, t);

  auto predecessors_bi = arlib::multi_predecessor_map<Vertex>{};
  arlib::esx(G, predecessors_bi, s, t, 3, 0.5,
             arlib::routing_kernels::dijkstra);
  auto res_paths_bi = arlib::to_paths(G, predecessors_bi, s, t);

  REQUIRE(res_paths_uni.size() == res_paths_bi.size());

  for (std::size_t i = 0; i < res_paths_uni.size(); ++i) {
    REQUIRE(res_paths_uni[i].length() == res_paths_bi[i].length());
  }
}

TEST_CASE("ESX times-out on large graph", "[esx]") {
  using namespace boost;
  using namespace std::chrono_literals;

  auto G = arlib::read_graph_from_string<Graph>(std::string(cittastudi_gr));

  Vertex s = 0, t = 20;
  auto k = 3;
  auto theta = 0.5;
  auto predecessors = arlib::multi_predecessor_map<Vertex>{};

  REQUIRE_THROWS_AS(arlib::esx(G, predecessors, s, t, k, theta,
                               arlib::routing_kernels::astar,
                               arlib::timer{1us}),
                    arlib::terminator_stop_error);
}

TEST_CASE(
    "ESX with Betweeness-Centrality is functionally equivalent to original one",
    "[esx]") {
  using namespace boost;

  auto G = arlib::read_csr_graph_from_string(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  int k = 3;
  double theta = 0.5;

  using EdgeCentralityProperty =
      exterior_edge_property<arlib::CSRGraph, double>;
  using EdgeCentralityContainer =
      typename EdgeCentralityProperty::container_type;
  using EdgeCentralityMap = typename EdgeCentralityProperty::map_type;

  // Compute Edge betweeness-centrality
  auto edge2centrality = EdgeCentralityContainer(num_edges(G));
  auto edge_centrality_map = EdgeCentralityMap(edge2centrality, G);
  boost::brandes_betweenness_centrality(
      G, boost::edge_centrality_map(edge_centrality_map));

  // ARP with Betweeness-Centrality
  auto predecessors_bc = arlib::multi_predecessor_map<Vertex>{};
  arlib::esx(G, predecessors_bc, edge_centrality_map, s, t, k, theta);
  auto res_paths_bc = arlib::to_paths(G, predecessors_bc, s, t);

  // ARP for vanilla ESX
  auto predecessors_bi = arlib::multi_predecessor_map<Vertex>{};
  arlib::esx(G, predecessors_bi, s, t, k, theta,
             arlib::routing_kernels::dijkstra);
  auto res_paths_bi = arlib::to_paths(G, predecessors_bi, s, t);

  // Require solutions to match
  REQUIRE(res_paths_bc.size() == res_paths_bi.size());

  for (std::size_t i = 0; i < res_paths_bc.size(); ++i) {
    REQUIRE(res_paths_bc[i].length() == res_paths_bi[i].length());
  }
}
