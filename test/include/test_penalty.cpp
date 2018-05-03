#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/graph_types.hpp"
#include "kspwlo/graph_utils.hpp"
#include "kspwlo/impl/kspwlo_impl.hpp"
#include "kspwlo/impl/penalty_impl.hpp"
#include "kspwlo/penalty.hpp"
#include "utils.hpp"

#include <experimental/filesystem>
#include <memory>
#include <string>

#include <string_view>

TEST_CASE("Penalty algorithm follows specifications", "[penalty]") {
  using namespace boost;
  using kspwlo::Vertex;

  auto G = read_graph_from_string<kspwlo::Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  int k = 3;
  double theta = 0.5;
  auto p = 0.1;
  auto r = 0.1;
  auto bound_limit = 10;
  auto max_nb_steps = 100000;

  auto res_paths = penalty_ag(G, s, t, k, theta, p, r, bound_limit, max_nb_steps);

  REQUIRE(res_paths.size() == 3);

  std::cout << "Penalty boost::Graph result:\n";
  for (auto &resPath : res_paths) {
    auto &p = resPath.graph;
    for (auto it = edges(p).first; it != edges(p).second; ++it) {
      std::cout << "(" << source(*it, p) << ", " << target(*it, p) << ") ";
    }
    std::cout << "\n";
  }
}

TEST_CASE("Graph penalization follows specifications", "[penalization]") {
  using namespace boost;
  using kspwlo::Vertex;
  using Edge = typename graph_traits<kspwlo::Graph>::edge_descriptor;

  auto G = read_graph_from_string<kspwlo::Graph>(std::string(graph_gr_esx));

  // Candidate solution
  auto candidate = std::vector<kspwlo::Edge>{{0, 3}, {3, 5}, {5, 6}};

  // Distance maps
  auto distance_s = std::vector<kspwlo::Length>{0, 5, 4, 3, 7, 6, 8};
  auto distance_t = std::vector<kspwlo::Length>{8, 6, 7, 5, 2, 2, 0};

  // Weight map
  const auto original_weight = get(edge_weight, G);
  auto weight = std::unordered_map<Edge, kspwlo::Length, boost::hash<Edge>>{};
  for (auto[it, end] = edges(G); it != end; ++it) {
    weight[*it] = original_weight[*it];
  }

  // Penalty bounds
  auto penalty_bounds = std::unordered_map<Edge, int, boost::hash<Edge>>{};
  Vertex s = 0, t = 6;

  SECTION("Penalizing the first candidate updates correct edges weights") {
    auto p = 0.1;
    auto r = 0.1;
    auto bound_limit = 1;

    auto old_weight =
        std::unordered_map<Edge, kspwlo::Length, boost::hash<Edge>>{weight};

    kspwlo_impl::penalize_candidate_path(candidate, G, s, t, p, r, weight,
                                         distance_s, distance_t, penalty_bounds,
                                         bound_limit);

    // Keep track of candidate edges to exclude them from incoming/outgoing
    // edges update
    auto candidate_edges = std::unordered_set<Edge, boost::hash<Edge>>{};
    auto candidate_vertices = std::unordered_set<Vertex, boost::hash<Vertex>>{};

    // Candidate path weights have been updated
    for (auto[u_c, v_c] : candidate) {
      auto[e, is_valid] = edge(u_c, v_c, G);
      REQUIRE(is_valid);

      candidate_edges.insert(e);

      auto u = source(e, G);
      auto v = target(e, G);
      candidate_vertices.insert(u);
      candidate_vertices.insert(v);

      // Weight penalization is correct
      REQUIRE(weight[e] == old_weight[e] + old_weight[e] * p);
      
      // A nb of updates counter have been created
      auto search = penalty_bounds.find(e);
      REQUIRE(search != std::end(penalty_bounds));
      REQUIRE(search->second == 1);
    }

    // Incoming edges weights have been updated
    for (auto e : candidate_edges) {
      auto u = source(e, G);
      auto v = target(e, G);

      for (auto[it, end] = in_edges(u, G); it != end; ++it) {
        auto a = source(*it, G);

        if (candidate_vertices.find(a) == std::end(candidate_vertices)) {
          auto closeness = distance_t[u] / distance_t[s];
          REQUIRE(weight[*it] ==
                  old_weight[*it] + old_weight[*it] * (0.1 + r * closeness));
        }
      }

      for (auto[it, end] = out_edges(v, G); it != end; ++it) {
        auto b = target(*it, G);

        if (candidate_vertices.find(b) == std::end(candidate_vertices)) {
          auto closeness = distance_s[v] / distance_s[t];
          REQUIRE(weight[*it] ==
                  old_weight[*it] + old_weight[*it] * (0.1 + r * closeness));
        }
      }
    }
  }

  SECTION("Penalizing edges that reached the limit of number of updates leave "
          "their weights unchanged") {
    auto p = 0.1;
    auto r = 0.1;
    auto bound_limit = 1;

    auto[e1, is_e1_valid] = edge(3, 5, G);
    auto[e2, is_e2_valid] = edge(1, 6, G);

    REQUIRE(is_e1_valid);
    REQUIRE(is_e2_valid);

    // Set e1 and e2 updates limit
    penalty_bounds.insert({e1, bound_limit});
    penalty_bounds.insert({e2, bound_limit});

    auto old_weight =
        std::unordered_map<Edge, kspwlo::Length, boost::hash<Edge>>{weight};

    kspwlo_impl::penalize_candidate_path(candidate, G, s, t, p, r, weight,
                                         distance_s, distance_t, penalty_bounds,
                                         bound_limit);

    // Weights must be left unchanged.
    REQUIRE(weight[e1] == old_weight[e1]);
    REQUIRE(weight[e2] == old_weight[e2]);
  }
}

TEST_CASE("Two-ways dijkstra computes right distance maps") {
  using namespace boost;
  using kspwlo::Vertex;

  auto G = read_graph_from_string<kspwlo::Graph>(std::string(graph_gr_esx));
  Vertex s = 0, t = 6;

  // Candidate solution
  auto candidate = std::vector<kspwlo::Edge>{{0, 3}, {3, 5}, {5, 6}};

  // Distance maps
  auto distance_s = std::vector<kspwlo::Length>{0, 5, 4, 3, 7, 6, 8};
  auto distance_t = std::vector<kspwlo::Length>{8, 6, 7, 5, 2, 2, 0};

  auto test_distance_s = std::vector<kspwlo::Length>(num_vertices(G));
  auto test_distance_t = std::vector<kspwlo::Length>(num_vertices(G));

  kspwlo_impl::dijkstra_shortest_path_two_ways(G, s, t, test_distance_s, test_distance_t);

  REQUIRE(distance_s == test_distance_s);
  REQUIRE(distance_t == test_distance_t);
}