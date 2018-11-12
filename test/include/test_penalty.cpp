#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/details/arlib_utils.hpp>
#include <arlib/details/penalty_impl.hpp>
#include <arlib/graph_utils.hpp>
#include <arlib/penalty.hpp>
#include <arlib/routing_kernels/types.hpp>
#include <arlib/terminators.hpp>

#include "cittastudi_graph.hpp"
#include "test_types.hpp"
#include "utils.hpp"

#include <chrono>
#include <experimental/filesystem>
#include <memory>
#include <string>
#include <string_view>

using namespace arlib::test;

TEST_CASE("Penalty algorithm follows specifications", "[penalty]") {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  int k = 3;
  double theta = 0.5;
  auto p = 0.1;
  auto r = 0.1;
  auto bound_limit = 10;
  auto max_nb_steps = 100000;

  auto predecessors = arlib::multi_predecessor_map<Vertex>{};
  arlib::penalty(G, predecessors, s, t, k, theta, p, r, bound_limit,
                 max_nb_steps);
  auto res_paths = arlib::to_paths(G, predecessors, s, t);

  REQUIRE(
      alternative_paths_are_dissimilar(res_paths, get(edge_weight, G), theta));
  REQUIRE(res_paths.size() == 3);

  std::cout << "Penalty boost::Graph result:\n";
  for (auto &p : res_paths) {
    for (auto it = edges(p).first; it != edges(p).second; ++it) {
      std::cout << "(" << source(*it, p) << ", " << target(*it, p) << ") ";
    }
    std::cout << "\n";
  }
}

TEST_CASE("Graph penalization follows specifications", "[penalty]") {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));

  // Candidate solution
  auto candidate = std::vector<Edge>{edge(0, 3, G).first, edge(3, 5, G).first,
                                     edge(5, 6, G).first};

  // Distance maps
  auto distance_s = std::vector<Length>{0, 5, 4, 3, 7, 6, 8};
  auto distance_t = std::vector<Length>{8, 6, 7, 5, 2, 2, 0};

  // Weight map
  auto original_weight = get(edge_weight, G);
  auto penalty = arlib::details::penalty_functor{original_weight};

  // Penalty bounds
  auto penalty_bounds = std::unordered_map<Edge, int, boost::hash<Edge>>{};
  Vertex s = 0, t = 6;

  SECTION("Penalizing the first candidate updates correct edges weights",
          "[penalty]") {
    auto p = 0.1;
    auto r = 0.1;
    auto bound_limit = 1;

    auto old_penalty = penalty.clone();

    arlib::details::penalize_candidate_path(candidate, G, s, t, p, r, penalty,
                                            distance_s, distance_t,
                                            penalty_bounds, bound_limit);

    // Keep track of candidate edges to exclude them from incoming/outgoing
    // edges update
    auto candidate_edges = std::unordered_set<Edge, boost::hash<Edge>>{};
    auto candidate_vertices = std::unordered_set<Vertex, boost::hash<Vertex>>{};

    // Candidate path weights have been updated
    for (auto e : candidate) {
      candidate_edges.insert(e);

      auto u = source(e, G);
      auto v = target(e, G);
      candidate_vertices.insert(u);
      candidate_vertices.insert(v);

      // Weight penalization is correct
      REQUIRE(penalty[e] == old_penalty[e] + old_penalty[e] * p);

      // A nb of updates counter have been created
      auto search = penalty_bounds.find(e);
      REQUIRE(search != std::end(penalty_bounds));
      REQUIRE(search->second == 1);
    }

    // Incoming edges weights have been updated
    for (auto e : candidate_edges) {
      auto u = source(e, G);
      auto v = target(e, G);

      for (auto [it, end] = in_edges(u, G); it != end; ++it) {
        auto a = source(*it, G);

        if (candidate_vertices.find(a) == std::end(candidate_vertices)) {
          auto closeness = distance_t[u] / distance_t[s];
          REQUIRE(penalty[*it] ==
                  old_penalty[*it] + old_penalty[*it] * (0.1 + r * closeness));
        }
      }

      for (auto [it, end] = out_edges(v, G); it != end; ++it) {
        auto b = target(*it, G);

        if (candidate_vertices.find(b) == std::end(candidate_vertices)) {
          auto closeness = distance_s[v] / distance_s[t];
          REQUIRE(penalty[*it] ==
                  old_penalty[*it] + old_penalty[*it] * (0.1 + r * closeness));
        }
      }
    }
  }

  SECTION("Penalizing edges that reached the limit of number of updates leave "
          "their weights unchanged",
          "[penalty]") {
    auto p = 0.1;
    auto r = 0.1;
    auto bound_limit = 1;

    auto [e1, is_e1_valid] = edge(3, 5, G);
    auto [e2, is_e2_valid] = edge(1, 6, G);

    REQUIRE(is_e1_valid);
    REQUIRE(is_e2_valid);

    // Set e1 and e2 updates limit
    penalty_bounds.insert({e1, bound_limit});
    penalty_bounds.insert({e2, bound_limit});

    auto old_penalty = arlib::details::penalty_functor{penalty};

    arlib::details::penalize_candidate_path(candidate, G, s, t, p, r, penalty,
                                            distance_s, distance_t,
                                            penalty_bounds, bound_limit);

    // Weights must be left unchanged.
    REQUIRE(penalty[e1] == old_penalty[e1]);
    REQUIRE(penalty[e2] == old_penalty[e2]);
  }
}

TEST_CASE("Two-ways dijkstra computes right distance maps", "[penalty]") {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));
  Vertex s = 0, t = 6;

  // Candidate solution
  auto candidate = std::vector<Edge>{edge(0, 3, G).first, edge(3, 5, G).first,
                                     edge(5, 6, G).first};

  // Distance maps
  auto distance_s = std::vector<Length>{0, 5, 4, 3, 7, 6, 8};
  auto distance_t = std::vector<Length>{8, 6, 7, 5, 2, 2, 0};

  auto test_distance_s = std::vector<Length>(num_vertices(G));
  auto test_distance_t = std::vector<Length>(num_vertices(G));

  arlib::details::dijkstra_shortest_path_two_ways(G, s, t, test_distance_s,
                                                  test_distance_t);

  REQUIRE(distance_s == test_distance_s);
  REQUIRE(distance_t == test_distance_t);
}

TEST_CASE("Bidirectional dijkstra works with reverse penalty functor adaptor",
          "[penalty]") {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));
  Vertex s = 0, t = 6;

  // Make a local weight map to avoid modifying existing graph.
  auto original_weight = get(edge_weight, G);
  auto penalty = arlib::details::penalty_functor{original_weight};

  // Forward Dijkstra sp
  auto sp_forward = arlib::details::dijkstra_shortest_path(G, s, t, penalty);
  REQUIRE(sp_forward);

  // Bi-Dijkstra sp
  auto sp_bi =
      arlib::details::bidirectional_dijkstra_shortest_path(G, s, t, penalty);
  REQUIRE(sp_bi);

  REQUIRE(*sp_forward == *sp_bi);
}

TEST_CASE("Penalty running with bidirectional dijkstra returns same result as "
          "unidirectional dijkstra",
          "[penalty]") {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  int k = 3;
  double theta = 0.5;
  auto p = 0.1;
  auto r = 0.1;
  auto bound_limit = 10;
  auto max_nb_steps = 100000;

  auto predecessors_uni = arlib::multi_predecessor_map<Vertex>{};
  arlib::penalty(G, predecessors_uni, s, t, k, theta, p, r, bound_limit,
                 max_nb_steps);
  auto res_paths_uni = arlib::to_paths(G, predecessors_uni, s, t);

  auto predecessors_bi = arlib::multi_predecessor_map<Vertex>{};
  arlib::penalty(G, predecessors_bi, s, t, k, theta, p, r, bound_limit,
                 max_nb_steps, arlib::routing_kernels::bidirectional_dijkstra);
  auto res_paths_bi = arlib::to_paths(G, predecessors_bi, s, t);

  REQUIRE(res_paths_uni.size() == res_paths_bi.size());

  for (std::size_t i = 0; i < res_paths_uni.size(); ++i) {
    REQUIRE(res_paths_uni[i].length() == res_paths_bi[i].length());
  }
}

TEST_CASE("Penalty running with astar returns same result as "
          "unidirectional dijkstra",
          "[penalty]") {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  int k = 3;
  double theta = 0.5;
  auto p = 0.1;
  auto r = 0.1;
  auto bound_limit = 10;
  auto max_nb_steps = 100000;

  auto predecessors_uni = arlib::multi_predecessor_map<Vertex>{};
  arlib::penalty(G, predecessors_uni, s, t, k, theta, p, r, bound_limit,
                 max_nb_steps);
  auto res_paths_uni = arlib::to_paths(G, predecessors_uni, s, t);

  auto predecessors_bi = arlib::multi_predecessor_map<Vertex>{};
  arlib::penalty(G, predecessors_bi, s, t, k, theta, p, r, bound_limit,
                 max_nb_steps, arlib::routing_kernels::astar);
  auto res_paths_bi = arlib::to_paths(G, predecessors_bi, s, t);

  REQUIRE(res_paths_uni.size() == res_paths_bi.size());

  for (std::size_t i = 0; i < res_paths_uni.size(); ++i) {
    REQUIRE(res_paths_uni[i].length() == res_paths_bi[i].length());
  }
}

TEST_CASE("Penalty times-out on large graph", "[penalty]") {
  using namespace boost;
  using namespace std::chrono_literals;

  auto G = arlib::read_graph_from_string<Graph>(std::string(cittastudi_gr));

  Vertex s = 0, t = 20;
  auto k = 3;
  auto theta = 0.5;
  auto p = 0.1;
  auto r = 0.1;
  auto max_nb_updates = 10;
  auto max_nb_steps = 10000;
  auto predecessors = arlib::multi_predecessor_map<Vertex>{};

  REQUIRE_THROWS_AS(arlib::penalty(G, predecessors, s, t, k, theta, p, r,
                                   max_nb_updates, max_nb_steps,
                                   arlib::routing_kernels::astar,
                                   arlib::timer{1us}),
                    arlib::terminator_stop_error);
}