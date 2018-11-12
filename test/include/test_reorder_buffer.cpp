#include "catch.hpp"

#include <arlib/penalty.hpp>
#include <arlib/reorder_buffer.hpp>
// Types always after or it does not compile.
#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>

#include "cittastudi_graph.hpp"
#include "test_types.hpp"
#include "utils.hpp"

#include <chrono>
#include <random>

using namespace arlib::test;

auto find_alternative_paths(Graph const &G, int k = 3, double theta = 0.5) {
  using namespace boost;

  Vertex s = 0, t = 20;
  auto p = 0.1;
  auto r = 0.1;
  auto bound_limit = 100;
  auto max_nb_steps = 100;

  auto predecessors = arlib::multi_predecessor_map<Vertex>{};
  arlib::penalty(G, predecessors, s, t, k, theta, p, r, bound_limit,
                 max_nb_steps, arlib::routing_kernels::dijkstra);
  auto res_paths = arlib::to_paths(G, predecessors, s, t);
  return res_paths;
}

TEST_CASE("ReorderBuffer::by_length sorts on path length") {
  auto G = arlib::read_graph_from_string<Graph>(std::string(cittastudi_gr));
  auto paths = find_alternative_paths(G, 3, 0.5);

  // Shuffle paths
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(paths.begin(), paths.end(), g);

  // Sort by length
  arlib::ReorderBuffer::by_length(paths.begin(), paths.end());
  REQUIRE(std::is_sorted(
      paths.cbegin(), paths.cend(),
      [](auto const &a, auto const &b) { return a.length() < b.length(); }));
}

TEST_CASE(
    "ReorderBuffer::by_relative_similarity sorts on similarity thresholds") {
  auto G = arlib::read_graph_from_string<Graph>(std::string(cittastudi_gr));
  auto paths = find_alternative_paths(G, 10, 0.999999);

  // Sort by length
  arlib::ReorderBuffer::by_length(paths.begin(), paths.end());
  // Sort on similarity thresholds
  arlib::ReorderBuffer::by_relative_similarity(G, paths.begin(), paths.end(), 5,
                                               0.5);
  // for (std::size_t i = 1; i < paths.size(); ++i) {
  //   std::cout << "[Path " << i << "]\n";
  //   for (std::size_t j = 0; j < i; ++j) {
  //     std::cout << "  Similarity with path " << j << ": "
  //               << arlib::details::compute_similarity(
  //                      paths[i], paths[j], get(boost::edge_weight, paths[j]))
  //               << "\n";
  //   }
  // }
}

TEST_CASE("Penalty+Reordering runs faster than Plain Penalty") {
  auto G = arlib::read_graph_from_string<Graph>(std::string(cittastudi_gr));

  auto t1 = std::chrono::steady_clock::now();
  auto plain_penalty = find_alternative_paths(G, 5, 0.5);
  auto t2 = std::chrono::steady_clock::now();

  auto t3 = std::chrono::steady_clock::now();
  auto paths = find_alternative_paths(G, 10, 0.999999);

  // Sort by length
  arlib::ReorderBuffer::by_length(paths.begin(), paths.end());

  // Sort on similarity thresholds
  arlib::ReorderBuffer::by_relative_similarity(G, paths.begin(), paths.end(), 5,
                                               0.5);
  auto t4 = std::chrono::steady_clock::now();

  REQUIRE((t4 - t3) < (t2 - t1));
  std::cout
      << "[Plain Penalty] "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
      << "ms\n"
      << "[Penalty + Reordering] "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
      << "ms\n";
}