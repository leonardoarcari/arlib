#include "catch.hpp"

#include "cittastudi_graph.hpp"
#include "test_types.hpp"
#include "utils.hpp"

#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>
#include <arlib/penalty.hpp>
#include <arlib/reorder_buffer.hpp>

#include <chrono>
#include <random>

using namespace arlib::test;

auto find_alternative_paths(int k = 3, double theta = 0.5) {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(cittastudi_gr));

  Vertex s = 0, t = 20;
  auto p = 0.1;
  auto r = 0.1;
  auto bound_limit = 100;
  auto max_nb_steps = 100;

  auto res_paths =
      penalty_ag(G, s, t, k, theta, p, r, bound_limit, max_nb_steps,
                 arlib::shortest_path_algorithm::dijkstra);
  return res_paths;
}

TEST_CASE("ReorderBuffer::by_length sorts on path length") {
  auto paths = find_alternative_paths();

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
  auto paths = find_alternative_paths(10, 0.999999);

  // Sort by length
  arlib::ReorderBuffer::by_length(paths.begin(), paths.end());
  // Sort on similarity thresholds
  arlib::ReorderBuffer::by_relative_similarity(paths.begin(), paths.end(), 5,
                                               0.5);
  for (std::size_t i = 1; i < paths.size(); ++i) {
    std::cout << "[Path " << i << "]\n";
    for (std::size_t j = 0; j < i; ++j) {
      std::cout << "  Similarity with path " << j << ": "
                << arlib::details::compute_similarity(
                       paths[i], paths[j],
                       boost::get(boost::edge_weight, paths[j].graph()))
                << "\n";
    }
  }
}

TEST_CASE("Penalty+Reordering runs faster than Plain Penalty") {
  auto t1 = std::chrono::steady_clock::now();
  auto plain_penalty = find_alternative_paths(5, 0.5);
  auto t2 = std::chrono::steady_clock::now();

  auto t3 = std::chrono::steady_clock::now();
  auto paths = find_alternative_paths(10, 0.999999);

  // Sort by length
  arlib::ReorderBuffer::by_length(paths.begin(), paths.end());

  // Sort on similarity thresholds
  arlib::ReorderBuffer::by_relative_similarity(paths.begin(), paths.end(), 5,
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