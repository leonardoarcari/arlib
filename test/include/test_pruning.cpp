#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/properties.hpp>

#include "cittastudi_graph.hpp"
#include "test_types.hpp"
#include "utils.hpp"

#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>
#include <arlib/uninformed_bidirectional_pruning.hpp>

#include <experimental/filesystem>
#include <memory>
#include <string>

#include <string_view>

using namespace arlib::test;

TEST_CASE("Uninformed Bidirectional Pruning with tau = 1.0 on a small graph do "
          "not prune nodes",
          "[pruning]") {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  double tau = 1.0;

  auto pruned_G = arlib::uninformed_bidirectional_pruner(G, s, t, tau);

  std::cout << "*** Pruned graph ***\n|V| = " << num_vertices(pruned_G) << "\n";
  print_graph(pruned_G);
  std::cout << "\n";

  REQUIRE(num_edges(pruned_G) == num_edges(G));
}

TEST_CASE("Uninformed Bidirectional Pruning with tau = 1.0 on a real world "
          "network prunes nodes",
          "[pruning]") {
  using namespace boost;

  auto G = arlib::read_graph_from_string<Graph>(std::string(cittastudi_gr));

  Vertex s = 0, t = 20;
  double tau = 1.0;

  auto pruned_G = arlib::uninformed_bidirectional_pruner(G, s, t, tau);
  const auto [first, last] = edges(pruned_G);
  const auto num_pruned_edges = std::distance(first, last);
  REQUIRE(num_pruned_edges < num_edges(G));
}
