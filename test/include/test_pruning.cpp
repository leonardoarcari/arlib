#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/graph_pruning.hpp"
#include "kspwlo/graph_types.hpp"
#include "kspwlo/graph_utils.hpp"
#include "utils.hpp"
#include "cittastudi_graph.hpp"

#include <experimental/filesystem>
#include <memory>
#include <string>

#include <string_view>

TEST_CASE("Uninformed Bidirectional Pruning with tau = 1.0 on a small graph do not prune nodes","[pruning]") {
  using namespace boost;
  using kspwlo::Vertex;

  auto G = read_graph_from_string<kspwlo::Graph>(std::string(graph_gr_esx));

  Vertex s = 0, t = 6;
  double tau = 1.0;

  auto pruned_G = uninformed_bidirectional_pruner(G, s, t, tau);

  std::cout << "*** Pruned graph ***\n|V| = " << num_vertices(pruned_G) << "\n";
  print_graph(pruned_G);
  std::cout << "\n";

  REQUIRE(num_vertices(pruned_G) == num_vertices(G));
  REQUIRE(num_edges(pruned_G) == num_edges(G));
}

TEST_CASE("Uninformed Bidirectional Pruning with tau = 1.0 on a real world network prunes nodes","[pruning]") {
  using namespace boost;
  using kspwlo::Vertex;

  auto G = read_graph_from_string<kspwlo::Graph>(std::string(cittastudi_gr));

  Vertex s = 0, t = 20;
  double tau = 1.2;

  auto pruned_G = uninformed_bidirectional_pruner(G, s, t, tau);

  REQUIRE(num_vertices(pruned_G) < num_vertices(G));
  REQUIRE(num_edges(pruned_G) < num_edges(G));
}
