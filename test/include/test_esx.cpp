#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/esx.hpp"
#include "kspwlo/graph_types.hpp"
#include "kspwlo/graph_utils.hpp"
#include "kspwlo/impl/esx_impl.hpp"
#include "kspwlo/impl/kspwlo_impl.hpp"
#include "utils.hpp"

#include <string>

TEST_CASE("Edge priority computation", "[esx]") {
  using namespace boost;
  using kspwlo::Vertex;
  using kspwlo_impl::compute_priority;

  auto G = read_graph_from_string<kspwlo::Graph>(std::string(graph_gr_esx));
  Vertex s = 0, t = 6;

  // Compute shortest path from s to t
  auto sp_path = kspwlo_impl::compute_shortest_path(G, s, t);
  auto &sp = sp_path.graph;

  // Compute lower bounds for AStar
  auto heuristic =
      kspwlo_impl::distance_heuristic<kspwlo::Graph, kspwlo::Length>(G, t);

  // We keep a set of deleted-edges
  using Edge = typename graph_traits<kspwlo::Graph>::edge_descriptor;
  auto deleted_edges = std::unordered_set<Edge, hash<Edge>>{};

  // Check that (0, 3) was computed in shortest path
  REQUIRE(edge(0, 3, sp).second);
  auto s_n3 = edge(0, 3, G).first;

  int prio_s_n3 = compute_priority(G, s_n3, heuristic, deleted_edges);
  REQUIRE(prio_s_n3 == 0);

  // Check that (3, 5) was computed in shortest path
  REQUIRE(edge(3, 5, sp).second);
  auto n3_n5 = edge(3, 5, G).first;

  int prio_n3_n5 = compute_priority(G, n3_n5, heuristic, deleted_edges);
  REQUIRE(prio_n3_n5 == 3);

  // Check that (5, 6) was computed in shortest path
  REQUIRE(edge(5, 6, sp).second);
  auto n5_t = edge(5, 6, G).first;

  int prio_n5_t = compute_priority(G, n5_t, heuristic, deleted_edges);
  REQUIRE(prio_n5_t == 0);
}