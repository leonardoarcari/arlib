#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "graph_types.hpp"
#include "graph_utils.hpp"

#include <algorithm>

TEST_CASE("Boost::Graph can be built from .gr files", "[boost::graph]") {
  std::string graph_gr = "d\n"
                         "7 24\n"
                         "0 1 6 0\n"
                         "1 0 6 0\n"
                         "0 3 3 0\n"
                         "3 0 3 0\n"
                         "0 2 4 0\n"
                         "2 0 4 0\n"
                         "1 6 6 0\n"
                         "6 1 6 0\n"
                         "3 4 5 0\n"
                         "4 3 5 0\n"
                         "3 1 2 0\n"
                         "1 3 2 0\n"
                         "3 2 3 0\n"
                         "2 3 3 0\n"
                         "2 4 5 0\n"
                         "4 2 5 0\n"
                         "3 5 3 0\n"
                         "5 3 3 0\n"
                         "4 5 1 0\n"
                         "5 4 1 0\n"
                         "4 6 2 0\n"
                         "6 4 2 0\n"
                         "5 6 2 0\n"
                         "6 5 2 0\n";

  auto G = boost::read_graph_from_string<kspwlo::Graph>(graph_gr);

  SECTION("Reading gr file builds a graph with same vertices") {
    using namespace boost;
    using Vertex = graph_traits<kspwlo::Graph>::vertex_descriptor;

    auto Vs = std::vector<Vertex>{vertices(G).first, vertices(G).second};

    auto v_end = std::end(Vs);

    // Only 7 vertices are in the graph structure
    REQUIRE(num_vertices(G) == 7);

    // All the vertices are in graph structure
    REQUIRE(std::find(std::begin(Vs), v_end, 0) != v_end);
    REQUIRE(std::find(std::begin(Vs), v_end, 1) != v_end);
    REQUIRE(std::find(std::begin(Vs), v_end, 2) != v_end);
    REQUIRE(std::find(std::begin(Vs), v_end, 3) != v_end);
    REQUIRE(std::find(std::begin(Vs), v_end, 4) != v_end);
    REQUIRE(std::find(std::begin(Vs), v_end, 5) != v_end);
    REQUIRE(std::find(std::begin(Vs), v_end, 6) != v_end);
  }

  SECTION("Reading gr file builds a graph with edges with correct weights") {
    using namespace boost;
    using kspwlo::Edge;

    auto test_edges = std::vector<Edge>{
        Edge{0, 1}, Edge{0, 3}, Edge{0, 2}, Edge{1, 6}, Edge{3, 4}, Edge{3, 1},
        Edge{3, 2}, Edge{2, 4}, Edge{3, 5}, Edge{4, 5}, Edge{4, 6}, Edge{5, 6}};
    auto test_weights = std::vector<int>{6, 3, 4, 6, 5, 2, 3, 5, 3, 1, 2, 2};

    REQUIRE(num_edges(G) == test_edges.size() * 2);

    auto weights = get(edge_weight, G);
    for (decltype(test_edges.size()) i = 0; i < test_edges.size(); ++i) {
      // Check weights inside graph structure
      auto[u, v] = test_edges[i];
      auto e = edge(u, v, G);

      std::cout << u << " " << v << "\n";

      REQUIRE(e.second);
      REQUIRE(weights[e.first] == test_weights[i]);
    }
  }
}