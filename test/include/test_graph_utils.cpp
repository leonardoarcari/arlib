#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/graph_types.hpp"
#include "kspwlo/graph_utils.hpp"
#include "kspwlo/onepass_plus.hpp"

#include "utils.hpp"

#include <algorithm>
#include <experimental/filesystem>
#include <fstream>
#include <string_view>

// Helper functions
template <typename InputIterator>
void require_vertices_in_graph(InputIterator begin, InputIterator end) {
  REQUIRE(std::find(begin, end, 0) != end);
  REQUIRE(std::find(begin, end, 1) != end);
  REQUIRE(std::find(begin, end, 2) != end);
  REQUIRE(std::find(begin, end, 3) != end);
  REQUIRE(std::find(begin, end, 4) != end);
  REQUIRE(std::find(begin, end, 5) != end);
  REQUIRE(std::find(begin, end, 6) != end);
}

template <typename Edge> std::vector<Edge> build_test_edges() {
  return std::vector<Edge>{Edge{0, 1}, Edge{0, 3}, Edge{0, 2}, Edge{1, 6},
                           Edge{3, 4}, Edge{3, 1}, Edge{3, 2}, Edge{2, 4},
                           Edge{3, 5}, Edge{4, 5}, Edge{4, 6}, Edge{5, 6}};
}

std::vector<int> build_test_weights() {
  return std::vector<int>{6, 3, 4, 6, 5, 2, 3, 5, 3, 1, 3, 2};
}

template <typename PropertyGraph, typename Edge>
void require_correct_weights(const std::vector<Edge> &test_edges,
                             const std::vector<int> &test_weights,
                             const PropertyGraph &G) {
  using namespace boost;
  auto weights = get(edge_weight, G);
  for (decltype(test_edges.size()) i = 0; i < test_edges.size(); ++i) {
    // Check weights inside graph structure
    auto[u, v] = test_edges[i];
    auto e = edge(u, v, G);

    REQUIRE(e.second);
    REQUIRE(weights[e.first] == test_weights[i]);
  }
}

TEST_CASE("Boost::Graph can be built from .gr strings", "[boost::graph]") {
  auto G = boost::read_graph_from_string<kspwlo::Graph>(std::string{graph_gr});

  SECTION("Reading gr string builds a graph with same vertices") {
    using namespace boost;
    using Vertex = graph_traits<kspwlo::Graph>::vertex_descriptor;

    auto Vs = std::vector<Vertex>{vertices(G).first, vertices(G).second};

    // Only 7 vertices are in the graph structure
    REQUIRE(num_vertices(G) == 7);

    // All the vertices are in graph structure
    require_vertices_in_graph(std::begin(Vs), std::end(Vs));
  }

  SECTION("Reading gr string builds a graph with edges with correct weights") {
    using namespace boost;
    using kspwlo::Edge;

    auto test_edges = build_test_edges<Edge>();
    auto test_weights = build_test_weights();

    REQUIRE(num_edges(G) == test_edges.size() * 2);

    require_correct_weights(test_edges, test_weights, G);
  }
}

TEST_CASE("Boost::Graph can be built from .gr files", "[boost::graph]") {
  // Create a new tmp file out of graph_gr
  namespace fs = std::experimental::filesystem;
  auto path = fs::temp_directory_path() / std::string{"graph_gr_file.gr"};
  auto of = std::ofstream(path.string());
  of << graph_gr;
  of.close();

  // Parse file
  using namespace boost;
  auto G_opt = read_graph_from_file<kspwlo::Graph>(path.string());

  SECTION("Reading existing gr files builds a graph with same vertices") {
    // File must exist
    REQUIRE(G_opt);
    auto G = *G_opt;

    using Vertex = graph_traits<kspwlo::Graph>::vertex_descriptor;

    auto Vs = std::vector<Vertex>{vertices(G).first, vertices(G).second};

    // Only 7 vertices are in the graph structure
    REQUIRE(num_vertices(G) == 7);

    // All the vertices are in graph structure
    require_vertices_in_graph(std::begin(Vs), std::end(Vs));
  }

  SECTION("Reading existing gr file builds a graph with edges with correct "
          "weights") {
    // File must exist
    REQUIRE(G_opt);
    auto G = *G_opt;

    using kspwlo::Edge;

    auto test_edges = build_test_edges<Edge>();
    auto test_weights = build_test_weights();

    REQUIRE(num_edges(G) == test_edges.size() * 2);

    require_correct_weights(test_edges, test_weights, G);
  }

  SECTION("Reading not-existing gr file returns empty optional") {
    // File must not exist
    auto non_existing_path =
        fs::path("/xyz/bla/bla/come/on/cant/be/existing.gr");
    G_opt = read_graph_from_file<kspwlo::Graph>(non_existing_path.string());

    REQUIRE(!G_opt);
  }
}

TEST_CASE("Building an AG from k alternative paths doesn't lose info",
          "[boost::graph]") {
  using namespace boost;
  // Build the graph
  auto G = read_graph_from_string<kspwlo::Graph>(std::string{graph_gr});
  auto weight = get(edge_weight, G);

  // Run kSPwLO
  kspwlo::Vertex s = 0, t = 6;
  auto res_paths = onepass_plus(G, s, t, 3, 0.5);

  // Build ground truth data
  auto test_edges = std::vector<kspwlo::Edge>();
  auto test_weights = std::vector<kspwlo::Length>();

  for (auto &path : res_paths) {
    auto &path_g = path.graph;
    for (auto it = edges(path_g).first; it != edges(path_g).second; ++it) {
      auto u = source(*it, G);
      auto v = target(*it, G);

      auto e = edge(u, v, G).first; // Assume it exists
      auto w = weight[e];

      test_edges.emplace_back(u, v);
      test_weights.push_back(w);
    }
  }

  // Now build the AG
  auto ag = build_AG(res_paths, G);

  // Check if all the edges are there with the right weights
  require_correct_weights(test_edges, test_weights, ag);
}