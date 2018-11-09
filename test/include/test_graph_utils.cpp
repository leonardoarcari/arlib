#include "catch.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/graph_types.hpp>
#include <arlib/onepass_plus.hpp>

#include "test_types.hpp"
#include "utils.hpp"
#include <arlib/graph_utils.hpp>

#include <algorithm>
#include <experimental/filesystem>
#include <fstream>
#include <string_view>

using namespace arlib::test;

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

std::vector<Length> build_test_weights() {
  return std::vector<Length>{6, 3, 4, 6, 5, 2, 3, 5, 3, 1, 3, 2};
}

template <typename PropertyGraph, typename Edge,
          typename Length =
              typename boost::property_traits<typename boost::property_map<
                  PropertyGraph, boost::edge_weight_t>::type>::value_type>
void require_correct_weights(const std::vector<Edge> &test_edges,
                             const std::vector<Length> &test_weights,
                             const PropertyGraph &G) {
  using namespace boost;
  auto weights = get(edge_weight, G);
  for (decltype(test_edges.size()) i = 0; i < test_edges.size(); ++i) {
    // Check weights inside graph structure
    auto [u, v] = test_edges[i];
    auto e = edge(u, v, G);

    REQUIRE(e.second);
    REQUIRE(weights[e.first] == test_weights[i]);
  }
}

TEST_CASE("Boost::Graph can be built from .gr strings", "[graph_utils]") {
  auto G = arlib::read_graph_from_string<Graph>(std::string{graph_gr});

  SECTION("Reading gr string builds a graph with same vertices") {
    using namespace boost;
    using Vertex = graph_traits<Graph>::vertex_descriptor;

    auto Vs = std::vector<Vertex>{vertices(G).first, vertices(G).second};

    // Only 7 vertices are in the graph structure
    REQUIRE(num_vertices(G) == 7);

    // All the vertices are in graph structure
    require_vertices_in_graph(std::begin(Vs), std::end(Vs));
  }

  SECTION("Reading gr string builds a graph with edges with correct weights",
          "[graph_utils]") {
    using namespace boost;
    using arlib::VPair;

    auto test_edges = build_test_edges<VPair>();
    auto test_weights = build_test_weights();

    REQUIRE(num_edges(G) == test_edges.size() * 2);

    require_correct_weights(test_edges, test_weights, G);
  }
}

TEST_CASE("Boost::Graph can be built from .gr files", "[graph_utils]") {
  // Create a new tmp file out of graph_gr
  namespace fs = std::experimental::filesystem;
  auto path = fs::temp_directory_path() / std::string{"graph_gr_file.gr"};
  auto of = std::ofstream(path.string());
  of << graph_gr;
  of.close();

  // Parse file
  using namespace boost;
  auto G_opt = arlib::read_graph_from_file<Graph>(path.string());

  SECTION("Reading existing gr files builds a graph with same vertices",
          "[graph_utils]") {
    // File must exist
    REQUIRE(G_opt);
    auto G = *G_opt;

    using Vertex = graph_traits<Graph>::vertex_descriptor;

    auto Vs = std::vector<Vertex>{vertices(G).first, vertices(G).second};

    // Only 7 vertices are in the graph structure
    REQUIRE(num_vertices(G) == 7);

    // All the vertices are in graph structure
    require_vertices_in_graph(std::begin(Vs), std::end(Vs));
  }

  SECTION("Reading existing gr file builds a graph with edges with correct "
          "weights",
          "[graph_utils]") {
    // File must exist
    REQUIRE(G_opt);
    auto G = *G_opt;

    using arlib::VPair;

    auto test_edges = build_test_edges<VPair>();
    auto test_weights = build_test_weights();

    REQUIRE(num_edges(G) == test_edges.size() * 2);

    require_correct_weights(test_edges, test_weights, G);
  }

  SECTION("Reading not-existing gr file returns empty optional") {
    // File must not exist
    auto non_existing_path =
        fs::path("/xyz/bla/bla/come/on/cant/be/existing.gr");
    G_opt = arlib::read_graph_from_file<Graph>(non_existing_path.string());

    REQUIRE(!G_opt);
  }
}

TEST_CASE("Building an AG from k alternative paths doesn't lose info",
          "[graph_utils]") {
  using namespace boost;
  // Build the graph
  auto G = arlib::read_graph_from_string<Graph>(std::string{graph_gr});
  auto weight = get(edge_weight, G);

  // Run kSPwLO
  Vertex s = 0, t = 6;
  auto predecessors = arlib::multi_predecessor_map<Vertex>{};
  arlib::onepass_plus(G, predecessors, s, t, 3, 0.5);
  auto res_paths = arlib::to_paths(G, predecessors, s, t);

  // Build ground truth data
  auto test_edges = std::vector<arlib::VPair>();
  auto test_weights = std::vector<Length>();

  for (auto &path : res_paths) {
    for (auto it = edges(path).first; it != edges(path).second; ++it) {
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

TEST_CASE("CSR Graph can be built from .gr strings", "[graph_utils]") {
  auto G = arlib::read_csr_graph_from_string(std::string{graph_gr});

  SECTION("Reading gr string builds a graph with same vertices") {
    using namespace boost;
    using CSRVertex = graph_traits<arlib::CSRGraph>::vertex_descriptor;

    auto Vs = std::vector<CSRVertex>{vertices(G).first, vertices(G).second};

    Vertex s = 0, t = 6;
    auto predecessors = arlib::multi_predecessor_map<Vertex>{};
    arlib::onepass_plus(G, predecessors, s, t, 3, 0.5);
    auto res_paths = arlib::to_paths(G, predecessors, s, t);

    // Only 7 vertices are in the graph structure
    REQUIRE(num_vertices(G) == 7);

    // All the vertices are in graph structure
    require_vertices_in_graph(std::begin(Vs), std::end(Vs));
  }

  SECTION("Reading gr string builds a graph with edges with correct weights",
          "[graph_utils]") {
    using namespace boost;
    using arlib::VPair;

    auto test_edges = build_test_edges<VPair>();
    auto test_weights = build_test_weights();

    REQUIRE(num_edges(G) == test_edges.size() * 2);

    require_correct_weights(test_edges, test_weights, G);
  }
}