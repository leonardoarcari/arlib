#ifndef BOOST_GRAPH_UTILS_H
#define BOOST_GRAPH_UTILS_H

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stack>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include <arlib/graph_types.hpp>

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

namespace arlib {
template <typename PropertyGraph>
PropertyGraph read_graph_from_string(const std::string &graph) {
  auto ss = std::stringstream{graph};
  std::string line{};

  // Drop graph type info
  std::getline(ss, line);

  // Get number of nodes and edges
  std::getline(ss, line);
  auto line_s = std::stringstream{line};
  int nb_nodes, nb_edges;
  line_s >> nb_nodes >> nb_edges;

  // Get edges and weights
  auto edges = std::vector<VPair>{};
  auto weights = std::vector<int>{};

  int s, t, w;
  while (std::getline(ss, line)) {
    line_s.str(line);
    line_s.seekg(std::ios_base::beg);
    line_s >> s >> t >> w;

    edges.emplace_back(s, t);
    weights.push_back(w);
  }

  // Return a PropertyGraph from data
  auto G = PropertyGraph(std::begin(edges), std::end(edges),
                         std::begin(weights), nb_nodes);

  return G;
}

template <typename PropertyGraph>
std::optional<PropertyGraph> read_graph_from_file(const std::string_view path) {
  namespace fs = std::filesystem;
  auto fs_path = fs::path(path);

  if (!fs::is_regular_file(fs_path)) {
    std::cerr << fs_path << " is not a regular file.\n";
    return {};
  }

  if (fs::is_empty(fs_path)) {
    std::cerr << fs_path << " is empty.\n";
    return {};
  }

  auto buffer = std::stringstream{};
  auto input = std::ifstream{fs_path.string()};
  buffer << input.rdbuf();

  return {read_graph_from_string<PropertyGraph>(buffer.str())};
}

template <typename Graph> std::string dump_edges_weight(const Graph &G);

template <typename Graph>
Graph build_graph_from_edges(const std::vector<VPair> &edge_list,
                             const Graph &G) {
  using namespace boost;
  using Length = typename boost::property_traits<typename boost::property_map<
      Graph, boost::edge_weight_t>::type>::value_type;
  auto weight = get(edge_weight, G);
  auto weights = std::vector<Length>{};

  for (auto &e : edge_list) {
    auto u = e.first;
    auto v = e.second;
    auto edge_in_G = edge(u, v, G).first;
    weights.push_back(weight[edge_in_G]);
  }

  return Graph{std::begin(edge_list), std::end(edge_list), std::begin(weights),
               num_vertices(G)};
}

template <typename Graph>
Graph build_AG(const std::vector<Path<Graph>> &paths, const Graph &g) {
  using namespace boost;
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
  using Length = typename boost::property_traits<typename boost::property_map<
      Graph, boost::edge_weight_t>::type>::value_type;

  auto weight = get(edge_weight, g);
  auto es = std::vector<VPair>{};
  auto weights = std::vector<Length>{};
  auto nodes = std::unordered_set<Vertex>{};

  for (auto &path : paths) {
    auto &path_g = path.graph();
    for (auto it = edges(path_g).first; it != edges(path_g).second; ++it) {
      auto u = source(*it, g);
      auto v = target(*it, g);

      auto e = edge(u, v, g).first; // Assume it exists
      auto w = weight[e];

      es.emplace_back(u, v);
      weights.push_back(w);

      if (nodes.find(u) == std::end(nodes)) {
        nodes.insert(u);
      }

      if (nodes.find(v) == std::end(nodes)) {
        nodes.insert(v);
      }
    }
  }

  return Graph{std::begin(es), std::end(es), std::begin(weights), nodes.size()};
}

namespace details {
template <typename Graph, typename Vertex = typename boost::graph_traits<
                              Graph>::vertex_descriptor>
Path<Graph> build_path_from(std::vector<Vertex> const &path, Graph const &G) {
  using namespace boost;
  using Length = typename property_traits<
      typename property_map<Graph, edge_weight_t>::type>::value_type;

  auto edge_list = std::vector<VPair>{};
  auto u = *path.begin();
  auto W = get(edge_weight, G);
  auto len = Length{};

  for (auto it = std::next(std::begin(path)); it != std::end(path); ++it) {
    auto v = *it;
    auto [e, is_ok] = edge(u, v, G);
    assert(is_ok && "[arlib::details::build_path_from] Edge not found.");
    auto weight = W[e];
    len += weight;
    edge_list.emplace_back(u, v);
    u = v;
  }

  return {build_graph_from_edges(edge_list, G), len};
}
} // namespace details

template <typename Graph, typename Vertex = typename boost::graph_traits<
                              Graph>::vertex_descriptor>
std::vector<Path<Graph>> to_paths(multi_predecessor_map<Vertex> &pmap,
                                  Graph const &G, Vertex s, Vertex t) {
  auto res = std::vector<Path<Graph>>{};
  auto Q = std::stack<std::vector<Vertex>>{};
  Q.push({t});

  while (!Q.empty()) {
    auto path = std::move(Q.top());
    Q.pop();
    if (path.front() != s) {
      auto const &preds = get(pmap, path.front());
      for (auto const &p : preds) {
        auto path_prime = path;
        path_prime.insert(path_prime.begin(), p);
        Q.push(std::move(path_prime));
      }
    } else {
      res.push_back(details::build_path_from(path, G));
    }
  }

  return res;
}
} // namespace arlib

#endif