#ifndef BOOST_GRAPH_UTILS_H
#define BOOST_GRAPH_UTILS_H

#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "kspwlo/graph_types.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

namespace boost {
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
  auto edges = std::vector<kspwlo::Edge>{};
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
std::optional<PropertyGraph>
read_graph_from_file(const std::string_view &path) {
  namespace fs = std::experimental::filesystem;
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

template <typename PropertyGraph>
std::string dump_edges_weight(const PropertyGraph &G);

template <> std::string dump_edges_weight(const kspwlo::Graph &G);
} // namespace boost

#endif