#ifndef BOOST_GRAPH_UTILS_H
#define BOOST_GRAPH_UTILS_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "graph_types.hpp"

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
  using v_size_type = typename graph_traits<kspwlo::Graph>::vertices_size_type;
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
PropertyGraph read_graph_from_file(const std::string &path) {
  return PropertyGraph{};
}

template <typename PropertyGraph>
std::string dump_edges_weight(const PropertyGraph &G);

template <> std::string dump_edges_weight(const kspwlo::Graph &G) {
  auto ss = std::stringstream{};
  auto weight = get(edge_weight, G);

  for (auto ei = edges(G).first; ei != edges(G).second; ++ei) {
    auto w = weight[*ei];

    ss << "(" << source(*ei, G) << ", " << target(*ei, G) << "; " << w << ") ";
  }

  return ss.str();
}
} // namespace boost

#endif