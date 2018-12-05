/**
 * @file graph_utils.hpp
 * @author Leonardo Arcari (leonardo1.arcari@gmail.com)
 * @version 1.0.0
 * @date 2018-10-28
 *
 * @copyright Copyright (c) 2018 Leonardo Arcari
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef BOOST_GRAPH_UTILS_H
#define BOOST_GRAPH_UTILS_H

#include <algorithm>
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

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/graph_types.hpp>
#include <arlib/path.hpp>
#include <arlib/type_traits.hpp>

#include <arlib/multi_predecessor_map.hpp>
#include <boost/graph/compressed_sparse_row_graph.hpp>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
using CSRGraph = boost::compressed_sparse_row_graph<
    boost::bidirectionalS, boost::no_property,
    boost::property<boost::edge_weight_t, double>>;

CSRGraph read_csr_graph_from_string(const std::string &graph);
std::optional<CSRGraph> read_csr_graph_from_file(const std::string_view path);

/**
 * Constructs a PropertyGraph from vertices, edges and weights contained in a
 * .gr-format string. An example of .gr-format string is the following:
 * ```
 * d
 * # nb_vertices nb_edges
 * 3 2
 * # v1 v2 weight
 * 0 1 4
 * 1 2 3
 * ```
 *
 * @tparam PropertyGraph The Graph type
 * @param graph A .gr-format string defining the graph.
 * @return the constructed graph.
 */
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
  // Get edges and weights
  using Length = typename arlib::length_of_t<CSRGraph>;
  auto edges = std::vector<std::pair<long unsigned, long unsigned>>{};
  auto weights = std::vector<Length>{};

  long unsigned s, t;
  Length w;
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

/**
 * Create a string representation of a graph edges and their weight.
 *
 * @tparam Graph A model of EdgeListGraph
 * @param G The graph
 * @return The edges and weights dump
 */
template <typename Graph> std::string dump_edges_weight(const Graph &G) {
  using namespace boost;
  auto ss = std::stringstream{};
  auto weight = get(edge_weight, G);

  for (auto ei = edges(G).first; ei != edges(G).second; ++ei) {
    auto w = weight[*ei];

    ss << "(" << source(*ei, G) << ", " << target(*ei, G) << "; " << w << ") ";
  }

  return ss.str();
}

/**
 * Construct a new Graph from a vector of edges, taking their weights from
 * another graph.
 *
 * @pre Edges in `edge_list` exist in `G`. If not, the behavior is undefined.
 *
 * @tparam Graph The type of the output graph.
 * @param edge_list Vector of `(vertex_id, vertex_id)` edges.
 * @param G The source graph from where to take `edge_list` edges' weights.
 * @return The constructed graph.
 */
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

/**
 * Construct an [Alternative Graph] from a sequence of
 * paths, taking their weights from another source graph.
 *
 * [Alternative Graph]: http://drops.dagstuhl.de/opus/volltexte/2013/4248/
 *
 * @pre Edges in `paths` exist in `g`. If not, the behavior is undefined.
 *
 * @tparam Graph The type of the output graph.
 * @param paths A vector of simple paths.
 * @param g The source graph from where to take `edge_list` edges' weights.
 * @return The constructed graph.
 */
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
    for (auto it = edges(path).first; it != edges(path).second; ++it) {
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

  return Graph{std::begin(es), std::end(es), std::begin(weights),
               num_vertices(g)};
}

/**
 * Implementations details of kSPwLO algorithms
 */
namespace details {
/**
 * Construct a Path from a sequence of vertices of a graph `G`.
 *
 * @pre `vertex_descriptor`s of `path` must be vertices of `G`.
 *
 * @tparam Graph A model of a graph for which `edge(u, v, G)` is a valid
 *         expression.
 * @tparam WeightMap The weight or "length" of each edge in the graph. The
 *         weights must all be non-negative, and the algorithm will throw a
 *         negative_edge exception is one of the edges is negative. The type
 *         WeightMap must be a model of Readable Property Map. The edge
 *         descriptor type of the graph needs to be usable as the key type for
 *         the weight map. The value type for this map must be the same as the
 *         value type of the distance map.
 * @param path A sequence of vertices of `G` representing a simple-path.
 * @param G The graph.
 * @param W The Weight Property Map of `G`.
 * @return The constructed Path.
 */
template <typename Graph, typename WeightMap,
          typename Vertex = vertex_of_t<Graph>>
Path<Graph> build_path_from(std::vector<Vertex> const &path, Graph const &G,
                            WeightMap const &W) {
  using namespace boost;
  using Length = typename property_traits<
      typename property_map<Graph, edge_weight_t>::type>::value_type;
  using Edge = typename graph_traits<Graph>::edge_descriptor;

  auto u = *path.begin();
  auto len = Length{};

  auto path_es = std::unordered_set<Edge, boost::hash<Edge>>{};
  for (auto it = std::next(std::begin(path)); it != std::end(path); ++it) {
    auto v = *it;
    auto [e, is_ok] = edge(u, v, G);
    assert(is_ok && "[arlib::details::build_path_from] Edge not found.");
    auto weight = W[e];
    len += weight;
    path_es.insert(e);
    u = v;
  }

  auto path_vs = std::unordered_set<Vertex, boost::hash<Vertex>>{path.cbegin(),
                                                                 path.cend()};
  auto edge_pred = alternative_path_edges{std::move(path_es)};
  auto vertex_pred = alternative_path_vertices{std::move(path_vs)};
  using FilteredGraph = filtered_graph<Graph, alternative_path_edges<Edge>,
                                       alternative_path_vertices<Vertex>>;
  auto fg = std::make_shared<FilteredGraph>(G, edge_pred, vertex_pred);
  return Path{fg, len};
}

/**
 * Construct a Path from a sequence of vertices of a graph `G`.
 *
 * @pre `vertex_descriptor`s of `path` must be vertices of `G`.
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @param path A sequence of vertices of `G` representing a simple-path.
 * @param G The graph.
 * @return The constructed Path.
 */
template <typename Graph, typename Vertex = vertex_of_t<Graph>>
Path<Graph> build_path_from(std::vector<Vertex> const &path, Graph const &G) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  BOOST_CONCEPT_ASSERT((PropertyGraphConcept<Graph, Edge, edge_weight_t>));

  auto W = get(edge_weight, G);
  return build_path_from(path, G, W);
}

} // namespace details

/**
 * Construct a sequence of Path from a graph `G` a pair of source-target
 * vertices a multi_predecessor_map holding alternative paths from `s` to `t`.
 *
 * multi_predecessor_map is often filled by *alternative routing* algorithms
 * like onepass_plus(), esx() or penalty().
 *
 * @tparam Graph A model of a graph for which `edge(u, v, G)` is a valid
 *         expression.
 * @tparam WeightMap The weight or "length" of each edge in the graph. The
 *         weights must all be non-negative, and the algorithm will throw a
 *         negative_edge exception is one of the edges is negative. The type
 *         WeightMap must be a model of Readable Property Map. The edge
 *         descriptor type of the graph needs to be usable as the key type for
 *         the weight map. The value type for this map must be the same as the
 *         value type of the distance map.
 * @param G The graph.
 * @param pmap The multi predecessor map of `G`.
 * @param weight The Weight Property Map of `G`.
 * @param s The source vertex.
 * @param t The target vertex.
 * @return A sequence of constructed Path.
 */
template <typename Graph, typename WeightMap,
          typename Vertex = vertex_of_t<Graph>>
std::vector<Path<Graph>> to_paths(Graph const &G,
                                  multi_predecessor_map<Vertex> &pmap,
                                  WeightMap const &weight, Vertex s, Vertex t) {
  auto res = std::vector<Path<Graph>>{};
  auto Q = std::stack<std::pair<int, std::vector<Vertex>>>{};
  for (auto const &[k_th, v] : get(pmap, t)) {
    Q.push({k_th, {t}});
  }

  while (!Q.empty()) {
    auto [k_th, path] = std::move(Q.top());
    Q.pop();
    if (path.front() != s) {
      auto const &preds = get(pmap, path.front());
      // Stack a pair only for predecessor of same k_th
      for (auto const &[k_prime, pred] : preds) {
        if (k_prime == k_th) {
          auto path_prime = path;
          path_prime.insert(path_prime.begin(), pred);
          Q.push({k_prime, std::move(path_prime)});
        }
      }
    } else {
      res.push_back(details::build_path_from(path, G, weight));
    }
  }

  std::sort(res.begin(), res.end(), [](auto const &a, auto const &b) {
    return a.length() < b.length();
  });
  return res;
}

/**
 * Construct a sequence of Path from a graph `G` a pair of source-target
 * vertices a multi_predecessor_map holding alternative paths from `s` to `t`.
 *
 * multi_predecessor_map is often filled by *alternative routing* algorithms
 * like onepass_plus(), esx() or penalty().
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @param G The graph.
 * @param pmap The multi predecessor map of `G`.
 * @param s The source vertex.
 * @param t The target vertex.
 * @return A sequence of constructed Path.
 */
template <typename Graph, typename Vertex = vertex_of_t<Graph>>
std::vector<Path<Graph>> to_paths(Graph const &G,
                                  multi_predecessor_map<Vertex> &pmap, Vertex s,
                                  Vertex t) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  BOOST_CONCEPT_ASSERT((PropertyGraphConcept<Graph, Edge, edge_weight_t>));

  auto weight = get(edge_weight, G);
  return to_paths(G, pmap, weight, s, t);
}
} // namespace arlib

#endif