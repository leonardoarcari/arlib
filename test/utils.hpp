#ifndef TEST_UTILS_HPP
#define TEST_UTILS_HPP

#include <boost/graph/edge_list.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <kspwlo_ref/algorithms/kspwlo.hpp>

#include <arlib/path.hpp>
#include <arlib/type_traits.hpp>

#include <string_view>
#include <vector>

//===----------------------------------------------------------------------===//
//                      Utility functions for testing
//===----------------------------------------------------------------------===//

template <typename Graph>
bool one_regression_path_have_edges(std::vector<Path> &res_regression,
                                    Graph &G) {
  using boost::edges;
  using boost::source;
  using boost::target;
  auto first = edges(G).first;
  auto last = edges(G).second;

  bool has_them = false;
  for (auto &regr_path : res_regression) {
    int edges_count = 0;
    int nb_edges_regr_path_has = 0;
    for (auto it = first; it != last; ++it) {
      ++edges_count;
      NodeID u = source(*it, G);
      NodeID v = target(*it, G);

      if (regr_path.containsEdge(std::make_pair(u, v))) {
        ++nb_edges_regr_path_has;
      }
    }
    if (edges_count == nb_edges_regr_path_has) {
      has_them = true;
      break;
    }
  }
  return has_them;
}

template <typename Graph>
std::vector<arlib::edge_of_t<Graph>>
make_edge_list(arlib::Path<Graph> const &alt_path) {
  using namespace boost;
  auto res = std::vector<arlib::edge_of_t<Graph>>{};
  for (auto [first, last] = edges(alt_path); first != last; ++first) {
    res.push_back(*first);
  }
  return res;
}

template <typename Graph, typename WeightMap>
bool alternative_paths_are_dissimilar(
    std::vector<arlib::Path<Graph>> const &alt_paths, WeightMap const &weight,
    double theta) {
  // Build edge lists
  using Edge = arlib::edge_of_t<Graph>;
  auto edge_lists = std::vector<std::vector<Edge>>{};
  for (auto const &path : alt_paths) {
    edge_lists.push_back(make_edge_list(path));
  }

  // For each path_i, check its similarity with all the other paths.
  for (std::size_t i = 0; i < alt_paths.size(); ++i) {
    auto const &path_i = alt_paths[i];
    auto const &edges_i = edge_lists[i];
    for (std::size_t j = 0; j < i; ++j) {
      auto const &path_j = alt_paths[j];
      auto shared_len = 0.0;
      for (auto const &e : edges_i) {
        auto u = arlib::source(e, path_i);
        auto v = arlib::target(e, path_i);
        auto ok = arlib::edge(u, v, path_j).second;
        if (ok) {
          shared_len += weight[e];
        }
      }
      auto similarity = shared_len / path_j.length();
      if (similarity > theta) {
        return false;
      }
    }
  }
  return true;
}

constexpr std::string_view graph_gr = "d\n"
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
                                      "4 6 3 0\n"
                                      "6 4 3 0\n"
                                      "5 6 2 0\n"
                                      "6 5 2 0\n";

constexpr std::string_view graph_gr_esx = "d\n"
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

#endif