#ifndef TEST_UTILS_HPP
#define TEST_UTILS_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo_ref/algorithms/kspwlo.hpp"

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