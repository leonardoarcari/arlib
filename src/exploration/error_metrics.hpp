#ifndef ERROR_METRICS_MARGOT_H
#define ERROR_METRICS_MARGOT_H

#include <numeric>
#include <vector>

#include "../model/graph.hpp"

namespace margot {

/**
 * @brief      Computes the average length of k paths.
 *
 * @param[in]  k_paths  A vector of the k paths.
 *
 * @return     The average length.
 */
float k_paths_avg_length(const std::vector<Path> &k_paths) {
  int k = k_paths.size();

  int sum = std::accumulate(
      k_paths.begin(), k_paths.end(), 0,
      [](const int &acc, const Path &path) { return acc + path.length; });

  return sum / k;
}
} // namespace margot

#endif // ERROR_METRICS_MARGOT_H