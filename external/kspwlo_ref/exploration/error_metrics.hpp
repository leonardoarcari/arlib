#ifndef ERROR_METRICS_MARGOT_H
#define ERROR_METRICS_MARGOT_H

#include <iostream>
#include <numeric>
#include <vector>

#include "../model/graph.hpp"
#include "../tools/tools.hpp"
#include "kspwlo_ref/exploration/graph_utils.hpp"

namespace margot {

struct ErrorMetrics {
  float total_distance;
  float average_distance;
  float decision_edges;
};

/**
 * @brief      Computes the average length of k paths.
 *
 * @param[in]  k_paths  A vector of the k paths.
 *
 * @return     The average length.
 */
float k_paths_avg_length(const std::vector<Path> &k_paths) {
  auto k = k_paths.size();

  int sum = std::accumulate(
      k_paths.begin(), k_paths.end(), 0,
      [](const int &acc, const Path &path) { return acc + path.length; });

  return sum / k;
}

float totalDistance(RoadNetwork &ag, NodeID source, NodeID target) {
  auto total_distance = 0.0f;
  using index = decltype(ag.adjListOut.size());

#pragma omp parallel for reduction(+ : total_distance)
  for (index i = 0; i < ag.adjListOut.size(); ++i) {
    NodeID u = i;
    auto &outEdges = ag.adjListOut[u];
    for (auto &[v, weight] : outEdges) {
      int d_s_u = dijkstra_path_and_bounds(&ag, source, u).first.length;
      int d_v_t = dijkstra_path_and_bounds(&ag, v, target).first.length;

      float distance = weight / static_cast<float>(d_s_u + weight + d_v_t);
      total_distance += distance;

      // std::cerr << "s = " << source << ", t = " << target << ", u = " << u
      //           << ", v = " << v << ", w = " << weight << ", d_s_u = " <<
      //           d_s_u
      //           << ", d_v_t = " << d_v_t << ", distance = " << distance
      //           << ", total_distance = " << total_distance << "\n";
    }
  }

  std::cerr << "Check total_distance: " << total_distance << "\n";

  return total_distance;
}

float averageDistance(RoadNetwork &ag, RoadNetwork &g, NodeID source,
                      NodeID target) {
  // Sum of weights
  auto weights_sum = 0;
  for (auto &edge_list : ag.adjListOut) {
    for (auto &edge : edge_list) {
      weights_sum += edge.second;
    }
  }

  auto total_distance = totalDistance(ag, source, target);
  int d_G_s_t = dijkstra_path_and_bounds(&g, source, target).first.length;

  return weights_sum / (d_G_s_t * total_distance);
}

float averageDistance(RoadNetwork &ag, RoadNetwork &g, NodeID source,
                      NodeID target, float total_distance) {
  // Sum of weights
  auto weights_sum = 0;
  for (auto &edge_list : ag.adjListOut) {
    for (auto &edge : edge_list) {
      weights_sum += edge.second;
    }
  }

  int d_G_s_t = dijkstra_path_and_bounds(&g, source, target).first.length;

  return weights_sum / (d_G_s_t * total_distance);
}

float decisionEdges(RoadNetwork &ag, NodeID target) {
  auto decision_edges = 0;

  using index = decltype(ag.adjListOut.size());
  for (index i = 0; i < ag.adjListOut.size(); ++i) {
    if (i != target) { // Sum over all vertices but the target
      // out-degree of node i
      auto out_degree = ag.adjListOut[i].size();
      decision_edges += (out_degree == 0 ? 0 : out_degree - 1);
    }
  }

  return static_cast<float>(decision_edges);
}

ErrorMetrics compute_errors(vector<Path> &result, RoadNetwork &g, NodeID source,
                            NodeID target) {
  auto ag = build_AG(result, g);

  auto total_distance = totalDistance(ag, source, target);
  auto average_distance =
      averageDistance(ag, g, source, target, total_distance);
  auto decision_edges = decisionEdges(ag, target);

  return {total_distance, average_distance, decision_edges};
}

auto parse_metric(const string &metric) {
  auto metric_f =
      std::function<float(vector<Path> &, RoadNetwork &, NodeID, NodeID)>{};

  if (metric == "totalDistance") {
    metric_f = [](vector<Path> &result, RoadNetwork &g, NodeID source,
                  NodeID target) {
      auto ag = build_AG(result, g);
      return margot::totalDistance(ag, source, target);
    };
  } else if (metric == "averageDistance") {
    metric_f = [](vector<Path> &result, RoadNetwork &g, NodeID source,
                  NodeID target) {
      auto ag = build_AG(result, g);
      return margot::averageDistance(ag, g, source, target);
    };
  } else if (metric == "decisionEdges") {
    metric_f = [](vector<Path> &result, RoadNetwork &g, NodeID, NodeID target) {
      auto ag = build_AG(result, g);
      return margot::decisionEdges(ag, target);
    };
  }

  return metric_f;
}

} // namespace margot

#endif // ERROR_METRICS_MARGOT_H