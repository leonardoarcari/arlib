#ifndef ERROR_METRICS_MARGOT_BOOST_GRAPH_H
#define ERROR_METRICS_MARGOT_BOOST_GRAPH_H

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <kspwlo/graph_types.hpp>
#include <kspwlo/graph_utils.hpp>

#include <numeric>

namespace margot {

struct ErrorMetrics {
  float total_distance;
  float average_distance;
  float decision_edges;
};

float total_distance(kspwlo::Graph &ag, kspwlo::Vertex s, kspwlo::Vertex t) {
  using namespace boost;
  using Length = typename boost::property_traits<typename boost::property_map<
      kspwlo::Graph, boost::edge_weight_t>::type>::value_type;

  auto tot_dist = 0.0f;
  for (auto it = edges(ag).first; it != edges(ag).second; ++it) {
    auto u = source(*it, ag);
    auto v = target(*it, ag);
    auto weight = get(edge_weight, ag);

    auto dists_s = std::vector<Length>(num_vertices(ag));
    auto dists_v = std::vector<Length>(num_vertices(ag));

    dijkstra_shortest_paths(ag, s, distance_map(&dists_s[0]));
    dijkstra_shortest_paths(ag, v, distance_map(&dists_v[0]));

    auto d_s_u = dists_s[u];
    auto d_v_t = dists_v[t];

    float distance =
        weight[*it] / static_cast<float>(d_s_u + weight[*it] + d_v_t);
    tot_dist += distance;
  }
  return tot_dist;
}

float average_distance(kspwlo::Graph &ag, kspwlo::Graph &g, kspwlo::Vertex s,
                       kspwlo::Vertex t) {
  using namespace boost;
  // Sum of the weights
  kspwlo::Length weights_sum = 0;
  auto weight = get(edge_weight, ag);

  for (auto it = edges(ag).first; it != edges(ag).second; ++it) {
    weights_sum += weight[*it];
  }

  auto tot_distance = total_distance(ag, s, t);

  auto dists_g = std::vector<kspwlo::Length>(num_edges(g));
  dijkstra_shortest_paths(g, s, distance_map(&dists_g[0]));
  auto d_G_s_t = dists_g[t];

  return weights_sum / (d_G_s_t * tot_distance);
}

float average_distance(kspwlo::Graph &ag, kspwlo::Graph &g, kspwlo::Vertex s,
                       kspwlo::Vertex t, float tot_distance) {
  using namespace boost;
  // Sum of the weights
  kspwlo::Length weights_sum = 0;
  auto weight = get(edge_weight, ag);

  for (auto it = edges(ag).first; it != edges(ag).second; ++it) {
    weights_sum += weight[*it];
  }

  auto dists_g = std::vector<kspwlo::Length>(num_edges(g));
  dijkstra_shortest_paths(g, s, distance_map(&dists_g[0]));
  auto d_G_s_t = dists_g[t];

  return weights_sum / (d_G_s_t * tot_distance);
}

float decision_edges(kspwlo::Graph &ag, kspwlo::Vertex t) {
  using namespace boost;
  kspwlo::Length d_edges = 0;

  // For all v in vertices
  for (auto v_it = vertices(ag).first; v_it != vertices(ag).second; ++v_it) {
    // Get the out-degree of v (except for the target node)
    if (*v_it != t) {
      auto out_d = out_degree(*v_it, ag);
      d_edges += (out_d == 0 ? 0 : out_d - 1);
    }
  }

  return static_cast<float>(d_edges);
}

ErrorMetrics
compute_errors(const std::vector<kspwlo::Path<kspwlo::Graph>> &paths,
               kspwlo::Graph &g, kspwlo::Vertex s, kspwlo::Vertex t) {
  auto ag = boost::build_AG(paths, g);

  auto tot_distance = total_distance(ag, s, t);
  auto avg_distance = average_distance(ag, g, s, t, tot_distance);
  auto decision_es = decision_edges(ag, t);

  return {tot_distance, avg_distance, decision_es};
}

double
compute_kspwlo_quality(const std::vector<kspwlo::Path<kspwlo::Graph>> &result) {
  auto n_sols = result.size();
  auto sp_len = result.front().length;

  auto len_sum = std::accumulate(
      result.cbegin(), result.cend(), 0.0,
      [](const auto &acc, const auto &path) { return acc + path.length; });
  auto avg_length = static_cast<double>(len_sum) / n_sols;
  auto diff_sp = (avg_length - sp_len) / sp_len;
  return diff_sp;
}
} // namespace margot

#endif