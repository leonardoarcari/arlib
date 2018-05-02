#ifndef BOOST_ONEPASS_PLUS_HPP
#define BOOST_ONEPASS_PLUS_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/graph_types.hpp"
#include "kspwlo/impl/onepass_plus_impl.hpp"

#include <cassert>
#include <iostream>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

/**
 * @brief Algorithms and utilities for Boost::Graph
 */
namespace boost {
/**
 * @brief An implementation of OnePass+ k-shortest path with limited overlap for
 *        Boost::Graph.
 *
 * This implementation refers to the following publication:
 * Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper and Ulf Leser,
 * Exact and Approximate Algorithms for Finding k-Shortest Paths with Limited
 * Overlap , In Proc. of the 20th Int. Conf. on Extending Database Technology
 * (EDBT) (2017)
 *
 * @tparam PropertyGraph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam Vertex A vertex of PropertyGraph.
 * @param G The graph.
 * @param s The source node.
 * @param t The target node.
 * @param k The number of alternative paths to compute.
 * @param theta The similarity threshold.
 *
 * @return A list of at maximum @p k alternative paths.
 */
template <typename PropertyGraph,
          typename Vertex =
              typename boost::graph_traits<PropertyGraph>::vertex_descriptor>
std::vector<kspwlo::Path<PropertyGraph>>
onepass_plus(PropertyGraph &G, Vertex s, Vertex t, int k, double theta) {
  // P_LO set of k paths
  using Length = typename boost::property_traits<typename boost::property_map<
      PropertyGraph, boost::edge_weight_t>::type>::value_type;
  auto weight = get(edge_weight, G);
  auto resPaths = std::vector<kspwlo::Path<PropertyGraph>>{};
  auto resPathsEdges = std::vector<std::vector<kspwlo::Edge>>{};
  auto resPathsLengths = std::vector<Length>{};

  // resEdges keeps track of the edges that make the paths in resPaths and which
  // path includes it.
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;
  using resPathIndex = typename decltype(resPaths)::size_type;
  auto resEdges =
      std::unordered_map<Edge, std::vector<resPathIndex>, hash<Edge>>{};

  // Min-priority queue
  using Label = kspwlo_impl::OnePassLabel<PropertyGraph>;
  using LabelPtr = std::unique_ptr<Label>;
  auto Q = std::priority_queue<
      Label *, std::vector<Label *>,
      kspwlo_impl::OnePassPlusASComparator<PropertyGraph>>{};
  auto created_labels = std::vector<LabelPtr>{};

  // Skyline for dominance checkind (Lemma 2)
  auto skyline = kspwlo_impl::SkylineContainer<PropertyGraph>{};

  // Compute lower bounds for AStar
  auto lower_bounds = kspwlo_impl::distance_from_target(G, t);

  // Compute shortest path from s to t
  auto sp_path = kspwlo_impl::compute_shortest_path(G, s, t);
  auto &sp = sp_path.graph;

  // P_LO <-- {shortest path p_0(s, t)};
  resPaths.push_back(sp_path);
  resPathIndex paths_count = 1;

  // If we need the shortest path only
  if (k == 1) {
    return resPaths;
  }

  // For each edge in the candidate path, we check if it's already in any of the
  // resPaths. If not, we add it to resEdges. If yes, we keep track of which
  // path includes it.
  kspwlo_impl::update_res_edges(sp, G, resEdges, paths_count);

  // Initialize min-priority queue Q with <s, empty_set>
  auto init_label =
      std::make_unique<Label>(s, 0, lower_bounds[s], k, paths_count - 1);
  Q.push(init_label.get());
  created_labels.push_back(std::move(init_label));

  // While Q is not empty
  while (!Q.empty()) {
    // Current path
    auto label = Q.top();
    Q.pop();

    // Perform lazy update of the similairty vector of 'label', since new paths
    // might have been added to P_LO from the time this 'label' was pushed into
    // priority queue.
    if (label->is_outdated(paths_count - 1)) {
      bool below_sim_threshold = kspwlo_impl::update_label_similarity(
          *label, G, resEdges, resPaths, weight, theta, paths_count - 1);

      label->set_last_check(paths_count - 1); // Update last check time step
      if (!below_sim_threshold) {
        continue; // Skip candidate path
      }
    }

    // If we found the target node
    if (label->get_node() == t) {
      // Build the new k-th shortest path
      resPathsEdges.push_back(label->get_path());
      resPathsLengths.push_back(label->get_length());

      auto &tmpPath = resPathsEdges.back();
      ++paths_count;

      if (static_cast<int>(paths_count) == k) { // we found k paths. End.
        // Add computed alternatives to resPaths
        using index = decltype(resPathsEdges.size());
        for (index i = 0; i < resPathsEdges.size(); ++i) {
          resPaths.emplace_back(build_graph_from_edges(resPathsEdges[i], G),
                                resPathsLengths[i]);
        }
        break;
      }

      // For each edge in the candidate path see if it's already in any of the
      // P_LO paths. If not, add it to the resEdges. If so, keep track of which
      // path includes it
      kspwlo_impl::update_res_edges(tmpPath, G, resEdges, paths_count);

    } else { // Expand Search
      if (skyline.dominates(*label)) {
        continue; // Prune path by Lemma 2
      }

      skyline.insert(label);
      auto node_n = label->get_node();
      // For each outgoing edge
      for (auto adj_it = adjacent_vertices(node_n, G).first;
           adj_it != adjacent_vertices(node_n, G).second; ++adj_it) {
        // Expand path
        auto c_edge = edge(node_n, *adj_it, G).first;
        auto c_label =
            kspwlo_impl::expand_path(label, *adj_it, lower_bounds[*adj_it],
                                     weight[c_edge], paths_count - 1);

        // Check for acyclicity
        bool acyclic = label->is_path_acyclic(*adj_it);

        if (acyclic) {
          auto c_similarity_map = label->get_similarity_map();

          // Check Lemma 1 for similarity thresholding
          bool below_sim_threshold = kspwlo_impl::is_below_sim_threshold(
              c_edge, c_similarity_map, theta, resEdges, resPaths, weight);

          if (below_sim_threshold) {
            c_label->set_similarities(std::begin(c_similarity_map),
                                      std::end(c_similarity_map));
            Q.push(c_label.get());
            created_labels.push_back(std::move(c_label));
          }
        }
      }
    }
  }

  if (static_cast<int>(paths_count) != k) {
    // Add computed alternatives to resPaths
    using index = decltype(resPathsEdges.size());
    for (index i = 0; i < resPathsEdges.size(); ++i) {
      resPaths.emplace_back(build_graph_from_edges(resPathsEdges[i], G),
                            resPathsLengths[i]);
    }
  }
  return resPaths;
}
} // namespace boost

#endif