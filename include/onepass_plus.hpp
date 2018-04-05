#ifndef BOOST_ONEPASS_PLUS_HPP
#define BOOST_ONEPASS_PLUS_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/exception.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/topological_sort.hpp>

#include "impl/onepass_plus_impl.hpp"
#include "graph_types.hpp"

#include <cassert>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_set>
#include <vector>

namespace boost {
template <typename PropertyGraph,
          typename Vertex =
              typename boost::graph_traits<PropertyGraph>::vertex_descriptor,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  PropertyGraph, boost::edge_weight_t>::type>::value_type>
std::vector<kspwlo::Path<PropertyGraph>>
onepass_plus(PropertyGraph &G, Vertex s, Vertex t, int k, double theta) {
  // P_LO set of k paths
  auto weight = get(edge_weight, G);
  auto resPaths = std::vector<kspwlo::Path<PropertyGraph>>{};

  // resEdges keeps track of the edges that make the paths in resPaths and which
  // path includes it.
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;
  using resPathIndex = typename decltype(resPaths)::size_type;
  auto resEdges =
      std::unordered_map<Edge, std::vector<resPathIndex>, hash<Edge>>{};

  // Min-priority queue
  using Label = kspwlo_impl::OnePassLabel<PropertyGraph>;
  using LabelPtr = std::shared_ptr<Label>;
  auto Q =
      std::priority_queue<LabelPtr, std::vector<LabelPtr>,
                          kspwlo_impl::OnePassPlusASComparator<PropertyGraph>>{};
  auto created_labels = std::vector<LabelPtr>{};

  // Skyline for dominance checkind (Lemma 2)
  auto skyline = kspwlo_impl::SkylineContainer<PropertyGraph>{};

  // Compute lower bounds for AStar
  auto lower_bounds = kspwlo_impl::distance_from_target(G, t);

  // Compute shortest path from s to t
  auto sp_distances = std::vector<length_type>(num_vertices(G));
  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);
  dijkstra_shortest_paths(G, s,
                          distance_map(&sp_distances[0])
                              .predecessor_map(make_iterator_property_map(
                                  std::begin(predecessor), vertex_id, s)));
  auto sp_path = kspwlo_impl::build_path_from_dijkstra(G, predecessor, s, t);
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
  using kspwlo_impl::track_res_edges;
  track_res_edges(sp, G, resEdges, paths_count);

  std::cout << "resEdges = {";
  for (auto resEdge : resEdges) {
    std::cout << resEdge.first << " : [ ";
    for (auto index : resEdge.second) {
      std::cout << index << " ";
    }
    std::cout << "], ";
  }
  std::cout << "}\n";

  // Initialize min-priority queue Q with <s, empty_set>
  auto init_label =
      std::make_shared<Label>(s, 0, lower_bounds[s], k, paths_count - 1);
  Q.push(init_label);
  created_labels.push_back(init_label);

  // While Q is not empty
  while (!Q.empty()) {
    // Current path
    auto label = Q.top();
    Q.pop();

    std::cout << "label = " << *label << "\n";

    // Perform lazy update of the similairty vector of 'label', since new paths
    // might have been added to P_LO from the time this 'label' was pushed into
    // priority queue.
    if (label->checkedAtStep() < static_cast<int>(paths_count - 1)) {
      bool below_sim_threshold = true;
      auto tmpPath = label->getPath();
      for (auto ei = edges(tmpPath).first; ei != edges(tmpPath).second; ++ei) {
        auto edge_in_g =
            edge(source(*ei, tmpPath), target(*ei, tmpPath), G).first;
        auto search = resEdges.find(edge_in_g);
        // if tmpPath share an edge with any k-th shortest path, update the
        // overlapping factor
        if (search != std::end(resEdges)) {
          for (auto index : search->second) {
            if (static_cast<int>(index) > label->checkedAtStep() &&
                index <= paths_count) {
              label->getSimilarityWith(index) += weight[edge_in_g];

              // Check Lemma 1. The similarity between the candidate path and
              // all the other k-shortest-paths must be less then theta
              if (label->getSimilarityWith(index) /
                      resPaths[paths_count - 1].length >
                  theta) {
                below_sim_threshold = false;
                break;
              }
            }
          }
        }
      }

      label->setCheckedAtStep(paths_count - 1); // Update last check time step
      std::cout << "updated label = " << *label << "\n";
      if (!below_sim_threshold) {
        continue; // Skip candidate path
      }
    }

    // If we found the target node
    if (label->getNode() == t) {
      // Build the new k-th shortest path
      auto resPath = kspwlo::Path(label->getPath(), label->getLength());
      auto &tmpPath = resPath.graph;
      resPaths.push_back(resPath);
      ++paths_count;

      std::cout << "Adding path from label: " << *label << "\n";

      if (static_cast<int>(paths_count) == k) { // we found k paths. End.
        break;
      }

      // For each edge in the candidate path see if it's already in any of the
      // P_LO paths. If not, add it to the resEdges. If so, keep track of which
      // path includes it
      track_res_edges(tmpPath, G, resEdges, paths_count);

      std::cout << "resEdges = {";
      for (auto resEdge : resEdges) {
        std::cout << resEdge.first << " : [ ";
        for (auto index : resEdge.second) {
          std::cout << index << " ";
        }
        std::cout << "], ";
      }
      std::cout << "}\n";
    } else { // Expand Search
      if (skyline.dominates(*label)) {
        continue; // Prune path by Lemma 2
      }

      skyline.insert(label);
      auto node_n = label->getNode();
      // For each outgoing edge
      for (auto adj_it = adjacent_vertices(node_n, G).first;
           adj_it != adjacent_vertices(node_n, G).second; ++adj_it) {
        // Check for acyclicity
        bool containsLoop = false;
        auto c_edge = edge(node_n, *adj_it, G).first; // Assume edge exists
        std::cout << "Exploring edge: " << c_edge << "\n";
        std::cout << "Candidate edge: (" << source(c_edge, G) << ", "
                  << target(c_edge, G) << ", " << weight[c_edge] << ")\n";
        auto tmpLength = label->getLength() + weight[c_edge];
        auto tmpLowerBound = tmpLength + lower_bounds[*adj_it];
        auto tmpLabel = std::make_shared<Label>(
            *adj_it, tmpLength, tmpLowerBound, label, k, paths_count - 1);
        auto tmpPath = tmpLabel->getPath();
        auto top_ordering = std::vector<Vertex>{};
        try {
          topological_sort(tmpPath, std::back_inserter(top_ordering));
        } catch (not_a_dag e) {
          // If not acyclic, continue
          containsLoop = true;
        }

        if (!containsLoop) {
          auto tmpSimilarityMap = label->getSimilarityMap();
          // Check Lemma 1 for similarity thresholding
          bool below_sim_threshold = true;
          auto search = resEdges.find(c_edge);
          if (search != std::end(resEdges)) {
            std::cout << "Found c_edge " << c_edge << "\n";
            auto &res_paths_with_c_edge = search->second;
            for (auto index : res_paths_with_c_edge) {
              tmpSimilarityMap[index] += weight[c_edge];
              auto similarity =
                  tmpSimilarityMap[index] / resPaths[index].length;
              std::cout << "Similarity = " << similarity << "\n";
              if (similarity > theta) {
                below_sim_threshold = false;
                break;
              }
            }
          }

          std::cout << "Similarities: [ ";
          for (auto sim : tmpSimilarityMap) {
            std::cout << sim << " ";
          }
          std::cout << "]\n";

          tmpLabel->importSimilarities(std::begin(tmpSimilarityMap),
                                       std::end(tmpSimilarityMap));
          std::cout << "Candidate path: " << *tmpLabel << '\n'
                    << "below_sim_threshold = " << below_sim_threshold << "\n";

          if (below_sim_threshold) {
            tmpLabel->importSimilarities(std::begin(tmpSimilarityMap),
                                         std::end(tmpSimilarityMap));
            tmpLabel->getPath();
            Q.push(tmpLabel);
            created_labels.push_back(tmpLabel);
          }
        }
      }
    }
  }

  return resPaths;
}
} // namespace boost

#endif