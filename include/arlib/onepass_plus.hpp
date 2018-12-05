/**
 * @file onepass_plus.hpp
 * @author Leonardo Arcari (leonardo1.arcari@gmail.com)
 * @version 1.0.0
 * @date 2018-10-28
 *
 * @copyright Copyright (c) 2018 Leonardo Arcari
 *
 * MIT Licence
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

#ifndef BOOST_ONEPASS_PLUS_HPP
#define BOOST_ONEPASS_PLUS_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/terminators.hpp>
#include <arlib/type_traits.hpp>

#include <arlib/details/onepass_plus_impl.hpp>

#include <cassert>
#include <iostream>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
/**
 * An implementation of OnePass+ k-shortest path with limited overlap for
 * Boost::Graph.
 *
 * This implementation refers to the following publication:
 * Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper and Ulf Leser,
 * Exact and Approximate Algorithms for Finding k-Shortest Paths with Limited
 * Overlap , In Proc. of the 20th Int. Conf. on Extending Database Technology
 * (EDBT) (2017)
 *
 * @tparam Graph A Boost::VertexAndEdgeListGraph
 * @tparam WeightMap The weight or "length" of each edge in the graph. The
 *         weights must all be non-negative, and the algorithm will throw a
 *         negative_edge exception is one of the edges is negative. The type
 *         WeightMap must be a model of Readable Property Map. The edge
 *         descriptor type of the graph needs to be usable as the key type for
 *         the weight map. The value type for this map must be the same as the
 *         value type of the distance map.
 * @tparam MultiPredecessorMap The multi predecessor map records the edges in
 *         the alternative paths from @p s to @p t. Upon completion of the
 *         algorithm, for any vertex `v` on any alternative path `p`, multi
 *         predecessor map stores the predecessor of node `v` on path `p`. If no
 *         predecessor for a node `v'` is reported, then `v'` is not part of any
 *         alternative path or it is the source node @p s. The type of
 *         `MultiPredecessorMap` must be a model of `Read Property Map`. The
 *         vertex descriptor of the input graph @p G must be usable as a key
 *         type for the multi predecessor map. Whereas the value type must
 *         satisfy the `UnorderedAssociativeContainer` concept, where its key is
 *         an `int`, the index/number of alternative path for which it exists a
 *         predecessor of `v`, and where the value type is a vertex descriptor
 *         of input graph @p G.
 * @tparam Vertex The vertex descriptor.
 * @param G The input graph.
 * @param weight The weight map of @p G.
 * @param predecessors The multi predecessor map of @p G.
 * @param s The source node.
 * @param t The target node.
 * @param k The number of alternative paths to compute.
 * @param theta The similarity threshold.
 *
 */
template <typename Graph, typename WeightMap, typename MultiPredecessorMap,
          typename Terminator = arlib::always_continue,
          typename Vertex = vertex_of_t<Graph>>
void onepass_plus(const Graph &G, WeightMap weight,
                  MultiPredecessorMap &predecessors, Vertex s, Vertex t, int k,
                  double theta, Terminator &&terminator = Terminator{}) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  using Length = typename boost::property_traits<WeightMap>::value_type;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<WeightMap, Edge>));
  BOOST_CONCEPT_ASSERT(
      (ReadablePropertyMapConcept<MultiPredecessorMap, Vertex>));

  auto resPathsEdges = std::vector<std::vector<Edge>>{};

  // resEdges keeps track of the edges that make the paths in resPaths and which
  // path includes it.
  using resPathIndex = typename decltype(resPathsEdges)::size_type;
  auto resEdges =
      std::unordered_map<Edge, std::vector<resPathIndex>, boost::hash<Edge>>{};

  // Min-priority queue
  using Label = details::OnePassLabel<Graph, Length>;
  using LabelPtr = std::unique_ptr<Label>;
  auto Q =
      std::priority_queue<Label *, std::vector<Label *>,
                          details::OnePassPlusASComparator<Graph, Length>>{};
  auto created_labels = std::vector<LabelPtr>{};

  // Skyline for dominance checkind (Lemma 2)
  auto skyline = details::SkylineContainer<Graph, Length>{};

  // Compute lower bounds for AStar
  auto lower_bounds = details::distance_from_target<Length>(G, t);

  // Compute shortest path from s to t
  auto sp_path = details::compute_shortest_path(G, weight, s, t);
  if (!sp_path) {
    auto oss = std::ostringstream{};
    oss << "Vertex " << t << " is unreachable from " << s;
    throw details::target_not_found{oss.str()};
  }

  // P_LO <-- {shortest path p_0(s, t)};
  resPathsEdges.push_back(*sp_path);
  resPathIndex paths_count = 1;

  // If we need the shortest path only
  if (k == 1) {
    details::fill_multi_predecessor(resPathsEdges.begin(), resPathsEdges.end(),
                                    G, predecessors);
    return;
  }

  // For each edge in the candidate path, we check if it's already in any of the
  // resPaths. If not, we add it to resEdges. If yes, we keep track of which
  // path includes it.
  details::update_res_edges(*sp_path, resEdges, paths_count);

  // Initialize min-priority queue Q with <s, empty_set>
  auto init_label =
      std::make_unique<Label>(s, 0, lower_bounds[s], k, paths_count - 1);
  Q.push(init_label.get());
  created_labels.push_back(std::move(init_label));

  // While Q is not empty
  while (!Q.empty()) {
    // The remainder code is the hot part of the algorithm. So we check here
    // if the algorithm should terminate
    if (terminator.should_stop()) {
      throw terminator_stop_error{
          "OnePass+ terminated before completing due to a Terminator. Please "
          "discard partial output."};
    }

    // Current path
    auto label = Q.top();
    Q.pop();

    // Perform lazy update of the similairty vector of 'label', since new paths
    // might have been added to P_LO from the time this 'label' was pushed into
    // priority queue.
    if (label->is_outdated(paths_count - 1)) {
      bool below_sim_threshold = details::update_label_similarity(
          *label, G, resEdges, resPathsEdges, weight, theta, paths_count);

      label->set_last_check(paths_count - 1); // Update last check time step
      if (!below_sim_threshold) {
        continue; // Skip candidate path
      }
    }

    // If we found the target node
    if (label->get_node() == t) {
      // Build the new k-th shortest path
      resPathsEdges.push_back(label->get_path(G));

      auto &tmpPath = resPathsEdges.back();
      ++paths_count;

      if (static_cast<int>(paths_count) == k) { // we found k paths. End.
        // Add computed alternatives to resPaths
        details::fill_multi_predecessor(resPathsEdges.begin(),
                                        resPathsEdges.end(), G, predecessors);
        break;
      }

      // For each edge in the candidate path see if it's already in any of the
      // P_LO paths. If not, add it to the resEdges. If so, keep track of which
      // path includes it
      details::update_res_edges(tmpPath, resEdges, paths_count);

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
            details::expand_path(label, *adj_it, lower_bounds[*adj_it],
                                 weight[c_edge], paths_count - 1);

        // Check for acyclicity
        bool acyclic = label->is_path_acyclic(*adj_it);

        if (acyclic) {
          auto c_similarity_map = label->get_similarity_map();

          // Check Lemma 1 for similarity thresholding
          bool below_sim_threshold = details::is_below_sim_threshold(
              c_edge, c_similarity_map, theta, resEdges, resPathsEdges, weight);

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
    details::fill_multi_predecessor(resPathsEdges.begin(), resPathsEdges.end(),
                                    G, predecessors);
  }
}

/**
 * An implementation of OnePass+ k-shortest path with limited overlap for
 * Boost::Graph.
 *
 * This overload takes an input graph modeling `PropertyGraph` concept having at
 * least one edge property with tag `boost::edge_weight_t`. Moreover it does not
 * require an explicit `WeightMap` parameter, because it is directly gathered
 * from the `PropertyGraph`.
 *
 * @see onepass_plus(const Graph &G, WeightMap weight,
 *                   MultiPredecessorMap &predecessors, Vertex s, Vertex t, int
 *                   k, double theta)
 *
 * @tparam PropertyGraph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 *
 */
template <typename PropertyGraph, typename MultiPredecessorMap,
          typename Terminator = arlib::always_continue,
          typename Vertex = vertex_of_t<PropertyGraph>>
void onepass_plus(const PropertyGraph &G, MultiPredecessorMap &predecessors,
                  Vertex s, Vertex t, int k, double theta,
                  Terminator &&terminator = Terminator{}) {
  using namespace boost;
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;

  BOOST_CONCEPT_ASSERT(
      (PropertyGraphConcept<PropertyGraph, Edge, edge_weight_t>));

  auto weight = get(edge_weight, G);
  onepass_plus(G, weight, predecessors, s, t, k, theta,
               std::forward<Terminator>(terminator));
}
} // namespace arlib

#endif