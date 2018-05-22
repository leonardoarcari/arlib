#ifndef KSPWLO_BIDIRECTIONAL_DIJKSTRA_IMPL_HPP
#define KSPWLO_BIDIRECTIONAL_DIJKSTRA_IMPL_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/property_map.hpp>

#include "kspwlo/visitor.hpp"

#include <deque>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

/**
 * @brief Implementations details of kSPwLO algorithms
 */
namespace kspwlo_impl {
//===----------------------------------------------------------------------===//
//                      Bidirectional Dijkstra types
//===----------------------------------------------------------------------===//

/**
 * @brief Enum to describe whether a forward or a backward step of bidirectional
 * dijkstra is executing.
 */
enum class Direction { forward = 1, backward };

/**
 * @brief Returns the opposite direction.
 *
 * @param prev_dir The current execution direction.
 * @return Direction::forward if @p prev_dir == Direction::backward.
 *         Direction::backward otherwise.
 */
constexpr Direction switch_direction(Direction prev_dir) {
  return (prev_dir == Direction::backward) ? Direction::forward
                                           : Direction::backward;
}

/**
 * @brief Return code of bi_dijkstra_step()
 */
enum class BiDijkStepRes {
  next = 1,        /**< bidirectional_dijkstra() should execute another step. */
  end,             /**< bidirectional_dijkstra() should end. */
  negative_weights /**< bidirectional_dijkstra() should exit because
                        preconditions are violated. */
};

//===----------------------------------------------------------------------===//
//                      Bidirectional Dijkstra  aliases
//===----------------------------------------------------------------------===//
template <typename Vertex>
using PathMap =
    std::unordered_map<Vertex, std::vector<Vertex>, boost::hash<Vertex>>;

template <typename Vertex, typename Length>
using FringeElem = std::pair<Vertex, Length>;

template <typename Vertex, typename Length> struct FringeComparator {
  using Elem = FringeElem<Vertex, Length>;
  bool operator()(const Elem &lhs, const Elem &rhs) const {
    return lhs.second > rhs.second;
  }
};

template <typename Vertex, typename Length>
using Fringe = std::priority_queue<FringeElem<Vertex, Length>,
                                   std::vector<FringeElem<Vertex, Length>>,
                                   FringeComparator<Vertex, Length>>;

template <typename Vertex, typename Length>
using Seen = std::unordered_map<Vertex, Length, boost::hash<Vertex>>;
//===----------------------------------------------------------------------===//
//                      Bidirectional Dijkstra routines
//===----------------------------------------------------------------------===//
/**
 * @brief Initialize a DistanceMap with @c infinite value for each vertex in the
 * graph.
 *
 * @post For each vertex @c v in @p G, <tt>distance[v] = +inf</tt>, given
 *       <tt>inf = std::numeric_limits<distance_type>::max()</tt>
 *
 * @tparam Graph A Boost::Graph.
 * @tparam DistanceMap The distance PropertyMap.
 * @param G The graph.
 * @param distance the DistanceMap to initialize.
 */
template <typename Graph, typename DistanceMap>
void init_distance_vector(Graph &G, DistanceMap distance) {
  using Length = typename boost::property_traits<DistanceMap>::value_type;
  constexpr Length inf = std::numeric_limits<Length>::max();

  for (auto [it, end] = vertices(G); it != end; ++it) {
    distance[*it] = inf;
  }
}

template <typename Vertex, typename PredecessorMap, typename BackPredecessorMap>
void merge_paths(Vertex s, Vertex t, Vertex w, PredecessorMap predecessor_f,
                 BackPredecessorMap predecessor_b,
                 std::deque<Vertex> &final_path) {
  final_path.clear();
  final_path.push_front(w);
  // Fill with forward partial result
  auto cur_f = w;
  while (cur_f != s) {
    auto pred_f = predecessor_f[cur_f];
    assert(pred_f != cur_f); // otherwise cur_f wouldnt be reachable, but if
                             // cur_f wouldnt be reachable we shouldnt be here.
    final_path.push_front(pred_f);
    cur_f = pred_f;
  }

  auto cur_b = w;
  while (cur_b != t) {
    auto pred_b = predecessor_b[cur_b];
    assert(pred_b != cur_b); // otherwise cur_b wouldnt be reachable, but if
                             // cur_b wouldnt be reachable we shouldnt be here.
    final_path.push_back(pred_b);
    cur_b = pred_b;
  }
}

template <typename Vertex, typename Length, typename PredecessorMap,
          typename BackPredecessorMap>
void compare_and_update_shortest_path(Seen<Vertex, Length> &seen_f,
                                      Seen<Vertex, Length> &seen_b,
                                      PredecessorMap predecessor_f,
                                      BackPredecessorMap predecessor_b,
                                      Vertex s, Vertex t, Vertex w,
                                      Length &final_distance,
                                      std::deque<Vertex> &final_path) {
  auto search_w_f = seen_f.find(w);
  auto search_w_b = seen_b.find(w);
  if (search_w_f != std::end(seen_f) && search_w_b != std::end(seen_b)) {
    auto total_distance = search_w_f->second + search_w_b->second;
    if (final_distance > total_distance) {
      final_distance = total_distance;
      // With predecessor map
      merge_paths(s, t, w, predecessor_f, predecessor_b, final_path);
    }
  }
}

template <
    typename Graph, typename PredecessorMap, typename DistanceMap,
    typename WeightMap, typename OtherPredecessorMap, typename OtherDistanceMap,
    typename BiDijkstraVisitorImpl,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Length = typename boost::property_traits<DistanceMap>::value_type>
BiDijkStepRes bi_dijkstra_step(
    const Graph &G, Vertex s, Vertex t, PredecessorMap predecessor,
    DistanceMap distance, WeightMap weight, Fringe<Vertex, Length> &fringe,
    Seen<Vertex, Length> &seen, OtherPredecessorMap other_predecessor,
    OtherDistanceMap other_distance, Fringe<Vertex, Length> &other_fringe,
    Seen<Vertex, Length> &other_seen, Direction direction,
    Length &final_distance, std::deque<Vertex> &final_path,
    boost::BiDijkstraVisitor<BiDijkstraVisitorImpl> &visitor) {
  constexpr Length inf = std::numeric_limits<Length>::max();
  // Extract closest node to expand
  const auto [v, dist] = fringe.top();
  fringe.pop();

  if (distance[v] != inf) {
    // Shortest path to 'node' already found. Continue.
    return BiDijkStepRes::next;
  }

  // Ask the visitor if vertex should be expanded
  auto lower_bound_v = other_fringe.top().second;
  if (!visitor.expand_vertex(v, dist, lower_bound_v, final_distance)) {
    return BiDijkStepRes::next;
  }

  // Update distance
  distance[v] = dist; // Equal to seen[v]
  if (other_distance[v] != inf) {
    // If we have scanned v in both directions we are done,
    // we have now discovered the shortest path. But the visitor may ask to keep
    // on searching.

    // Check terminating condition:
    auto min_dist = fringe.top().second;
    auto other_min_dist = other_fringe.top().second;

    if (visitor.terminating_condition(min_dist, other_min_dist,
                                      final_distance)) {
      return BiDijkStepRes::end;
    }
    // Please refer to:
    // Andreas Paraskevopoulos, Christos Zaroliagis. Improved Alternative Route
    // Planning. Daniele Frigioni and Sebastian Stiller. ATMOS - 13th Workshop
    // on Algorithmic Approaches for Transportation Modelling, Optimizations and
    // Systems - 2013
    // auto terminating_condition = min_dist + other_min_dist > d_s_t;
  }

  // Compute neighbor distances
  for (auto [it, end] = out_edges(v, G); it != end; ++it) {
    auto w = target(*it, G);
    auto min_weight = weight[*it];
    auto vw_length = distance[v] + min_weight;

    if (distance[w] != inf) {
      if (vw_length < distance[v]) {
        // Throw some exception "Contradictory paths found: negative weights?"
        return BiDijkStepRes::negative_weights;
      }
    } else if (auto search_w = seen.find(w);
               search_w == std::end(seen) || vw_length < search_w->second) {
      // Relax v-w edge
      seen.insert_or_assign(w, vw_length);
      fringe.push(std::make_pair(w, vw_length));

      // Build new path to w
      predecessor[w] = v;

      // See if this path is better than the already discovered shortests path
      if (direction == Direction::forward) {
        compare_and_update_shortest_path(seen, other_seen, predecessor,
                                         other_predecessor, s, t, w,
                                         final_distance, final_path);
      } else {
        compare_and_update_shortest_path(other_seen, seen, other_predecessor,
                                         predecessor, s, t, w, final_distance,
                                         final_path);
      }
    }
  }
  // Everything went fine, go with other direction
  return BiDijkStepRes::next;
}

} // namespace kspwlo_impl

#endif