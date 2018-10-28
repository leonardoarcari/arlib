/**
 * @file visitor.hpp
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

#ifndef ARLIB_VISITOR_HPP
#define ARLIB_VISITOR_HPP

#include <boost/graph/graph_concepts.hpp>

#include <limits>
#include <unordered_set>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
//===----------------------------------------------------------------------===//
//                    Bidirectional Dijkstra visitor
//===----------------------------------------------------------------------===//
/**
 * CRTP interface class for bidirectional_dijkstra() visitor.
 *
 * A BiDijkstraVisitor is passed to bidirectional_dijkstra() in order to
 * customize the algorithm behavior with respect to
 *  - terminating_condition(): when a vertex is found both from source and
 * target searches, bidirectional_dijkstra() asks the visitor whether to end the
 *    search or keep going.
 *  - expand_vertex(): given a vertex v, its distance from one root and the
 *    distance-lower-bound from the other and the current minimum st-distance,
 *    the visitor should return whether to expand the search in the direction of
 *    v or not.
 *
 * @tparam Derived The derived class implementing BiDijkstraVisitor interface
 */
template <typename Derived> class BiDijkstraVisitor {
public:
  /**
   * Whether or not bidirectional_dijkstra() should stop. It is invoked when a
   * vertex v is found both from source search and target search.
   *
   * @tparam Length The weight value type.
   * @param min_distance The minimum distance to any node in current direction
   *        fringe.
   * @param other_min_distance The minimum distance to any node in the other
   *        direction fringe.
   * @param st_distance Current minimum source-target distance.
   * @return true if bidirectional_dijkstra() should stop.
   * @return false otherwise.
   */
  template <typename Length>
  bool terminating_condition(Length min_distance, Length other_min_distance,
                             Length st_distance) {
    return static_cast<Derived *>(this)->terminating_condition(
        min_distance, other_min_distance, st_distance);
  }
  /**
   * Whether or not to bidirectional_dijkstra() should expand the search through
   * the input Vertex v.
   *
   * @tparam Vertex The vertex_descriptor.
   * @tparam Length The weight value type.
   * @param v The candidate Vertex to expand.
   * @param v_distance The Vertex distance according to current-direction
   *        search.
   * @param lower_bound_v The Vertex distance-heuristic according to
   *        opposite-direction search.
   * @param st_distance Current minimum source-target distance.
   * @return true if bidirectional_dijkstra() should expand vertex @p v
   * @return false otherwise.
   */
  template <typename Vertex, typename Length>
  bool expand_vertex(Vertex v, Length v_distance, Length lower_bound_v,
                     Length st_distance) {
    return static_cast<Derived *>(this)->expand_vertex(
        v, v_distance, lower_bound_v, st_distance);
  }
};

/**
 * BiDijkstraVisitor implementing vanilla Bidirectional Dijkstra behavior as
 * described in Nicholson's paper (1966):
 * https://academic.oup.com/comjnl/article/9/3/275/406281
 */
class IdentityBiDijkstraVisitor
    : public BiDijkstraVisitor<IdentityBiDijkstraVisitor> {
public:
  /**
   * Vanilla Bidirectional Dijkstra stopping condition.
   *
   * @tparam Length The weight value type.
   * @param min_distance The minimum distance to any node in current direction
   *        fringe.
   * @param other_min_distance The minimum distance to any node in the other
   *        direction fringe.
   * @param st_distance Current minimum source-target distance.
   * @return true if @p min_distance + @p other_min_distance > @p st_distance.
   * @return false otherwise.
   */
  template <typename Length>
  bool terminating_condition(Length min_distance, Length other_min_distance,
                             Length st_distance) {
    return (min_distance + other_min_distance) > st_distance;
  }
  /**
   * Always-expand policy.
   *
   * @tparam Vertex The vertex_descriptor
   * @tparam Length The weight value type.
   * @return true always.
   */
  template <typename Vertex, typename Length>
  bool expand_vertex(Vertex, Length, Length, Length) {
    return true;
  }
};

/**
 * BiDijkstraVisitor implementing Uninformed Bidirectional Pruner as described
 * in
 *
 * A. Paraskevopoulos, C. Zaroliagis, Improved alternative route
 * planning,198in: OASIcs-OpenAccess Series in Informatics, Vol. 33, Schloss
 * Dagstuhl-199Leibniz-Zentrum fuer Informatik, 2013
 * http://drops.dagstuhl.de/opus/volltexte/2013/4248/
 *
 * The purpose of this visitor is to explore the whole graph an keep track of
 * all those vertices that should be pruned.
 * Let v be a Vertex, let tau be the pruning factor, v_distance be the distance
 * of v from the root of current search, other_min_distance be
 * distance-heuristic of v from the opposite search-root and st_distance be the
 * current minimum distance between source and target nodes. Then v should be
 * pruned if:
 * v_distance + lower_bound_v > tau * st_distance
 *
 * @see uninformed_bidirectional_pruner()
 *
 * @tparam Vertex The vertex_descriptor
 */
template <typename Vertex>
class UninformedBiPrunerVisitor
    : public BiDijkstraVisitor<UninformedBiPrunerVisitor<Vertex>> {
public:
  /**
   * A set of Vertex of the graph
   */
  using VertexSet = std::unordered_set<Vertex, boost::hash<Vertex>>;
  /**
   * The VertexSet const-iterator type
   */
  using const_iterator = typename VertexSet::const_iterator;
  /**
   * Construct a new UninformedBiPrunerVisitor object with a pruning factor of
   * @p tau.
   *
   * @param tau The pruning factor.
   */
  explicit UninformedBiPrunerVisitor(double tau)
      : tau{tau}, is_pruning_phase{false} {}
  /**
   * Always keep searching.
   *
   * @tparam Length The weight value type.
   * @param min_distance The minimum distance to any node in current direction
   *        fringe.
   * @param other_min_distance The minimum distance to any node in the other
   *        direction fringe.
   * @param st_distance Current minimum source-target distance.
   * @return false always.
   */
  template <typename Length>
  bool terminating_condition(Length min_distance, Length other_min_distance,
                             Length st_distance) {
    if (min_distance + other_min_distance > st_distance) {
      is_pruning_phase = true;
    }
    return false;
  }
  /**
   * If the shortest-path from source to target hasn't been found yet, then
   * always expand. Otherwise, expand only if
   * @p v_distance + @p lower_bound_v <= tau * @p st_distance
   *
   * @tparam Length The weight value type.
   * @param v The candidate Vertex to expand.
   * @param v_distance The Vertex distance according to current-direction
   *        search.
   * @param lower_bound_v The Vertex distance-heuristic according to
   *        opposite-direction search.
   * @param st_distance Current minimum source-target distance.
   * @return true If the shortest-path from source to target hasn't been found
   * yet or if @p v_distance + @p lower_bound_v <= tau * @p st_distance
   * @return false otherwise.
   */
  template <typename Length>
  bool expand_vertex(Vertex v, Length v_distance, Length lower_bound_v,
                     Length st_distance) {
    if (is_pruning_phase) {
      if (v_distance + lower_bound_v <= tau * st_distance) {
        return true;
      } else {
        pruned_vertices.insert(v);
        return false;
      }
    } else
      return true;
  }
  /**
   * @return a pair of const-iterators to begin and end of the pruned VertexSet.
   */
  std::pair<const_iterator, const_iterator> get_pruned_vertices() const {
    return {pruned_vertices.cbegin(), pruned_vertices.cend()};
  }
  /**
   * @param v The vertex to test
   * @return true if @p v should be pruned.
   * @return false otherwise.
   */
  bool is_pruned(const Vertex &v) const {
    return (pruned_vertices.find(v) != std::end(pruned_vertices));
  }

private:
  double tau;
  bool is_pruning_phase;
  std::unordered_set<Vertex, boost::hash<Vertex>> pruned_vertices;
};
} // namespace arlib
#endif