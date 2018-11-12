/**
 * @file ubp_impl.hpp
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

#ifndef ALTERNATIVE_ROUTING_LIB_UBP_IMPL_HPP
#define ALTERNATIVE_ROUTING_LIB_UBP_IMPL_HPP

#include <limits>

namespace arlib {
namespace details {
template <typename Edge> struct pruned_edges {
public:
  pruned_edges() = default;
  pruned_edges(pruned_edges const &) = default;
  pruned_edges(pruned_edges &&) noexcept = default;

  explicit pruned_edges(
      std::unordered_set<Edge, boost::hash<Edge>> const &edges)
      : _edges{std::make_shared<std::unordered_set<Edge, boost::hash<Edge>>>(
            edges)} {}
  explicit pruned_edges(std::unordered_set<Edge, boost::hash<Edge>> &&edges)
      : _edges{std::make_shared<std::unordered_set<Edge, boost::hash<Edge>>>(
            std::move(edges))} {}

  pruned_edges &operator=(pruned_edges const &other) {
    if (&other != this) {
      _edges = other._edges;
    }
    return *this;
  }

  pruned_edges &operator=(pruned_edges &&other) noexcept {
    if (&other != this) {
      _edges = std::move(other._edges);
    }
    return *this;
  }

  bool operator()(const Edge &e) const {
    return _edges->find(e) == _edges->end();
  }

private:
  std::shared_ptr<std::unordered_set<Edge, boost::hash<Edge>>> _edges = {};
};

template <typename Vertex, typename FPredecessorMap, typename BPredecessorMap,
          typename FDistanceMap, typename BDistanceMap, typename Length>
bool pruning_policy(Vertex s, Vertex t, Vertex v, FPredecessorMap predecessor_f,
                    BPredecessorMap predecessor_b, FDistanceMap distance_f,
                    BDistanceMap distance_b, double tau,
                    Length final_distance) {
  // Source and Target vertices must not be pruned.
  if (v != s && v != t) {
    // If v is not in any shortest path from forward nor backward, prune it.
    bool found_v_f = (predecessor_f[v] != v);
    bool found_v_b = (predecessor_b[v] != v);
    Length inf = std::numeric_limits<Length>::max();
    if (distance_f[v] == inf && !found_v_b) {
      return true;
    }

    if (distance_b[v] == inf && !found_v_f) {
      return true;
    }

    if (!found_v_f && !found_v_b) {
      // If a node was not added to SP-Trees but we met it from both
      // directions we must keep it!
      if (distance_f[v] == inf && distance_b[v] == inf) {
        return true;
      }
    }

    // If distance_s_v + distance_t_v > tau * final_distance, prun it
    if (distance_f[v] != inf && distance_b[v] != inf) {
      if (distance_f[v] + distance_b[v] > tau * final_distance) {
        return true;
      }
    }
  }
  return false;
}
} // namespace details
} // namespace arlib

#endif // ALTERNATIVE_ROUTING_LIB_UBP_IMPL_HPP
