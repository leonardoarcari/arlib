#ifndef ALTERNATIVE_ROUTING_LIB_UBP_IMPL_HPP
#define ALTERNATIVE_ROUTING_LIB_UBP_IMPL_HPP

#include <limits>

namespace arlib {
namespace details {
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
