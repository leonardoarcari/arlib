#ifndef BOOST_KSPWLO_VISITOR_HPP
#define BOOST_KSPWLO_VISITOR_HPP

#include <boost/graph/graph_concepts.hpp>
#include <limits>
#include <unordered_set>

/**
 * @brief Algorithms and utilities for Boost::Graph
 */
namespace boost {
//===----------------------------------------------------------------------===//
//                    Bidirectional Dijkstra visitor
//===----------------------------------------------------------------------===//
template <typename Derived> class BiDijkstraVisitor {
public:
  template <typename Length>
  bool terminating_condition(Length min_distance, Length other_min_distance,
                             Length st_distance) {
    return static_cast<Derived *>(this)->terminating_condition(
        min_distance, other_min_distance, st_distance);
  }

  template <typename Vertex, typename Length>
  bool expand_vertex(Vertex v, Length v_distance, Length lower_bound_v,
                     Length st_distance) {
    return static_cast<Derived *>(this)->expand_vertex(
        v, v_distance, lower_bound_v, st_distance);
  }
};

class IdentityBiDijkstraVisitor
    : public BiDijkstraVisitor<IdentityBiDijkstraVisitor> {
public:
  template <typename Length>
  bool terminating_condition(Length min_distance, Length other_min_distance,
                             Length st_distance) {
    return (min_distance + other_min_distance) > st_distance;
  }

  template <typename Vertex, typename Length>
  bool expand_vertex(Vertex, Length, Length, Length) {
    return true;
  }
};

template <typename Vertex>
class UninformedBiPrunerVisitor
    : public BiDijkstraVisitor<UninformedBiPrunerVisitor<Vertex>> {
public:
  using VertexSet = std::unordered_set<Vertex, boost::hash<Vertex>>;
  using const_iterator = typename VertexSet::const_iterator;
  explicit UninformedBiPrunerVisitor(double tau)
      : tau{tau}, is_pruning_phase{false} {}

  template <typename Length>
  bool terminating_condition(Length min_distance, Length other_min_distance,
                             Length st_distance) {
    if (min_distance + other_min_distance > st_distance) {
      is_pruning_phase = true;
    }
    return false;
  }

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

  std::pair<const_iterator, const_iterator> get_pruned_vertices() {
    return {pruned_vertices.cbegin(), pruned_vertices.cend()};
  }

private:
  double tau;
  bool is_pruning_phase;
  std::unordered_set<Vertex, boost::hash<Vertex>> pruned_vertices;
};
} // namespace boost
#endif