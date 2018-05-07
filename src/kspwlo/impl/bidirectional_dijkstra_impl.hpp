#ifndef KSPWLO_BIDIRECTIONAL_DIJKSTRA_IMPL_HPP
#define KSPWLO_BIDIRECTIONAL_DIJKSTRA_IMPL_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/property_map.hpp>

#include <iostream>
#include <limits>
#include <queue>
#include <vector>

namespace kspwlo_impl {
template <typename Edge, typename Length>
using WeightMap = std::unordered_map<Edge, Length, boost::hash<Edge>>;

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

enum class Direction { forward = 1, backward };
constexpr Direction switch_direction(Direction prev_dir) {
  return (prev_dir == Direction::backward) ? Direction::forward
                                           : Direction::backward;
}

enum class BiDijkStepRes { next = 1, end, negative_weights };

//===----------------------------------------------------------------------===//
//                      Penalty algorithm classes
//===----------------------------------------------------------------------===//

template <typename Graph, typename DistanceMap, typename Vertex>
void init_distance_vector(Graph &G, DistanceMap distance, Vertex s) {
  using Length = typename boost::property_traits<DistanceMap>::value_type;
  constexpr Length inf = std::numeric_limits<Length>::max();

  for (auto[it, end] = vertices(G); it != end; ++it) {
    distance[*it] = inf;
  }
}

template <typename Vertex>
void merge_predecessors(Vertex w, Vertex t, PathMap<Vertex> &paths,
                        PathMap<Vertex> &paths_b,
                        std::vector<Vertex> &final_path) {
  // auto v = w;
  // while (v != t) {
  //   auto u = predecessor_b[v];
  //   predecessor[u] = v;
  //   v = u;
  // }
  auto &path_w = paths[w];
  auto &path_w_b = paths_b[w];
  final_path.clear();
  final_path.insert(final_path.end(), path_w.cbegin(), path_w.cend());

  auto w_t_it = std::next(path_w_b.crbegin());
  final_path.insert(final_path.end(), w_t_it, path_w_b.crend());
}

template <
    typename Graph, typename DistanceMap, typename WeightMap,
    typename VertexIndexMap, typename OtherDistanceMap, typename OtherIndexMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Length = typename boost::property_traits<DistanceMap>::value_type>
BiDijkStepRes
bi_dijkstra_step(const Graph &G, Vertex t, PathMap<Vertex> &paths,
                 DistanceMap distance, WeightMap weight,
                 VertexIndexMap index_map, Fringe<Vertex, Length> &fringe,
                 Seen<Vertex, Length> &seen, PathMap<Vertex> &other_paths,
                 OtherDistanceMap other_distance, OtherIndexMap other_index_map,
                 Fringe<Vertex, Length> &other_fringe,
                 Seen<Vertex, Length> &other_seen, Direction direction,
                 Length &final_distance, std::vector<Vertex> &final_path) {
  constexpr Length inf = std::numeric_limits<Length>::max();
  // Extract closest node to expand
  const auto[v, dist] = fringe.top();
  fringe.pop();

  if (distance[v] != inf) {
    // Shortest path to 'node' already found. Continue.
    return BiDijkStepRes::next;
  }

  // Update distance
  distance[v] = dist; // Equal to seen[index_map[v]]
  if (other_distance[v] != inf) {
    // If we have scanned v in both directions we are done,
    // we have now discovered the shortest path

    // Check terminating condition:
    auto min_dist = fringe.top().second;
    auto other_min_dist = other_fringe.top().second;

    Length d_s_t = inf;
    if (direction == Direction::forward)
      d_s_t = final_distance;
    else
      d_s_t = final_distance;

    auto condition = min_dist + other_min_dist > d_s_t;

    std::cout << "Terminating condition: " << min_dist + other_min_dist << " > "
              << d_s_t << ": " << std::boolalpha << condition << "\n";

    if (condition) {
      return BiDijkStepRes::end;
    } else {
      return BiDijkStepRes::next;
    }
  }

  // Compute neighbor distances
  for (auto[it, end] = out_edges(v, G); it != end; ++it) {
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
      auto new_path = std::vector<Vertex>(paths[v]);
      new_path.push_back(w);
      paths.insert_or_assign(w, std::move(new_path));

      // See if this path is better than the already discovered shortests path
      if (direction == Direction::forward) {
        auto &seen_f = seen;
        auto &paths_f = paths;
        auto &seen_b = other_seen;
        auto &paths_b = other_paths;

        auto search_w_f = seen_f.find(w);
        auto search_w_b = seen_b.find(w);
        if (search_w_f != std::end(seen_f) && search_w_b != std::end(seen_b)) {
          auto total_distance = search_w_f->second + search_w_b->second;
          if (final_distance > total_distance) {
            final_distance = total_distance;
            merge_predecessors(w, t, paths_f, paths_b, final_path);
          }
        }
      } else {
        auto seen_f = other_seen;
        auto paths_f = other_paths;
        auto seen_b = seen;
        auto paths_b = paths;

        auto search_w_f = seen_f.find(w);
        auto search_w_b = seen_b.find(w);
        if (search_w_f != std::end(seen_f) && search_w_b != std::end(seen_b)) {
          auto total_distance = search_w_f->second + search_w_b->second;
          if (final_distance > total_distance) {
            final_distance = total_distance;
            merge_predecessors(w, t, paths_f, paths_b, final_path);
          }
        }
      }
    }
  }

  return BiDijkStepRes::next;
}

} // namespace kspwlo_impl

#endif