#ifndef BOOST_ONEPASS_PLUS_HPP
#define BOOST_ONEPASS_PLUS_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>

#include "graph_types.hpp"

#include <memory>
#include <optional>
#include <queue>
#include <unordered_set>
#include <vector>

namespace kspwlo {
template <typename Vertex, typename size_type> class OnePassLabel {
public:
  OnePassLabel(Vertex node, size_type length, size_type lower_bound,
               std::shared_ptr<OnePassLabel> &previous, int checked_at_step)
      : node{node}, length{length}, lower_bound{lower_bound},
        previous{previous}, similarity_map{}, checked_at_step{checked_at_step} {
  }
  OnePassLabel(Vertex node, size_type length, size_type lower_bound,
               int checked_at_step)
      : node{node}, length{length}, lower_bound{lower_bound}, previous{},
        similarity_map{}, checked_at_step{checked_at_step} {}

  template <typename VertexListGraph> VertexListGraph getPath() {
    auto edges = std::vector<Edge>{};
    auto nodes = std::unordered_set<Vertex>{};

    auto prev = previous.lock();
    while (prev) {
      auto u = prev->node;
      auto v = node;
      edges.emplace_back(u, v);
      nodes.insert(u);
      nodes.insert(v);
      prev = prev->previous.lock();
    }

    return VertexListGraph(std::begin(edges), std::end(edges), nodes.size());
  }

private:
  Vertex node;
  size_type length;
  size_type lower_bound;
  std::weak_ptr<OnePassLabel> previous;
  std::vector<double> similarity_map;
  int checked_at_step;
};

template <typename Vertex, typename size_type> struct OnePassPlusASComparator {
  using LabelPtr = std::shared_ptr<kspwlo::OnePassLabel<Vertex, size_type>>;

  bool operator()(LabelPtr lhs, LabelPtr rhs) {
    return lhs->lower_bound > rhs->lower_bound;
  }
};

template <
    typename size_type, typename Graph,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
std::vector<size_type> distance_from_target(Graph &G, Vertex target) {
  // Reverse graph
  auto G_rev = boost::make_reverse_graph(G);
  auto distance = std::vector<size_type>(boost::num_vertices(G_rev));

  // Run dijkstra_shortest_paths and return distance vector
  boost::dijkstra_shortest_paths(G_rev, target,
                                 boost::distance_map(&distance[0]));
  return distance;
}

} // namespace kspwlo

namespace boost {
template <typename Graph, typename PredecessorMap, typename Vertex>
Graph build_graph_from_dijkstra(Graph& G, const PredecessorMap& p, Vertex source, Vertex target) {
    auto G_weight = get(edge_weight, G);
    auto path = Graph{};
    auto path_weight = get(edge_weight, G);

    auto current = target;
    while (current != source) {
        auto u = p[current];
        auto w = G_weight[edge(u, current, G).first];
        add_edge(u, current, path);
        path_weight[edge(u, current, path).first] = w;

        current = u;
    }

    return path;
}

template <typename PropertyGraph, typename Vertex = typename graph_traits<
                                      PropertyGraph>::vertex_descriptor>
std::vector<PropertyGraph> onepass_plus(PropertyGraph &G, Vertex source,
                                        Vertex target, int k, double theta) {
  // P_LO set of k paths
  auto resPaths = std::vector<PropertyGraph>{};

  // Min-priority queue
  auto weight = get(edge_weight, G);
  using size_type = typename property_traits<decltype(weight)>::value_type;
  using LabelPtr = std::shared_ptr<kspwlo::OnePassLabel<Vertex, size_type>>;
  auto Q =
      std::priority_queue<LabelPtr, std::vector<LabelPtr>,
                          kspwlo::OnePassPlusASComparator<Vertex, size_type>>{};

  // Compute lower bounds for AStar
  auto lower_bounds = kspwlo::distance_from_target<size_type>(G, target);

  // Compute shortest path from s to t
  auto sp_distances = std::vector<size_type>(num_vertices(G));
  auto predecessor = std::vector<Vertex>(num_vertices(G), source);
  auto vertex_id = get(vertex_index, G);
  dijkstra_shortest_paths(G, source,
                          distance_map(&sp_distances[0])
                              .predecessor_map(make_iterator_property_map(
                                  std::begin(predecessor), vertex_id, source)));
  auto sp = build_graph_from_dijkstra(G, predecessor, source, target);
  

  return resPaths;
}
} // namespace boost

#endif