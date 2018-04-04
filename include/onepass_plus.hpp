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
template <typename Graph,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type,
          typename Vertex =
              typename boost::graph_traits<Graph>::vertex_descriptor>
class OnePassLabel {
public:
  OnePassLabel(Vertex node, length_type length, length_type lower_bound,
               std::shared_ptr<OnePassLabel> &previous, int k,
               int checked_at_step)
      : node{node}, length{length},
        lower_bound{lower_bound}, previous{previous},
        similarity_map(k, 0), checked_at_step{checked_at_step} {}
  OnePassLabel(Vertex node, length_type length, length_type lower_bound, int k,
               int checked_at_step)
      : node{node}, length{length}, lower_bound{lower_bound}, previous{},
        similarity_map(k, 0), checked_at_step{checked_at_step} {}

  Graph getPath() {
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

    return Graph(std::begin(edges), std::end(edges), nodes.size());
  }

  double &getSimilarityWith(int kth) { return similarity_map[kth]; }

  Vertex getNode() { return node; }
  length_type getLength() { return length; }
  length_type getLowerBound() { return lower_bound; }
  int checkedAtStep() { return checked_at_step; }
  void setCheckedAtStep(int step) { checked_at_step = step; }

private:
  Vertex node;
  length_type length;
  length_type lower_bound;
  std::weak_ptr<OnePassLabel> previous;
  std::vector<double> similarity_map;
  int checked_at_step;
};

template <typename Graph,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type,
          typename Vertex =
              typename boost::graph_traits<Graph>::vertex_descriptor>
class SkylineContainer {
public:
  using Label = OnePassLabel<Graph>;
  using LabelPtr = std::weak_ptr<Label>;

  void insert(std::shared_ptr<Label>& label);
  bool contains(Vertex node);
  bool dominates(const Label& label);
  int nb_labels();
private:
  std::unordered_map<Vertex, std::vector<LabelPtr>, boost::hash<Vertex>> container;
};

template <typename Graph,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
struct OnePassPlusASComparator {
  using LabelPtr = std::shared_ptr<kspwlo::OnePassLabel<Graph>>;

  bool operator()(LabelPtr lhs, LabelPtr rhs) {
    return lhs->getLowerBound() > rhs->getLowerBound();
  }
};

template <
    typename Graph,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename length_type =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
std::vector<length_type> distance_from_target(Graph &G, Vertex target) {
  // Reverse graph
  auto G_rev = boost::make_reverse_graph(G);
  auto distance = std::vector<length_type>(boost::num_vertices(G_rev));

  // Run dijkstra_shortest_paths and return distance vector
  boost::dijkstra_shortest_paths(G_rev, target,
                                 boost::distance_map(&distance[0]));
  return distance;
}

template <typename Graph,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
struct Path {
  Path(Graph graph, length_type length) : graph(graph), length{length} {}; 
  Graph graph;
  length_type length;
};

} // namespace kspwlo

namespace boost {
template <typename Graph, typename PredecessorMap, typename Vertex,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
kspwlo::Path<Graph, length_type>
build_path_from_dijkstra(Graph &G, const PredecessorMap &p, Vertex source,
                         Vertex target) {
  auto G_weight = get(edge_weight, G);
  auto path = Graph{};
  auto path_weight = get(edge_weight, path);
  length_type length = 0;

  auto current = target;
  while (current != source) {
    auto u = p[current];
    auto w = G_weight[edge(u, current, G).first];
    add_edge(u, current, path);
    path_weight[edge(u, current, path).first] = w;

    length += w;
    current = u;
  }

  return {path, length};
}

template <typename Graph, typename EdgeMap,
          typename resPathIndex = typename EdgeMap::mapped_type::size_type>
void track_res_edges(Graph &candidate, EdgeMap &resEdges,
                     resPathIndex paths_count) {
  using mapped_type = typename EdgeMap::mapped_type;
  for (auto ei = edges(candidate).first; ei != edges(candidate).second; ++ei) {
    auto search = resEdges.find(*ei);
    if (search != std::end(resEdges)) { // edge found
      search->second.push_back(paths_count - 1);
    } else { // new edge, add it to resEdges
      resEdges.insert(std::make_pair(*ei, mapped_type{paths_count - 1}));
    }
  }
}

template <
    typename PropertyGraph,
    typename Vertex = typename graph_traits<PropertyGraph>::vertex_descriptor,
    typename length_type =
        typename boost::property_traits<typename boost::property_map<
            PropertyGraph, boost::edge_weight_t>::type>::value_type>
std::vector<kspwlo::Path<PropertyGraph>>
onepass_plus(PropertyGraph &G, Vertex source, Vertex target, int k,
             double theta) {
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
  using Label = kspwlo::OnePassLabel<PropertyGraph>;
  using LabelPtr = std::shared_ptr<Label>;
  auto Q =
      std::priority_queue<LabelPtr, std::vector<LabelPtr>,
                          kspwlo::OnePassPlusASComparator<PropertyGraph>>{};

  // Compute lower bounds for AStar
  auto lower_bounds = kspwlo::distance_from_target(G, target);

  // Compute shortest path from s to t
  auto sp_distances = std::vector<length_type>(num_vertices(G));
  auto predecessor = std::vector<Vertex>(num_vertices(G), source);
  auto vertex_id = get(vertex_index, G);
  dijkstra_shortest_paths(G, source,
                          distance_map(&sp_distances[0])
                              .predecessor_map(make_iterator_property_map(
                                  std::begin(predecessor), vertex_id, source)));
  auto sp_path = build_path_from_dijkstra(G, predecessor, source, target);
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
  track_res_edges(sp, resEdges, paths_count);

  // Initialize min-priority queue Q with <s, empty_set>
  Q.push(std::make_shared<Label>(source, 0, lower_bounds[source], k,
                                 paths_count - 1));

  // While Q is not empty
  while (!Q.empty()) {
    // Current path
    auto label = Q.top();
    Q.pop();

    // Perform lazy update of the similairty vector of 'label', since new paths
    // might have been added to P_LO from the time this 'label' was pushed into
    // priority queue.
    if (label->checkedAtStep() < static_cast<int>(paths_count - 1)) {
      bool below_sim_threshold = true;
      auto tmpPath = label->getPath();
      for (auto ei = edges(tmpPath).first; ei != edges(tmpPath).second; ++ei) {
        auto search = resEdges.find(*ei);
        // if tmpPath share an edge with any k-th shortest path, update the
        // overlapping factor
        if (search != std::end(resEdges)) {
          for (auto index : search->second) {
            if (static_cast<int>(index) > label->checkedAtStep() &&
                index <= paths_count) {
              label->getSimilarityWith(index) += weight[*ei];

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
      if (!below_sim_threshold) {
        continue; // Skip candidate path
      }
    }

    // If we found the target node
    if (static_cast<int>(label->getNode()) == target) {
      // Build the new k-th shortest path
      auto resPath = kspwlo::Path(label->getPath(), label->getLength());
      auto &tmpPath = resPath.graph;
      resPaths.push_back(resPath);
      ++paths_count;

      if (static_cast<int>(paths_count) == k) { // we found k paths. End.
        break;
      }

      // For each edge in the candidate path see if it's already in any of the
      // P_LO paths. If not, add it to the resEdges. If so, keep track of which
      // path includes it
      track_res_edges(tmpPath, resEdges, paths_count);
    } else { // Expand Search

    }
  }

  return resPaths;
}
} // namespace boost

#endif