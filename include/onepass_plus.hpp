#ifndef BOOST_ONEPASS_PLUS_HPP
#define BOOST_ONEPASS_PLUS_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/exception.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/topological_sort.hpp>

#include "graph_types.hpp"

#include <cassert>
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
private:
  Vertex node;
  length_type length;
  length_type lower_bound;
  std::weak_ptr<OnePassLabel> previous;
  std::vector<double> similarity_map;
  int k;
  int checked_at_step;

public:
  using similarity_map_size_type = typename decltype(similarity_map)::size_type;
  using similarity_map_iterator_type =
      typename decltype(similarity_map)::iterator;

  OnePassLabel(Vertex node, length_type length, length_type lower_bound,
               std::shared_ptr<OnePassLabel> &previous, int k,
               int checked_at_step)
      : node{node}, length{length},
        lower_bound{lower_bound}, previous{previous},
        similarity_map(k, 0), k{k}, checked_at_step{checked_at_step} {}
  OnePassLabel(Vertex node, length_type length, length_type lower_bound, int k,
               int checked_at_step)
      : node{node}, length{length}, lower_bound{lower_bound}, previous{},
        similarity_map(k, 0), k{k}, checked_at_step{checked_at_step} {}

  Graph getPath() {
    auto edge_set = std::vector<Edge>{};
    auto nodes = std::unordered_set<Vertex>{};

    bool source_found = false;
    auto v = node;
    auto prev = previous.lock();
    while (prev) {
      auto u = prev->node;
      edge_set.emplace_back(u, v);
      nodes.insert(u);
      nodes.insert(v);

      // Shift label pointer back
      v = u;
      prev = prev->previous.lock();
    }

    auto g = Graph(std::begin(edge_set), std::end(edge_set), nodes.size());
    std::cout << "getGraph(): ";
    for (auto it = edges(g).first; it != edges(g).second; ++it) {
      std::cout << "(" << source(*it, g) << ", " << target(*it, g) << ") ";
    }
    std::cout << "\n";
    return g;
  }

  double &getSimilarityWith(int kth) { return similarity_map.at(kth); }
  const double &getSimilarityWith(int kth) const {
    return similarity_map.at(kth);
  }

  similarity_map_size_type getNumPaths() const { return similarity_map.size(); }
  std::vector<double> getSimilarityMap() const {
    return std::vector<double>(std::begin(similarity_map),
                               std::end(similarity_map));
  }
  void importSimilarities(similarity_map_iterator_type first,
                          similarity_map_iterator_type last) {
    std::copy(first, last, std::begin(similarity_map));
  }

  Vertex getNode() const { return node; }
  length_type getLength() const { return length; }
  length_type getLowerBound() const { return lower_bound; }
  int numPathsK() const { return k; }
  int checkedAtStep() const { return checked_at_step; }
  void setCheckedAtStep(int step) {
    assert(step > 0);
    checked_at_step = step;
  }

private:
  template <typename Graph2,
            typename length_type2 =
                typename boost::property_traits<typename boost::property_map<
                    Graph2, boost::edge_weight_t>::type>::value_type,
            typename Vertex2 =
                typename boost::graph_traits<Graph2>::vertex_descriptor>
  friend std::ostream &
  operator<<(std::ostream &os,
             const OnePassLabel<Graph2, length_type2, Vertex2> &label) {
    os << "(node = " << label.node << ", length = " << label.length
       << ", lower_bound = " << label.lower_bound << ", k = " << label.k
       << ", checked_at_step = " << label.checked_at_step
       << ", similarities = [ ";
    for (auto sim : label.similarity_map) {
      os << sim << " ";
    }

    os << "])";
    return os;
  }
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

  void insert(std::shared_ptr<Label> &label) {
    auto node_n = label->getNode();
    // If node_n is new
    if (!contains(node_n)) {
      // Initialize the labels for node n with 'label'
      auto labels_for_n = std::vector<LabelPtr>{LabelPtr{label}};
      container.insert(std::make_pair(node_n, labels_for_n));
    } else {
      // If not, just add the label to node_n's list of labels
      container[node_n].push_back(LabelPtr{label});
    }
  }

  bool contains(Vertex node) {
    return container.find(node) != std::end(container);
  }

  bool dominates(const Label &label) {
    auto node_n = label.getNode();
    // If node_n is not in the skyline, then we have no labels to check
    if (!contains(node_n)) {
      return false;
    }

    // For each of the paths p' we have similarity measure in 'label' we
    // check if label.sim(p') is less than at least one of the labels in the
    // skyline. If so, the skyline does NOT dominates 'label'.
    for (auto label_p : container[node_n]) {
      if (auto tmpLabel = label_p.lock()) {
        bool skyline_dominates_label = true;

        for (int i = 0; i < static_cast<int>(label.getNumPaths()); ++i) {
          if (label.getSimilarityWith(i) < tmpLabel->getSimilarityWith(i)) {
            skyline_dominates_label = false;
            break;
          }
        }

        if (skyline_dominates_label) {
          return true;
        }
      }
    }

    return false;
  }

  int num_labels() {
    int nb_labels = 0;

    for (auto it = std::begin(container); it != std::end(container); ++it) {
      nb_labels += it->second.size();
    }
  }

private:
  std::unordered_map<Vertex, std::vector<LabelPtr>, boost::hash<Vertex>>
      container;
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
std::vector<length_type> distance_from_target(Graph &G, Vertex t) {
  // Reverse graph
  auto G_rev = boost::make_reverse_graph(G);
  auto distance = std::vector<length_type>(boost::num_vertices(G_rev));

  // Run dijkstra_shortest_paths and return distance vector
  boost::dijkstra_shortest_paths(G_rev, t, boost::distance_map(&distance[0]));
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
build_path_from_dijkstra(Graph &G, const PredecessorMap &p, Vertex s,
                         Vertex t) {
  auto G_weight = get(edge_weight, G);
  auto path = Graph{};
  auto path_weight = get(edge_weight, path);
  length_type length = 0;

  auto current = t;
  while (current != s) {
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
void track_res_edges(Graph &candidate, Graph &graph, EdgeMap &resEdges,
                     resPathIndex paths_count) {
  using mapped_type = typename EdgeMap::mapped_type;
  for (auto ei = edges(candidate).first; ei != edges(candidate).second; ++ei) {
    auto edge_in_g =
        edge(source(*ei, candidate), target(*ei, candidate), graph).first;

    auto search = resEdges.find(edge_in_g);
    if (search != std::end(resEdges)) { // edge found
      search->second.push_back(paths_count - 1);
    } else { // new edge, add it to resEdges
      resEdges.insert(std::make_pair(edge_in_g, mapped_type{paths_count - 1}));
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
  using Label = kspwlo::OnePassLabel<PropertyGraph>;
  using LabelPtr = std::shared_ptr<Label>;
  auto Q =
      std::priority_queue<LabelPtr, std::vector<LabelPtr>,
                          kspwlo::OnePassPlusASComparator<PropertyGraph>>{};
  auto created_labels = std::vector<LabelPtr>{};

  // Skyline for dominance checkind (Lemma 2)
  auto skyline = kspwlo::SkylineContainer<PropertyGraph>{};

  // Compute lower bounds for AStar
  auto lower_bounds = kspwlo::distance_from_target(G, t);

  // Compute shortest path from s to t
  auto sp_distances = std::vector<length_type>(num_vertices(G));
  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);
  dijkstra_shortest_paths(G, s,
                          distance_map(&sp_distances[0])
                              .predecessor_map(make_iterator_property_map(
                                  std::begin(predecessor), vertex_id, s)));
  auto sp_path = build_path_from_dijkstra(G, predecessor, s, t);
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
    if (static_cast<int>(label->getNode()) == t) {
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