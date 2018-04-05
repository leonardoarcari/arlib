#ifndef ONEPASS_PLUS_IMPL_HPP
#define ONEPASS_PLUS_IMPL_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include "graph_types.hpp"

#include <cassert>
#include <memory>
#include <unordered_set>
#include <vector>

namespace kspwlo_impl {
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
    auto edge_set = std::vector<kspwlo::Edge>{};
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

    return nb_labels;
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
  using LabelPtr = std::shared_ptr<OnePassLabel<Graph>>;

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

template <typename Graph, typename PredecessorMap, typename Vertex,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
kspwlo::Path<Graph, length_type>
build_path_from_dijkstra(Graph &G, const PredecessorMap &p, Vertex s,
                         Vertex t) {
  using namespace boost;
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
} // namespace kspwlo_impl

#endif