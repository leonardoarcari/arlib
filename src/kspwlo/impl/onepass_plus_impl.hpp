#ifndef ONEPASS_PLUS_IMPL_HPP
#define ONEPASS_PLUS_IMPL_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/exception.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/topological_sort.hpp>

#include <boost/graph/reverse_graph.hpp>

#include "kspwlo/graph_types.hpp"

#include <cassert>
#include <iostream>
#include <memory>
#include <queue>
#include <unordered_set>
#include <vector>

/**
 * @brief Implementations details of kSPwLO algorithms
 */
namespace kspwlo_impl {

//===----------------------------------------------------------------------===//
//                    OnePass+ algorithm support classes
//===----------------------------------------------------------------------===//

/**
 * @brief A label for a node in the graph to keep track of its exploration
 *        state.
 *
 * A label tracks the path from the source to the node @c n it's attached, the
 * similarity of the path <tt>p(s -> n)</tt> wrt to the alternative paths
 * computed so far and the time step the similarities were updated (i.e. the
 * number of alternative paths against which the similarities are currently
 * computed)
 *
 * Labels can either be @c head labels if they are attached to source node, or
 * have a predecessor label, i.e. they are attached to a node @c n such that
 * there exist a label attached to a node @c n' and an edge <tt>(n', n)</tt> in
 * Graph.
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam Vertex A vertex of Graph.
 * @tparam Length The value type of an edge weight of Graph.
 */
template <typename Graph,
          typename Length =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type,
          typename Vertex =
              typename boost::graph_traits<Graph>::vertex_descriptor>
class OnePassLabel {
private:
  Vertex node;
  Length length;
  Length lower_bound;
  OnePassLabel *previous;
  std::vector<double> similarity_map;
  int k;
  int checked_at_step;

public:
  /**
   * @brief size_type trait of similarity vector
   */
  using similarity_map_size_type = typename decltype(similarity_map)::size_type;

  /**
   * @brief iterator trait of similarity vector
   */
  using similarity_map_iterator_type =
      typename decltype(similarity_map)::iterator;

  /**
   * @brief Length
   */
  using length_type = Length;

  /**
   * @brief Vertex
   */
  using vertex_type = Vertex;

  /**
   * @brief Construct a new OnePass Label object with a predecessor label.
   *
   * @param node Node to attach this label to.
   * @param length Distance of this node from source following the path from
   *        this label to the source's one.
   * @param lower_bound AStar heuristic of the distance of @p node from target.
   * @param previous Predecessor label.
   * @param k Number of k alternative paths to compute.
   * @param checked_at_step The current time step (i.e. the number of
   *        alternative paths currently computed)
   */
  OnePassLabel(Vertex node, length_type length, length_type lower_bound,
               OnePassLabel *previous, int k, int checked_at_step)
      : node{node}, length{length},
        lower_bound{lower_bound}, previous{previous},
        similarity_map(k, 0), k{k}, checked_at_step{checked_at_step} {}

  /**
   * @brief Construct a new OnePass Label object with no predecessor (i.e. a @c
   * head label)
   *
   * @param node Node to attach this label to.
   * @param length Distance of this node from source following the path from
   *        this label to the source's one.
   * @param lower_bound AStar heuristic of the distance of @p node from target.
   * @param k Number of k alternative paths to compute.
   * @param checked_at_step The current time step (i.e. the number of
   *        alternative paths currently computed)
   */
  OnePassLabel(Vertex node, length_type length, length_type lower_bound, int k,
               int checked_at_step)
      : node{node}, length{length}, lower_bound{lower_bound}, previous{nullptr},
        similarity_map(k, 0), k{k}, checked_at_step{checked_at_step} {}

  std::vector<kspwlo::Edge> get_path() {
    auto edge_set = std::vector<kspwlo::Edge>{};

    auto v = node;
    auto prev = previous;
    while (prev != nullptr) {
      auto u = prev->node;
      edge_set.emplace_back(u, v);

      // Shift label pointer back
      v = u;
      prev = prev->previous;
    }

    return edge_set;
  }

  bool is_path_acyclic(Vertex begin) {
    bool is_acyclic = true;
    auto current = this;
    while (current != nullptr) {
      if (current->node == begin) {
        is_acyclic = false;
        break;
      }
      current = current->previous;
    }
    return is_acyclic;
  }

  /**
   * @param kth The kth alternative path index.
   * @return A reference to the similarity of path <tt>p(source, n)</tt> wrt to
   *         @p kth alternative path.
   */
  double &get_similarity_with(int kth) { return similarity_map.at(kth); }

  /**
   * @param kth The kth alternative path index.
   * @return A const-reference to the similarity of path <tt>p(source, n)</tt>
   *         wrt to @p kth alternative path.
   */
  const double &get_similarity_with(int kth) const {
    return similarity_map.at(kth);
  }

  /**
   * @return the number of alternative paths for which there exists a similarity
   *         measure for, for this label.
   */
  similarity_map_size_type get_num_paths() const {
    return similarity_map.size();
  }

  /**
   * @return a copy of the similarity vector wrt the alternative paths.
   */
  std::vector<double> get_similarity_map() const {
    return std::vector<double>(std::begin(similarity_map),
                               std::end(similarity_map));
  }

  /**
   * @brief Copies the similarity values from [@p first, @p last) iterators into
   *        label's similarity vector.
   *
   * @param first begin iterator.
   * @param last past-to-end iterator.
   */
  void set_similarities(similarity_map_iterator_type first,
                        similarity_map_iterator_type last) {
    std::copy(first, last, std::begin(similarity_map));
  }

  /**
   * @return the node this label is attached to.
   */
  Vertex get_node() const { return node; }

  /**
   * @return the distance of this node from source following the path from
   *         this label to the source's one.
   */
  length_type get_length() const { return length; }

  /**
   * @return AStar heuristic of the distance of the node from target.
   */
  length_type get_lower_bound() const { return lower_bound; }

  /**
   * @return Number of k alternative paths to compute.
   */
  int num_paths_k() const { return k; }

  /**
   * @return the time step the similarities were updated (i.e. the
   *         number of alternative paths against which the similarities are
   *         currently computed).
   */
  int last_check() const { return checked_at_step; }

  /**
   * @param currentStep the current time step.
   * @return true if @c last_check() < @p currentStep.
   * @return false otherwise.
   */
  bool is_outdated(int currentStep) const {
    return checked_at_step < currentStep;
  }

  /**
   * @brief Set the time step of similarities update.
   *
   * @param step the time step.
   */
  void set_last_check(int step) {
    assert(step > 0);
    checked_at_step = step;
  }

private:
  template <typename Graph2>
  friend std::ostream &operator<<(std::ostream &os,
                                  const OnePassLabel<Graph2> &label) {
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

/**
 * @brief A conventient container for labels dominance checking.
 *
 * When a new label @c l' for a node @c n is created we must check it against
 * Lemma 2: Let @c P_LO the set of computed alternative paths so far. If all the
 * labels already existing for node @c n have a similarity with @c P_LO that is
 * less than the similarity of @c l' with @c P_LO, then @c l' is dominated and
 * should be pruned.
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @tparam Vertex A vertex of Graph.
 */
template <typename Graph,
          typename Vertex =
              typename boost::graph_traits<Graph>::vertex_descriptor>
class SkylineContainer {
public:
  using Label = OnePassLabel<Graph>;
  using LabelPtr = Label *;

  /**
   * @brief Inserts a label into the skyline. Before insertion, you should check
   * if the skyline already dominates @p label. See dominates().
   *
   * @param label A label to add to the skyline.
   */
  void insert(Label *label) {
    auto node_n = label->get_node();
    // If node_n is new
    if (!contains(node_n)) {
      // Initialize the labels for node n with 'label'
      auto labels_for_n = std::vector<LabelPtr>{label};
      container.insert(std::make_pair(node_n, labels_for_n));
    } else {
      // If not, just add the label to node_n's list of labels
      container[node_n].push_back(label);
    }
  }

  /**
   * @param node A node of the graph.
   * @return true if the skyline contains @p node.
   * @return false otherwise.
   */
  bool contains(Vertex node) {
    return container.find(node) != std::end(container);
  }

  /**
   * @brief Check if the skyline dominates @p label. Refer to SkylineContainer
   * description for a more clear explanation.
   *
   * @param label A label to check if it's dominated by the skyline.
   * @return true if the skyline dominates @p label.
   * @return false otherwise.
   */
  bool dominates(const Label &label) {

    auto node_n = label.get_node();
    // If node_n is not in the skyline, then we have no labels to check
    if (!contains(node_n)) {
      return false;
    }

    // For each of the paths p' we have similarity measure in 'label' we
    // check if label.sim(p') is less than at least one of the labels in the
    // skyline. If so, the skyline does NOT dominates 'label'.
    for (auto label_p : container[node_n]) {
      // if (auto tmpLabel = label_p.lock()) {
      LabelPtr tmpLabel = label_p;
      bool skyline_dominates_label = true;

      for (int i = 0; i < static_cast<int>(label.get_num_paths()); ++i) {
        if (label.get_similarity_with(i) < tmpLabel->get_similarity_with(i)) {
          skyline_dominates_label = false;
          break;
        }
      }

      if (skyline_dominates_label) {
        return true;
      }
      //}
    }

    return false;
  }

  /**
   * @return the number of labels stored in the skyline.
   */
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

/**
 * @brief A Comparator functor to compare two labels in an A* fashion.
 *
 * A lower bound for the distance of the label from the target is used to decide
 * the ordering. The label with the lowest lower bound is the smallest.
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 */
template <typename Graph> struct OnePassPlusASComparator {
  using LabelPtr = OnePassLabel<Graph> *;

  bool operator()(LabelPtr lhs, LabelPtr rhs) {
    return lhs->get_lower_bound() > rhs->get_lower_bound();
  }
};

//===----------------------------------------------------------------------===//
//                      OnePass+ algorithm routines
//===----------------------------------------------------------------------===//

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
kspwlo::Path<Graph> build_path_from_dijkstra(Graph &G, const PredecessorMap &p,
                                             Vertex s, Vertex t) {
  length_type length = 0;
  auto weight = boost::get(boost::edge_weight, G);
  auto edge_list = std::vector<kspwlo::Edge>{};

  auto current = t;
  while (current != s) {
    auto u = p[current];
    edge_list.emplace_back(u, current);

    auto edge_in_G = boost::edge(u, current, G).first;

    length += weight[edge_in_G];
    current = u;
  }

  return {build_graph_from_edges(edge_list, G), length};
}

template <
    typename Graph,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Length =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
kspwlo::Path<Graph> compute_shortest_path(Graph &G, Vertex s, Vertex t) {
  using namespace boost;
  auto sp_distances = std::vector<Length>(num_vertices(G));
  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);
  dijkstra_shortest_paths(G, s,
                          distance_map(&sp_distances[0])
                              .predecessor_map(make_iterator_property_map(
                                  std::begin(predecessor), vertex_id, s)));
  return build_path_from_dijkstra(G, predecessor, s, t);
}

template <typename Graph, typename EdgeMap,
          typename resPathIndex = typename EdgeMap::mapped_type::size_type>
void update_res_edges(Graph &candidate, Graph &graph, EdgeMap &resEdges,
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

template <typename Graph, typename EdgeMap,
          typename resPathIndex = typename EdgeMap::mapped_type::size_type>
void update_res_edges(const std::vector<kspwlo::Edge> &candidate, Graph &graph,
                      EdgeMap &resEdges, resPathIndex paths_count) {
  using mapped_type = typename EdgeMap::mapped_type;
  for (auto &e : candidate) {
    auto edge_in_g = edge(e.first, e.second, graph).first;

    auto search = resEdges.find(edge_in_g);
    if (search != std::end(resEdges)) { // edge found
      search->second.push_back(paths_count - 1);
    } else { // new edge, add it to resEdges
      resEdges.insert(std::make_pair(edge_in_g, mapped_type{paths_count - 1}));
    }
  }
}

template <typename Label, typename Graph, typename EdgesMap, typename PathsMap,
          typename WeightMap>
bool update_label_similarity(Label &label, Graph &G, const EdgesMap &resEdges,
                             const PathsMap &resPaths, WeightMap &weight,
                             double theta, int step) {
  using namespace boost;
  bool below_sim_threshold = true;
  auto tmpPath = label.get_path();
  for (auto &e : tmpPath) {
    auto edge_in_g = edge(e.first, e.second, G).first;
    auto search = resEdges.find(edge_in_g);
    // if tmpPath share an edge with any k-th shortest path, update the
    // overlapping factor
    if (search != std::end(resEdges)) {
      for (auto index : search->second) {
        if (static_cast<int>(index) > label.last_check() &&
            static_cast<int>(index) < step) {
          label.get_similarity_with(index) += weight[edge_in_g];

          // Check Lemma 1. The similarity between the candidate path and
          // all the other k-shortest-paths must be less then theta
          if (label.get_similarity_with(index) / resPaths[step].length >
              theta) {
            below_sim_threshold = false;
            break;
          }
        }
      }
    }
  }
  return below_sim_threshold;
}

template <typename Label, typename Vertex = typename Label::Vertex,
          typename length_type = typename Label::length_type>
std::unique_ptr<Label> expand_path(Label *label, Vertex node,
                                   length_type node_lower_bound,
                                   length_type edge_weight, int step) {
  auto tmpLength = label->get_length() + edge_weight;
  auto tmpLowerBound = tmpLength + node_lower_bound;
  return std::make_unique<Label>(node, tmpLength, tmpLowerBound, label,
                                 label->num_paths_k(), step);
}

template <typename EdgesMap, typename PathsList, typename WeightMap,
          typename Edge = typename EdgesMap::key_type>
bool is_below_sim_threshold(const Edge &c_edge,
                            std::vector<double> &similarity_map, double theta,
                            const EdgesMap &resEdges, const PathsList &resPaths,
                            const WeightMap &weight) {
  auto search = resEdges.find(c_edge);
  if (search != std::end(resEdges)) {
    auto &res_paths_with_c_edge = search->second;
    for (auto index : res_paths_with_c_edge) {
      similarity_map[index] += weight[c_edge];
      auto similarity = similarity_map[index] / resPaths[index].length;
      if (similarity > theta) {
        return false;
      }
    }
  }
  return true;
}

} // namespace kspwlo_impl

#endif