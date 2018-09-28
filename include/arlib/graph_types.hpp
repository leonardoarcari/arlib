#ifndef GRAPH_TYPES_H
#define GRAPH_TYPES_H

#include <boost/graph/adjacency_list.hpp>
#include <iostream>
#include <utility>

namespace arlib {
using Graph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                          boost::no_property,
                          boost::property<boost::edge_weight_t, double>>;
using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
using VPair = std::pair<int, int>;
using Length = typename boost::property_traits<typename boost::property_map<
    Graph, boost::edge_weight_t>::type>::value_type;

template <typename Graph,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
class Path {
public:
  Path(const Graph &g, length_type length) : graph_{g}, length_{length} {};
  Path(Graph &&g, length_type length) : graph_{}, length_{length} {
    // Swap data from graph to move from (g) to this->graph
    this->graph_.swap(g);
  }
  Path(Path const &other) = default;
  Path(Path &&other) : graph_{}, length_{other.length_} {
    this->graph_.swap(other.graph_);
  }

  Path &operator=(Path const &other) = default;
  Path &operator=(Path &&other) {
    using std::swap;
    graph_.swap(other.graph_);
    swap(length_, other.length_);
    return *this;
  }

  void swap(Path &other) {
    using std::swap;
    graph_.swap(other.graph_);
    swap(length_, other.length_);
  }

  Graph const &graph() const { return graph_; }
  Graph &graph() { return graph_; }

  length_type length() const { return length_; }

private:
  Graph graph_ = {};
  length_type length_ = {};
};

template <typename Graph> void swap(Path<Graph> &p1, Path<Graph> &p2) {
  p1.swap(p2);
}

enum class shortest_path_algorithm {
  dijkstra = 1,
  astar,
  bidirectional_dijkstra
};
} // namespace arlib

#endif