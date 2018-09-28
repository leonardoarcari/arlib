#ifndef GRAPH_TYPES_H
#define GRAPH_TYPES_H

#include <boost/graph/properties.hpp>

#include <iostream>
#include <utility>

namespace arlib {
using VPair = std::pair<int, int>;

template <typename Graph,
          typename Length =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
class Path {
public:
  Path(const Graph &g, Length length) : graph_{g}, length_{length} {};
  Path(Graph &&g, Length length) : graph_{}, length_{length} {
    // Swap data from graph to move from (g) to this->graph
    this->graph_.swap(g);
  }
  Path(Path const &other) = default;
  Path(Path &&other) noexcept : graph_{}, length_{other.length_} {
    this->graph_.swap(other.graph_);
  }

  Path &operator=(Path const &other) = default;
  Path &operator=(Path &&other) noexcept {
    if (this != &other) {
      using std::swap;
      graph_.swap(other.graph_);
      swap(length_, other.length_);
    }
    return *this;
  }

  void swap(Path &other) {
    using std::swap;
    graph_.swap(other.graph_);
    swap(length_, other.length_);
  }

  Graph const &graph() const { return graph_; }
  Graph &graph() { return graph_; }

  Length length() const { return length_; }

private:
  Graph graph_ = {};
  Length length_ = {};
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