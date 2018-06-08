#ifndef GRAPH_TYPES_H
#define GRAPH_TYPES_H

#include <boost/graph/adjacency_list.hpp>
#include <iostream>
#include <utility>

namespace kspwlo {
using Graph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                          boost::no_property,
                          boost::property<boost::edge_weight_t, double>>;
using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
using Edge = std::pair<int, int>;
using Length = typename boost::property_traits<typename boost::property_map<
    kspwlo::Graph, boost::edge_weight_t>::type>::value_type;

template <typename Graph,
          typename length_type =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
struct Path {
  Path(const Graph &g, length_type length) : graph{g}, length{length} {};
  Path(Graph &&g, length_type length) : graph{}, length{length} {
    // Swap data from graph to move from (g) to this->graph
    this->graph.swap(g);
  }
  Graph graph;
  length_type length;
};

enum class shortest_path_algorithm {
  dijkstra = 1,
  astar,
  bidirectional_dijkstra
};
} // namespace kspwlo

#endif