#ifndef GRAPH_TYPES_H
#define GRAPH_TYPES_H

#include <boost/graph/adjacency_list.hpp>
#include <utility>

namespace kspwlo {
using Graph = boost::adjacency_list<boost::vecS, boost::vecS,
                                    boost::bidirectionalS, boost::no_property,
                                    boost::property<boost::edge_weight_t, int>>;
using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
using Edge = std::pair<int, int>;
}

#endif