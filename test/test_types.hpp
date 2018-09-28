#ifndef ALTERNATIVE_ROUTING_LIB_TEST_TYPES_HPP
#define ALTERNATIVE_ROUTING_LIB_TEST_TYPES_HPP

#include <boost/graph/adjacency_list.hpp>

namespace arlib {
namespace test {
using Graph =
boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                      boost::no_property,
                      boost::property<boost::edge_weight_t, double>>;
using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
using Length = typename boost::property_traits<typename boost::property_map<
    Graph, boost::edge_weight_t>::type>::value_type;
}
} // namespace arlib

#endif // ALTERNATIVE_ROUTING_LIB_TEST_TYPES_HPP
