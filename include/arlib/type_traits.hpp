#ifndef ALTERNATIVE_ROUTING_LIB_TYPE_TRAITS_HPP
#define ALTERNATIVE_ROUTING_LIB_TYPE_TRAITS_HPP

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

namespace arlib {
template <typename Graph>
using vertex_of_t = typename boost::graph_traits<Graph>::vertex_descriptor;

template <typename Graph>
using edge_of_t = typename boost::graph_traits<Graph>::edge_descriptor;

template <typename Map>
using key_of_t = typename boost::property_traits<Map>::key_type;

template <typename Map>
using value_of_t = typename boost::property_traits<Map>::value_type;

template <typename Graph>
using length_of_t =
    typename boost::property_traits<typename boost::property_map<
        Graph, boost::edge_weight_t>::type>::value_type;
} // namespace arlib

#endif // ALTERNATIVE_ROUTING_LIB_TYPE_TRAITS_HPP
