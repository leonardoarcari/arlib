/**
 * @file type_traits.hpp
 * @author Leonardo Arcari (leonardo1.arcari@gmail.com)
 * @version 1.0.0
 * @date 2018-10-28
 *
 * @copyright Copyright (c) 2018 Leonardo Arcari
 *
 * MIT Licence
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef ALTERNATIVE_ROUTING_LIB_TYPE_TRAITS_HPP
#define ALTERNATIVE_ROUTING_LIB_TYPE_TRAITS_HPP

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
/**
 * Helper type to query a Graph `vertex descriptor` type.
 *
 * @tparam Graph The graph type.
 */
template <typename Graph>
using vertex_of_t = typename boost::graph_traits<Graph>::vertex_descriptor;

/**
 * Helper type to query a Graph `edge descriptor` type.
 *
 * @tparam Graph The graph type.
 */
template <typename Graph>
using edge_of_t = typename boost::graph_traits<Graph>::edge_descriptor;

/**
 * Helper type to query a `Property Map`'s `key` type.
 *
 * @tparam Map The property map type.
 */
template <typename Map>
using key_of_t = typename boost::property_traits<Map>::key_type;

/**
 * Helper type to query a `Property Map`'s `value` type.
 *
 * @tparam Map The property map type.
 */
template <typename Map>
using value_of_t = typename boost::property_traits<Map>::value_type;

/**
 * Helper type to query a Graph `edge_weight` value type.
 *
 * For instance, given a Graph type:
 * ```cpp
 * using Graph = boost::adjacency_list<boost::vecS, boost::vecS,
                                       boost::bidirectionalS,
                                       boost::no_property,
                                       boost::property<
                                         boost::edge_weight_t, int>>;
 * ```
 * The following expression is `true`
 * ```cpp
 * std::is_same_v<length_of_t<Graph>, int>
 * ```
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 */
template <typename Graph>
using length_of_t =
    typename boost::property_traits<typename boost::property_map<
        Graph, boost::edge_weight_t>::type>::value_type;
} // namespace arlib

#endif // ALTERNATIVE_ROUTING_LIB_TYPE_TRAITS_HPP
