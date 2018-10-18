#ifndef GRAPH_TYPES_H
#define GRAPH_TYPES_H

#include <arlib/multi_predecessor_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <iostream>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace arlib {
using VPair = std::pair<long unsigned int, long unsigned int>;
} // namespace arlib

#endif