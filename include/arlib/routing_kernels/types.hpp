#ifndef ALTERNATIVE_ROUTING_LIB_TYPES_HPP
#define ALTERNATIVE_ROUTING_LIB_TYPES_HPP

namespace arlib {
enum class shortest_path_algorithm {
  dijkstra = 1,
  astar,
  bidirectional_dijkstra
};
}

#endif // ALTERNATIVE_ROUTING_LIB_TYPES_HPP
