#ifndef ALTERNATIVE_ROUTING_LIB_MULTI_PREDECESSOR_MAP_HPP
#define ALTERNATIVE_ROUTING_LIB_MULTI_PREDECESSOR_MAP_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/property_map/property_map.hpp>

#include <memory>
#include <unordered_map>

namespace arlib {
template <typename Vertex, typename UnorderedAssociativeContainer =
                               std::unordered_map<int, Vertex>>
class multi_predecessor_map {
public:
  using key_type = Vertex;
  using value_type =
      std::remove_cv_t<std::remove_reference_t<UnorderedAssociativeContainer>>;
  using reference = std::add_lvalue_reference_t<value_type>;
  using category = boost::readable_property_map_tag;

  multi_predecessor_map()
      : _pmap{std::make_shared<std::unordered_map<
            Vertex, UnorderedAssociativeContainer, boost::hash<Vertex>>>()} {}
  multi_predecessor_map(multi_predecessor_map const &other)
      : _pmap{other._pmap} {}

  multi_predecessor_map &operator=(multi_predecessor_map const &other) {
    if (this != &other) {
      _pmap = other._pmap;
    }
    return *this;
  }

  template <typename MultiPredecessorMap, typename Key>
  friend typename MultiPredecessorMap::reference get(MultiPredecessorMap &pmap,
                                                     Key const &k);

private:
  std::shared_ptr<std::unordered_map<Vertex, UnorderedAssociativeContainer,
                                     boost::hash<Vertex>>>
      _pmap;
};

template <typename MultiPredecessorMap, typename Key>
typename MultiPredecessorMap::reference get(MultiPredecessorMap &pmap,
                                            Key const &k) {
  return (*pmap._pmap)[k];
}
} // namespace arlib

#endif // ALTERNATIVE_ROUTING_LIB_MULTI_PREDECESSOR_MAP_HPP
