/**
 * @file multi_predecessor_map.hpp
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

#ifndef ALTERNATIVE_ROUTING_LIB_MULTI_PREDECESSOR_MAP_HPP
#define ALTERNATIVE_ROUTING_LIB_MULTI_PREDECESSOR_MAP_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/property_map/property_map.hpp>

#include <memory>
#include <unordered_map>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
/**
 * The multi predecessor map records the edges in the alternative paths from a
 * pair of *source-target* vertices. Upon completion of any *alternative
 * routing* algorithm, for any vertex `v` on any alternative path `p`,
 * multi_predecessor_map stores the predecessor of node `v` on path `p`. If no
 * predecessor for a node `v'` is reported, then `v'` is not part of any
 * alternative path or it is the source node. multi_predecessor_map is a model
 * of `Read Property Map`. The `key` type of a multi_predecessor_map is the
 * `vertex_descriptor` of some `Graph`. Whereas the `value` type satisfies the
 * `UnorderedAssociativeContainer` concept, where **its** key is an `int`, which
 * is the index/number of alternative path for which it exists a predecessor of
 * `v`, and where the value type is a `vertex descriptor` of some `Graph`.
 *
 * @see onepass_plus(), esx() and penalty() if you are looking for *alternative
 *      routing* algorithms.
 *
 * @tparam Vertex The `vertex_descriptor`.
 * @tparam UnorderedAssociativeContainer A model of
 *         `UnorderedAssociativeContainer` concept. Defaulted to
 *         `std::unordered_map<int, Vertex>`
 */
template <typename Vertex, typename UnorderedAssociativeContainer =
                               std::unordered_map<int, Vertex>>
class multi_predecessor_map {
public:
  /**
   * The `key` type.
   */
  using key_type = Vertex;
  /**
   * The `value` type.
   */
  using value_type =
      std::remove_cv_t<std::remove_reference_t<UnorderedAssociativeContainer>>;
  /**
   * Reference to value_type.
   */
  using reference = std::add_lvalue_reference_t<value_type>;
  /**
   * The category of this Property Map.
   */
  using category = boost::readable_property_map_tag;
  /**
   * Default-construct a new multi_predecessor_map object
   */
  multi_predecessor_map()
      : _pmap{std::make_shared<std::unordered_map<
            Vertex, UnorderedAssociativeContainer, boost::hash<Vertex>>>()} {}
  /**
   * Copy-construct a new multi_predecessor_map object
   *
   * Copy operations on multi_predecessor_map are always **shallow** because
   * Property Map requires cheap copy operation. That means that multiple copies
   * always refer to the **same memory location**.
   *
   * @param other The multi_predecessor_map to copy from.
   */
  multi_predecessor_map(multi_predecessor_map const &other)
      : _pmap{other._pmap} {}
  /**
   * Assign another multi_predecessor_map to `this`.
   *
   * Copy operations on multi_predecessor_map are always **shallow** because
   * Property Map requires cheap copy operation. That means that multiple copies
   * always refer to the **same memory location**.
   *
   * @param other The multi_predecessor_map to copy from.
   * @return `this`.
   */
  multi_predecessor_map &operator=(multi_predecessor_map const &other) {
    if (this != &other) {
      _pmap = other._pmap;
    }
    return *this;
  }

  template <typename Vertex2, typename Key>
  friend typename multi_predecessor_map<Vertex2>::reference
  get(multi_predecessor_map<Vertex2> &pmap, Key const &k);

private:
  std::shared_ptr<std::unordered_map<Vertex, UnorderedAssociativeContainer,
                                     boost::hash<Vertex>>>
      _pmap;
};

/**
 * Lookup the value associated with `k`
 *
 * @tparam MultiPredecessorMap The multi predecessor map records the edges in
 *         the alternative paths from @p s to @p t. Upon completion of the
 *         algorithm, for any vertex `v` on any alternative path `p`, multi
 *         predecessor map stores the predecessor of node `v` on path `p`. If no
 *         predecessor for a node `v'` is reported, then `v'` is not part of any
 *         alternative path or it is the source node @p s. The type of
 *         `MultiPredecessorMap` must be a model of `Read Property Map`. The
 *         vertex descriptor of the input graph @p G must be usable as a key
 *         type for the multi predecessor map. Whereas the value type must
 *         satisfy the `UnorderedAssociativeContainer` concept, where its key is
 *         an `int`, the index/number of alternative path for which it exists a
 *         predecessor of `v`, and where the value type is a vertex descriptor
 *         of input graph @p G.
 * @tparam Key Must be a valid key type for `MultiPredecessorMap`.
 * @param pmap The `MultiPredecessorMap` to get value from.
 * @param k The key.
 * @return A reference to the value associated with `k`.
 */
template <typename Vertex2, typename Key>
typename multi_predecessor_map<Vertex2>::reference
get(multi_predecessor_map<Vertex2> &pmap, Key const &k) {
  return (*pmap._pmap)[k];
}
} // namespace arlib

#endif // ALTERNATIVE_ROUTING_LIB_MULTI_PREDECESSOR_MAP_HPP
