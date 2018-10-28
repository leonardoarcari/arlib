/**
 * @file path.hpp
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

#ifndef ALTERNATIVE_ROUTING_LIB_PATH_HPP
#define ALTERNATIVE_ROUTING_LIB_PATH_HPP

#include <arlib/details/path_impl.hpp>
#include <arlib/type_traits.hpp>

#include <boost/graph/filtered_graph.hpp>

/**
 * An Alternative-Routing library for Boost.Graph
 */
namespace arlib {
/**
 * A simple-path
 * 
 * @tparam Graph The graph type
 */
template <typename Graph, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>,
          typename Length = length_of_t<Graph>>
class Path {
public:
  using FilteredGraph =
      boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                            details::alternative_path_vertices<Vertex>>;

  Path(std::shared_ptr<FilteredGraph> g, Length length)
      : graph_{std::move(g)}, length_{length} {};
  Path(Path const &other) = default;
  Path(Path &&other) noexcept = default;

  Path &operator=(Path const &other) = default;
  Path &operator=(Path &&other) noexcept = default;

  void swap(Path &other) {
    using std::swap;
    swap(graph_, other.graph_);
    swap(length_, length_);
  }

  FilteredGraph const &graph() const { return *graph_; }
  FilteredGraph &graph() { return *graph_; }

  Length length() const { return length_; }

private:
  std::shared_ptr<FilteredGraph> graph_ = {};
  Length length_ = {};
};

template <typename Graph> void swap(Path<Graph> &v1, Path<Graph> &v2) {
  v1.swap(v2);
}

template <typename Graph, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
std::shared_ptr<
    boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                          details::alternative_path_vertices<Vertex>>>
make_path_filtered_graph(
    Graph const &G, std::unordered_set<Edge, boost::hash<Edge>> const &edges,
    std::unordered_set<Vertex, boost::hash<Vertex>> const &vertices) {
  using FilteredGraph =
      boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                            details::alternative_path_vertices<Vertex>>;
  auto ap_edges = details::alternative_path_edges{edges};
  auto ap_vertices = details::alternative_path_vertices{vertices};
  return std::make_shared<FilteredGraph>(G, ap_edges, ap_vertices);
}

template <typename Graph, typename Vertex = vertex_of_t<Graph>,
          typename Edge = edge_of_t<Graph>>
std::shared_ptr<
    boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                          details::alternative_path_vertices<Vertex>>>
make_path_filtered_graph(
    Graph const &G, std::unordered_set<Edge, boost::hash<Edge>> &&edges,
    std::unordered_set<Vertex, boost::hash<Vertex>> &&vertices) {
  using FilteredGraph =
      boost::filtered_graph<Graph, details::alternative_path_edges<Edge>,
                            details::alternative_path_vertices<Vertex>>;
  auto ap_edges = details::alternative_path_edges{std::move(edges)};
  auto ap_vertices = details::alternative_path_vertices{std::move(vertices)};
  return std::make_shared<FilteredGraph>(G, ap_edges, ap_vertices);
}
} // namespace arlib

#endif // ALTERNATIVE_ROUTING_LIB_PATH_HPP
