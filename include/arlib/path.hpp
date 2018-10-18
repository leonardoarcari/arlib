#ifndef ALTERNATIVE_ROUTING_LIB_PATH_HPP
#define ALTERNATIVE_ROUTING_LIB_PATH_HPP

#include <arlib/details/path_impl.hpp>
#include <arlib/type_traits.hpp>

#include <boost/graph/filtered_graph.hpp>

namespace arlib {
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
