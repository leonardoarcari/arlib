#ifndef GRAPH_TYPES_H
#define GRAPH_TYPES_H

#include <arlib/multi_predecessor_map.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>

#include <iostream>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace arlib {
using VPair = std::pair<long unsigned int, long unsigned int>;

template <typename G, typename = int>
struct has_edge_weight : std::false_type {};

template <typename G>
struct has_edge_weight<
    G, decltype((void)boost::property_traits<typename boost::property_map<
                    G, boost::edge_weight_t>::type>::value_type,
                0)> : std::true_type {};

template <typename G>
constexpr auto has_edge_weight_v = has_edge_weight<G>::value;

template <typename Vertex> struct alternative_path_vertices {
public:
  alternative_path_vertices() = default;
  alternative_path_vertices(alternative_path_vertices const &) = default;
  alternative_path_vertices(alternative_path_vertices &&) noexcept = default;

  explicit alternative_path_vertices(
      std::unordered_set<Vertex, boost::hash<Vertex>> const &vertices)
      : _vertices{
            std::make_shared<std::unordered_set<Vertex, boost::hash<Vertex>>>(
                vertices)} {}
  explicit alternative_path_vertices(
      std::unordered_set<Vertex, boost::hash<Vertex>> &&vertices)
      : _vertices{
            std::make_shared<std::unordered_set<Vertex, boost::hash<Vertex>>>(
                std::move(vertices))} {}

  alternative_path_vertices &operator=(alternative_path_vertices const &other) {
    if (&other != this) {
      _vertices = other._vertices;
    }
    return *this;
  }

  alternative_path_vertices &
  operator=(alternative_path_vertices &&other) noexcept {
    if (&other != this) {
      _vertices = std::move(other._vertices);
    }
    return *this;
  }

  bool operator()(const Vertex &v) const {
    return _vertices->find(v) != _vertices->end();
  }

private:
  std::shared_ptr<std::unordered_set<Vertex, boost::hash<Vertex>>> _vertices =
      {};
};

template <typename Edge> struct alternative_path_edges {
public:
  alternative_path_edges() = default;
  alternative_path_edges(alternative_path_edges const &) = default;
  alternative_path_edges(alternative_path_edges &&) noexcept = default;

  explicit alternative_path_edges(
      std::unordered_set<Edge, boost::hash<Edge>> const &edges)
      : _edges{std::make_shared<std::unordered_set<Edge, boost::hash<Edge>>>(
            edges)} {}
  explicit alternative_path_edges(
      std::unordered_set<Edge, boost::hash<Edge>> &&edges)
      : _edges{std::make_shared<std::unordered_set<Edge, boost::hash<Edge>>>(
            std::move(edges))} {}

  alternative_path_edges &operator=(alternative_path_edges const &other) {
    if (&other != this) {
      _edges = other._edges;
    }
    return *this;
  }

  alternative_path_edges &operator=(alternative_path_edges &&other) noexcept {
    if (&other != this) {
      _edges = std::move(other._edges);
    }
    return *this;
  }

  bool operator()(const Edge &e) const {
    return _edges->find(e) != _edges->end();
  }

private:
  std::shared_ptr<std::unordered_set<Edge, boost::hash<Edge>>> _edges = {};
};

template <typename Graph,
          typename Vertex =
              typename boost::graph_traits<Graph>::vertex_descriptor,
          typename Edge = typename boost::graph_traits<Graph>::edge_descriptor,
          typename Length =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
class Path {
public:
  using FilteredGraph =
      boost::filtered_graph<Graph, alternative_path_edges<Edge>,
                            alternative_path_vertices<Vertex>>;

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

enum class shortest_path_algorithm {
  dijkstra = 1,
  astar,
  bidirectional_dijkstra
};

template <typename Graph> void swap(Path<Graph> &v1, Path<Graph> &v2) {
  v1.swap(v2);
}

} // namespace arlib

#endif