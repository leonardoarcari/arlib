/**
 * @file path_impl.hpp
 * @author Leonardo Arcari (leonardo1.arcari@gmail.com)
 * @version 1.0.0
 * @date 2018-10-28
 *
 * @copyright Copyright (c) 2018 Leonardo Arcari
 *
 * MIT License
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

#ifndef ALTERNATIVE_ROUTING_LIB_PATH_IMPL_HPP
#define ALTERNATIVE_ROUTING_LIB_PATH_IMPL_HPP

#include <boost/graph/graph_concepts.hpp>

#include <memory>
#include <unordered_set>

namespace arlib {
namespace details {
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
} // namespace details
} // namespace arlib

#endif // ALTERNATIVE_ROUTING_LIB_PATH_IMPL_HPP
