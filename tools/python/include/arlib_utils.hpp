/**
 * @file arlib_utils.hpp
 * @author Leonardo Arcari (leonardo1.arcari@gmail.com)
 * @version 0.0.1
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

#ifndef ARLIB_PYTHON_ARLIB_UTILS_HPP
#define ARLIB_PYTHON_ARLIB_UTILS_HPP

#include "graph_types.hpp"

#include <arlib/path.hpp>
#include <arlib/type_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <exception>
#include <vector>

namespace details {
template <typename Graph>
ARReturnType to_py_paths(std::vector<arlib::Path<Graph>> const &alt_paths,
                         arlib::vertex_of_t<Graph> source,
                         arlib::vertex_of_t<Graph> target) {
  using Vertex = arlib::vertex_of_t<Graph>;
  auto res = ARReturnType{};
  for (auto const &p : alt_paths) {
    auto py_path = std::vector<Vertex>{source};
    auto u = source;
    while (u != target) {
      auto [first, last] = arlib::out_edges(u, p);
      if (first == last) {
        throw std::runtime_error{"An error occured while building a path. "
                                 "Couldn't find target vertex."};
      }
      auto v = arlib::target(*first, p);
      py_path.push_back(v);
      u = v;
    }
    res.push_back(std::move(py_path));
  }
  return res;
}
}

#endif