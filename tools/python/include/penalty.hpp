/**
 * @file penalty.hpp
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

#ifndef ARLIB_PYTHON_PENALTY_HPP
#define ARLIB_PYTHON_PENALTY_HPP

#include "arlib_utils.hpp"
#include "graph_types.hpp"

<<<<<<< HEAD
#include <boost/graph/graph_traits.hpp>
=======
#include <arlib/graph_utils.hpp>
>>>>>>> 1b28b4d5382de3135408921948e7e20f77952a03
#include <arlib/multi_predecessor_map.hpp>
#include <arlib/penalty.hpp>
#include <arlib/routing_kernels/types.hpp>
#include <arlib/type_traits.hpp>
<<<<<<< HEAD

#include <arlib/graph_utils.hpp>
=======
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
>>>>>>> 1b28b4d5382de3135408921948e7e20f77952a03

#include <exception>
#include <filesystem>
#include <vector>

ARReturnType penalty(std::string_view path_or_graph, long unsigned int source,
                     long unsigned int target, int k, double theta, double p,
                     double r, int max_nb_updates, int max_nb_steps,
                     arlib::routing_kernels kernel, Graphs gtype);

namespace details {
template <typename Graph>
ARReturnType penalty(Graph const &G, arlib::vertex_of_t<Graph> source,
                     arlib::vertex_of_t<Graph> target, int k, double theta,
                     double p, double r, int max_nb_updates, int max_nb_steps,
                     arlib::routing_kernels kernel) {
  using Vertex = arlib::vertex_of_t<Graph>;
  auto predecessors = arlib::multi_predecessor_map<Vertex>{};
  arlib::penalty(G, predecessors, source, target, k, theta, p, r,
                                         max_nb_updates, max_nb_steps, kernel);
  const auto alt_paths = arlib::to_paths(G, predecessors, source, target);

  return to_py_paths(alt_paths, source, target);
}

template <typename Graph>
ARReturnType penalty(std::string_view path_or_graph,
                     arlib::vertex_of_t<Graph> source,
                     arlib::vertex_of_t<Graph> target, int k, double theta,
                     double p, double r, int max_nb_updates, int max_nb_steps,
                     arlib::routing_kernels kernel) {
  // Check if path_or_graph is a file path
  namespace fs = std::filesystem;
  auto path = fs::path{path_or_graph};
  if (fs::is_regular_file(path)) {
    auto G = arlib::read_graph_from_file<Graph>(path_or_graph);
    if (!G) {
      throw std::invalid_argument{"Error while reading graph file."};
    }
    return details::penalty(*G, source, target, k, theta, p, r, max_nb_updates,
                            max_nb_steps, kernel);
  } else {
    auto G = arlib::read_graph_from_string<Graph>(std::string{path_or_graph});
    return details::penalty(G, source, target, k, theta, p, r, max_nb_updates,
                            max_nb_steps, kernel);
  }
}
} // namespace details

#endif