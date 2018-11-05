/**
 * @file arlib.cpp
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

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "esx.hpp"
#include "graph_types.hpp"
#include "onepass_plus.hpp"
#include "penalty.hpp"

PYBIND11_MODULE(arlib, m) {
  py::enum_<Graphs>(m, "Graphs")
      .value("AdjListInt", Graphs::AdjListInt)
      .value("AdjListFloat", Graphs::AdjListFloat);
  py::enum_<arlib::routing_kernels>(m, "RoutingKernels")
      .value("dijkstra", arlib::routing_kernels::dijkstra)
      .value("astar", arlib::routing_kernels::astar)
      .value("bidirectional_dijkstra",
             arlib::routing_kernels::bidirectional_dijkstra);

  m.def("esx", &esx, "ESX alternative routing algorithm", "path_or_graph"_a,
        "source"_a, "target"_a, "k"_a, "theta"_a, "kernel"_a, "gtype"_a);
  m.def("onepass_plus", &onepass_plus, "OnePass+ alternative routing algorithm",
        "path_or_graph"_a, "source"_a, "target"_a, "k"_a, "theta"_a, "gtype"_a);
  m.def("penalty", &penalty, "Penalty alternative routing algorithm",
        "path_or_graph"_a, "source"_a, "target"_a, "k"_a, "theta"_a, "p"_a,
        "r"_a, "max_nb_updates"_a, "max_nb_steps"_a, "kernel"_a, "gtype"_a);
}