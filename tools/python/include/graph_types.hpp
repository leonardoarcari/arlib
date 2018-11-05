/**
 * @file graph_types.hpp
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

#ifndef ARLIB_PYTHON_GRAPH_TYPES_HPP
#define ARLIB_PYTHON_GRAPH_TYPES_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <memory>
#include <vector>

enum class Graphs { AdjListInt = 1, AdjListFloat };

struct AdjListInt {};
struct AdjListFloat {};

using GrAdjListInt =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                          boost::no_property,
                          boost::property<boost::edge_weight_t, int>>;
using GrAdjListFloat =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                          boost::no_property,
                          boost::property<boost::edge_weight_t, double>>;

using ARReturnType = std::vector<std::vector<long unsigned int>>;

#endif