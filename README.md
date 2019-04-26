# ARLib - Alternative Routing Library for Boost.Graph
| Branch | Travis CI |
| ------ | --------- |
| master | [![Build Status](https://travis-ci.com/leonardoarcari/arlib.svg?branch=master)](https://travis-ci.com/leonardoarcari/arlib)

**ARLib** is a generic library for [Boost.Graph][boost-graph] providing several
**Alternative Routing** algorithms. Alternative routing, also known in the
literature as *alternative route planning* or *k-shortest paths with limited
overlapping*, is the problem of finding a number of *good s-t* paths in a graph.
While the problem of finding *the* shortest path between a pair of nodes has
been intensively studied and efficient solutions are well-known (most notably
the [Dijkstra's algorithm]), finding several, high-quality paths introduces some
challenges. In fact, we look for *s-t* paths that are **(a)** as short as
possible **(b)** sufficiently dissimilar from each other.

### Features

ARLib implements the following state-of-the-art algorithms to solve the problem:
 - [*k-SPwLO* OnePass+][opplus]
 - [*k-SPwLO* ESX][esx]
 - [Penalty][penalty]

Moreover, the following algorithms are also available:
 - [Bidirectional Dijkstra][bidijkstra] - *A speed-up variant of Dijkstra's algorithm that
   searches the graph both from the source and the target for faster
   convergence*.
 - [Uninformed Bidirectional Pruner][bidijpruner] - *A pre-processing algorithm to prune a 
   graph from those vertices that unlikely could be part of an s-t path*.


### Requirements

 - A C++17 compliant compiler
 - [CMake] (>= 3.5)
 - [Boost::Graph][boost-graph] (>= 1.65)
 
### Getting  started

If you want to use this library, please check the following resources
 - [Include ARLib in your CMake project](examples/include_in_cmake_project.md)
 - [Tutorial](examples/getting_started.md)
 - [Documentation]
 - [Extending ARLib](examples/extending_arlib.md)

### Motivation

In the context of software frameworks for managing and operating on graphs, **Boost.Graph library** (BGL) is an established solution, known for providing a high-quality, high-performance implementation of graph structures and algorithms operating on them. Moreover, the generic interface that the library provides, along with its permissive license, makes BGL a valuable choice for the development of software systems that leverage graphs.

The purpose of ARLib is to provide a set of algorithms that solve the *alternative routing* problem for systems using BGL, for which an **open source, high-performance C++ implementation is**, as of now, **missing**. In the interest of preserving the generality of usage of BGL, ARLib follows the same conventions that official BGL algorithms set, so that ARLib users will find executing *alternative routing* algorithms on their graphs natural and non-intrusive.

### References
The algorithm provided in this library are implementations of algorithms
described in these scientific papers:

| Algorithm                         | Paper |
|---------------------------------- | ----- |
| OnePass+                          | Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper and UlfLeser, Exact and Approximate Algorithms for Finding k-Shortest Paths with Limited Overlap , In Proc. of the 20th Int. Conf. on Extending Database Technology (EDBT) (2017)
| ESX                               | *Same as above*
| Penalty                           | Yanyan Chen, Michael GH Bell, and Klaus Bogenberger. Reliable pretrip multipath planning and dynamic adaptation for a centralized road navigation system. Intelligent Transportation Systems, IEEE Transactions on, 8(1):14–20, 2007
| Bidirectional Dijkstra            | Nicholson, T. Alastair J. "Finding the shortest route between two points in a network." The computer journal 9.3 (1966): 275-280.
| Uninformed Bidirectional Pruner   | Andreas Paraskevopoulos, Christos Zaroliagis. Improved Alternative Route Planning. Daniele Frigioni and Sebastian Stiller. ATMOS - 13th Workshop on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems - 2013, Sep 2013, Sophia Antipolis, France. Schloss Dagstuhl–Leibniz-Zentrum fuer Informatik, 33, pp.108–122, 2013, OpenAccess Series in Informatics (OASIcs).

[Dijkstra's algorithm]: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
[CMake]: https://cmake.org/cmake/help/v3.5/
[boost-graph]: https://www.boost.org/doc/libs/1_68_0/libs/graph/doc/index.html
[Documentation]: https://leonardoarcari.github.io/arlib
[opplus]: https://leonardoarcari.github.io/arlib/namespacearlib.html#a2483e66e28c003d4986e884306d95a7f
[esx]: https://leonardoarcari.github.io/arlib/namespacearlib.html#aefd973ec7b5bcd76458d09445495d9b4
[penalty]: https://leonardoarcari.github.io/arlib/namespacearlib.html#adaeac433c566ef6c94b20870769344d4
[bidijkstra]: https://leonardoarcari.github.io/arlib/namespacearlib.html#a8faaab97ab54b2cace47a7fcb09622aa
[bidijpruner]: https://leonardoarcari.github.io/arlib/namespacearlib.html#a355efeea09e4955c318a79d58099d167
