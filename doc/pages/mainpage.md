# Main page {#mainpage}

**ARLib** is a generic library for [Boost.Graph][boost-graph] providing several
**Alternative-Routing** algorithms. Alternative routing, also known in the
literature as *alternative route planning* or *k-shortest paths with limited
overlapping*, is the problem of finding a number of *good s-t* paths in a graph.
While the problem of finding *the* shortest path between a pair of nodes has
been intensively studied and efficient solutions are well-known (most notably
the [Dijkstra's algorithm]), finding several, high-quality paths introduces some
challenges. In fact, we look for *s-t* paths that are **a)** as short as
possible **b)** sufficiently dissimilar from each other.

### Features

ARLib implements the following state-of-the-art algorithms to solve the problem:
 - [k-SPwLO OnePass+](@ref opplus-page)
 - [k-SPwLO ESX](@ref esx-page)
 - [Penalty](@ref penalty-page)

Moreover, the following algorithms are also available:
 - Bidirectional Dijkstra - *A speed-up variant of Dijkstra's algorithm that
   searches the graph both from the source and the target for faster
   convergence*.
 - Uninformed Bidirectional Pruner - *A pre-processing algorithm to prune a 
   graph from those vertices that unlikely could be part of an s-t path*.

### Scope

This documentation describes the **internal** software that makes up `ARLib`, not the **external** use of `ARLib`. There are no instructions here on how to use `ARLib`, only the APIs that make up the software. For usage instructions, please see the [Tutorial][tutorial] section in [GitHub repository][github].

[Dijkstra's algorithm]: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
[boost-graph]: https://www.boost.org/doc/libs/1_68_0/libs/graph/doc/index.html
[tutorial]: https://github.com/leonardoarcari/arlib#getting--started
[github]: https://github.com/leonardoarcari/arlib
