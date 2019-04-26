# Extending ARLib

So you decided to contribute to ARLib project and implement a new **Alternative Route Planning** heuristic. First of all, thank you a lot for your precious time and appreciated effort.

ARLib's strength relies on the efficiency of alternative routing algorithms implementations and high configurability, allowing the developer to switch from one heuristic to another and tune research parameters. Having as many implementations of heuristics as possible inevitabily raise the value of this project.

In the remainder of this document, I will guide you through the main design points to keep in mind while implementing a new routing heuristic in order to gratefully merge with existing ones. Let's get it started!

- [Extending ARLib](#extending-arlib)
  - [Source files](#source-files)
  - [Function Interface](#function-interface)
      - [Concepts](#concepts)
      - [Property Maps](#property-maps)
      - [Interface example](#interface-example)
      - [Multi Predecessor Map](#multi-predecessor-map)
  - [Concept checking](#concept-checking)
      - [Checking graph concept](#checking-graph-concept)
      - [Checking property map concept](#checking-property-map-concept)
  - [Function body](#function-body)
      - [Filling a Multi Predecessor Map](#filling-a-multi-predecessor-map)
      - [Adding Terminator support for early stopping](#adding-terminator-support-for-early-stopping)

## Source files

The first aspect to consider is where should we put our new heuristic implementation. Let us suppose to be developing a new algorithm called *k-shortest paths*. As you already known, ARLib is a generic library, strongly relying on C++ templates. Therefore, function declarations and definitions must be put all in the same source file. Just like we did for other algorithms, like OnePass+, we create a ```.hpp``` header file in ```include/arlib``` folder with the name of the heuristic, together with include guards

```cpp
/// k_shortest_paths.hpp

#ifndef ARLIB_K_SHORTEST_PATHS_HPP
#define ARLIB_K_SHORTEST_PATHS_HPP

#endif
```

Moreover, since we want to keep user-exposed include files as simple as possible, with minimum implementation details and just documentation, we are going to create an implementation header file in ```include/arlib/details``` with an ```_impl``` suffix to the heuristic name

```cpp
/// k_shortest_paths_impl.hpp

#ifndef ARLIB_K_SHORTEST_PATHS_IMPL_HPP
#define ARLIB_K_SHORTEST_PATHS_IMPL_HPP

#endif
```

and of course, we include that in its main header

```cpp
/// k_shortest_paths.hpp

#ifndef ARLIB_K_SHORTEST_PATHS_HPP
#define ARLIB_K_SHORTEST_PATHS_HPP

#include "arlib/details/k_shortest_paths_impl.hpp"

#endif
```

## Function Interface

Now that we have header files in place, let's move to the algorithm function declaration. As you know, ARLib relies on Boost.Graph for representing a road network structure. Moreover, ARLib follows the same Boost.Graph coding conventions for algorithms and parameters passing. We strongly suggest to read the [this chapter in Boost.Graph documentation](https://www.boost.org/doc/libs/1_70_0/libs/graph/doc/constructing_algorithms.html) as a basis for the remainder of this document. 

The two key aspects of Boost.Graph conventions are **Concepts** and **Property Maps**.

#### Concepts
Being Boost.Graph and ARLib generic libraries, concrete graph implementations are abstracted by means of *concepts*, which define the set of operation allowed on some generic type, together with their input and outputs. Therefore, instead of taking some graph type as input, like an ```adjacency_list```, every ARLib algorithm accepts a generic ```Graph``` template parameter, respecting some properties. You can read more about graph concepts on [Boost.Graph documentation](https://www.boost.org/doc/libs/1_70_0/libs/graph/doc/graph_concepts.html).

#### Property Maps
On the other hand, properties of a graph are passed to routing algorithms by means of *Property Maps*, i.e. objects mapping to graph element, such as a vertex or an edge, a value representing some property. A common example is the **EdgeWeightMap**, a map associating to each graph edge a *weight*, possibly used to compute the shortest path between two nodes. You can read more about property maps on [Boost.Graph documentation](https://www.boost.org/doc/libs/1_70_0/libs/property_map/doc/property_map.html)

#### Interface example
By means of Concepts and Property Maps, we are able to completely decouple a graph structure from its properties, and thus using the same algorithm implementation for any graph type modelling the required concept.

In the context of implementing an alternative route planning algorithm, we should identify the information we need to compute a set of alternative paths. For our example, let us assume that we need the following information:

 - The graph *G* on which to search
 - The property map *EdgeWeightMap* providing a weight for each edge
 - Number of alternative paths *k*
 - Similarity threshold *theta*
 - The source vertex *s* and the destination vertex *v*

Therefore, an initial draft of our function interface might be:

```cpp
/// k_shortest_paths.hpp

#ifndef ARLIB_K_SHORTEST_PATHS_HPP
#define ARLIB_K_SHORTEST_PATHS_HPP

#include "arlib/details/k_shortest_paths_impl.hpp"

namespace arlib {
template <typename Graph, typename EdgeWeightMap, typename Vertex>
void k_shortest_paths(Graph const& G, EdgeWeightMap weight, 
                      Vertex s, Vertex t,
                      int k, double theta);
}

#endif
```

First of all, we put the function under the **arlib** namespace. Then, we declare a function with the name of the algorithm, taking the arguments we discussed about. We highlight that we pass the ```EdgeWeightMap``` by copy as, by convention, Property Maps are defined to be cheap to copy.

#### Multi Predecessor Map
As you might notice, we declared the function to be ```void```. Why so? In the Boost.Graph framework, results are computed and stored in *output parameters* passed to the function itself. 

In our context, we are writing a function to compute *k* alternative paths from *s* to *t*. For this reason, ARLib provides a Property Map to store such result: **Multi Predecessor Map**.

It is defined in ```include/arlib/multi_predecessor_map.hpp``` in ```multi_predecessor_map``` class. 

It models the
`ReadablePropertyMap` concept and we use it to keep track of the predecessors of
each vertex in the alternative routes. Its `key_type` is the same as the vertex 
descriptor of the graph. The `value_type` is an `UnorderedAssociativeContainer`,
like `std::unordered_map`, such that each entry maps an `int` to a vertex
descriptor. The `int` value represents the index of the alternative path,
which ranges in `[0, k]`. 

For instance, let `v` be a node of the graph and `MP` be a 
`multi_predecessor_map`. `MP[v]` returns a reference to an 
`UnorderedAssociativeContainer` `P`. Each entry of `P` is a pair `(n, p)`, where
`n` is the index of the alternative path for which `p` is the predecessor of `v`.

Therefore, we modify the previously declared function interface to allow for a Multi Predecessor Map output parameter.

```cpp
/// k_shortest_paths.hpp

#ifndef ARLIB_K_SHORTEST_PATHS_HPP
#define ARLIB_K_SHORTEST_PATHS_HPP

#include "arlib/details/k_shortest_paths_impl.hpp"

namespace arlib {
template <typename Graph, typename EdgeWeightMap,
          typename MultiPredecessorMap, typename Vertex>
void k_shortest_paths(Graph const& G, EdgeWeightMap weight, 
                      MultiPredecessorMap &predecessors,
                      Vertex s, Vertex t,
                      int k, double theta);
}

#endif
```

Here we pass a *MultiPredecessorMap* parameter by reference as we want to fill it for the caller inside the *k_shortest_paths* function body.

Great! Now we have our function interface. It's time to move to function body!

## Concept checking

As we previosuly stated, ARLib strongly relies on Concepts for abstracting graph concrete types and property maps, making it possible to write a single algorithm for every possible graph implementation satisfying some concept requirements. Therefore, the first thing to do in function body is to check if passed template parameters actually satisfy those requirements. In order to do that, we rely on Boost.Graph concepts.

#### Checking graph concept

The first thing to check is whether the passed ```Graph``` type satisfies all the operations we expect to use in the function body. Let's suppose that *k-shortest paths* algorithm requires just to explore out-going edges from a given node on the graph, say starting from *s* until *t* is discovered. By looking at [Boost.Graph concepts page](https://www.boost.org/doc/libs/1_70_0/libs/graph/doc/graph_concepts.html) we notice that an [**IncidenceGraph**](https://www.boost.org/doc/libs/1_70_0/libs/graph/doc/IncidenceGraph.html) is all we are gonna need. Therefore, the first line of our function body is going to check if ```Graph``` parameter satisfies IncidenceGraph requirements.

```cpp
/// k_shortest_paths.hpp

#ifndef ARLIB_K_SHORTEST_PATHS_HPP
#define ARLIB_K_SHORTEST_PATHS_HPP

#include "arlib/details/k_shortest_paths_impl.hpp"

#include <boost/graph/graph_concepts.hpp>

namespace arlib {
template <typename Graph, typename EdgeWeightMap,
          typename MultiPredecessorMap, typename Vertex>
void k_shortest_paths(Graph const& G, EdgeWeightMap weight, 
                      MultiPredecessorMap &predecessors,
                      Vertex s, Vertex t,
                      int k, double theta) {
  using namespace boost;
  // Concept checking macro 
  BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept<Graph> ));
}
}

#endif
```

In this way, if ```Graph``` template parameter is substituted by something not modelling an IncidenceGraph a *gentle* compilation message will inform you on what operations are not supported on that type.

#### Checking property map concept

In the same way, we are going to check on Property Maps requirements, to assert that they behave as expected. In particular, as we expect to read from both ```EdgeWeightMap``` and ```MultiPredecessorMap``` we are going to assert that they both model **ReadablePropertyMap** concept, using Edges and Vertices, respectiveley, as map keys.

```cpp
/// k_shortest_paths.hpp

#ifndef ARLIB_K_SHORTEST_PATHS_HPP
#define ARLIB_K_SHORTEST_PATHS_HPP

#include "arlib/details/k_shortest_paths_impl.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>

namespace arlib {
template <typename Graph, typename EdgeWeightMap,
          typename MultiPredecessorMap, typename Vertex>
void k_shortest_paths(Graph const& G, EdgeWeightMap weight, 
                      MultiPredecessorMap &predecessors,
                      Vertex s, Vertex t,
                      int k, double theta) {
  using namespace boost;
  using Edge = typename graph_traits<Graph>::edge_descriptor;

  // Graph concept checking
  BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept<Graph> ));
  // Property Maps concept checking
  BOOST_CONCEPT_ASSERT(
      (ReadablePropertyMapConcept<WeightMap, Edge>));
  BOOST_CONCEPT_ASSERT(
      (ReadablePropertyMapConcept<MultiPredecessorMap, Vertex>));
}
}

#endif
```

Great! Now we can be sure, from a type point-of-view, of the correctness of input parameters and operations that we are going to perform on them. It's time to implement the real algorithm function body.

## Function body

#### Filling a Multi Predecessor Map

Let's suppose that you wrote your full implementation of *k-shortest path* algorithm and you want to return the discovered alternative routes to the caller. As we said earlier, in ARLib this is done by means of a Multi Predecessor Map.

Let's say that at step *i* we computed the *i-th* alternative path from *s* to *t*, composed of edges ```[(s, n1), (n1, n5), (n5, t)]```. To return this result we should encode in the Multi Predecessor Map that, for path *i*, *n5* is a predecessor of *t*, *n1* is a predecessor if *n5* and, finally, that *s* is a predecessor of *n1*.

A possible implementation migh be the following:

```cpp
/// k_shortest_paths_impl.hpp

using namespace boost;

auto ith_path = std::vector<Edge>{ /* (s, n1), (n1, n5), (n5, t) */ };

for (auto const& e : ith_path) {
  auto u = source(e, G);
  auto v = target(e, G);
  // Get predecessors of 'v' in any alternative path.
  auto &preds = get(predecessors, v);
  // Record that for path number 'i', the predecessor of vertex 'v' is 'u'.
  preds.insert({i, u});
}
```

#### Adding Terminator support for early stopping

One of the major limitations of alternative routing heuristics is the execution time, which can easily explode when the number of alternative paths *k* is too high and the similarity threshold *theta* is too low.

For this reason, it can be very handy to have the execution terminated sooner if the execution time passes some given timeout. In ARLib, this behavior is implemented by means of visitor objects, called **Terminators**. A Terminator object exposes the following interface

```cpp
struct terminator {
  bool should_stop() const;
};
```

and the Alternative Route Planning algorithm should call ```should_stop()``` method periodically and terminate the algorithm execution if the call returns ```true```.

In ARLib, a ```timer``` Terminator is implemented to terminate the algorithm execution time after a given amount of time and it's available in ```include/arlib/terminators.hpp```.

In order to add the support of a Terminator to your algorithm, it's enough to add the following to the function definition:

```cpp
/// k_shortest_paths.hpp

#ifndef ARLIB_K_SHORTEST_PATHS_HPP
#define ARLIB_K_SHORTEST_PATHS_HPP

#include "arlib/terminators.hpp"
#include "arlib/details/k_shortest_paths_impl.hpp"

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>

namespace arlib {
template <typename Graph, typename EdgeWeightMap,
          typename MultiPredecessorMap, typename Vertex,
          // Add Terminator template parameter defaulted to 
          // 'arlib::always_continue'. This way, if no
          // Terminator is passed, the algorithm continues
          // until completion. 
          typename Terminator = arlib::always_continue>
void k_shortest_paths(Graph const& G, EdgeWeightMap weight, 
                      MultiPredecessorMap &predecessors,
                      Vertex s, Vertex t,
                      int k, double theta,
                      // Default argument, in case no Terminator
                      // is passed.
                      Terminator &&terminator = Terminator{}) {
  /* ... */
  
  // Implementation main loop
  while ( /* condition */ ) {
    if (terminator.should_stop())
      return;
  }
  
  /* ... */
}
}

#endif
```

In the above example, the terminating condition check has been introduced at the very beginning of the main loop of the algorithm. This is done assuming that Alternative Route Planning algorithms are designed in terms of iterative operations on the graph to build alternative paths one after the other. Therefore, that looks like a sweet spot to check if the execution should terminate, especially considering a ```timer``` Terminator which should force the heuristic to end as soon as the time has expired.

----------------------------