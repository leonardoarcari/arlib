#ifndef BOOST_GRAPH_PRUNING_ALGORITHMS_HPP
#define BOOST_GRAPH_PRUNING_ALGORITHMS_HPP

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/properties.hpp>

#include <arlib/bidirectional_dijkstra.hpp>
#include <arlib/details/arlib_utils.hpp>
#include <arlib/visitor.hpp>

#include <limits>
#include <unordered_set>
#include <vector>

namespace arlib {
template <typename Vertex, typename FPredecessorMap, typename BPredecessorMap,
          typename FDistanceMap, typename BDistanceMap, typename Length>
bool pruning_policy(Vertex s, Vertex t, Vertex v, FPredecessorMap predecessor_f,
                    BPredecessorMap predecessor_b, FDistanceMap distance_f,
                    BDistanceMap distance_b, double tau,
                    Length final_distance) {
  // Source and Target vertices must not be pruned.
  if (v != s && v != t) {
    // If v is not in any shortest path from forward nor backward, prune it.
    bool found_v_f = (predecessor_f[v] != v);
    bool found_v_b = (predecessor_b[v] != v);
    Length inf = std::numeric_limits<Length>::max();
    if (distance_f[v] == inf && !found_v_b) {
      return true;
    }

    if (distance_b[v] == inf && !found_v_f) {
      return true;
    }

    if (!found_v_f && !found_v_b) {
      // If a node was not added to SP-Trees but we met it from both
      // directions we must keep it!
      if (distance_f[v] == inf && distance_b[v] == inf) {
        return true;
      }
    }

    // If distance_s_v + distance_t_v > tau * final_distance, prun it
    if (distance_f[v] != inf && distance_b[v] != inf) {
      if (distance_f[v] + distance_b[v] > tau * final_distance) {
        return true;
      }
    }
  }
  return false;
}

template <
    typename Graph, typename WeightMap, typename RevWeightMap,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor>
Graph uninformed_bidirectional_pruner(const Graph &G, WeightMap const &weight_f,
                                      boost::reverse_graph<Graph> const &rev_G,
                                      RevWeightMap const &weight_b, Vertex s,
                                      Vertex t, double tau) {
  using namespace boost;
  using Length = typename boost::property_traits<WeightMap>::value_type;
  using Edge = typename graph_traits<Graph>::edge_descriptor;
  using RevEdge =
      typename graph_traits<boost::reverse_graph<Graph>>::edge_descriptor;

  BOOST_CONCEPT_ASSERT((VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT(
      (VertexAndEdgeListGraphConcept<boost::reverse_graph<Graph>>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<WeightMap, Edge>));
  BOOST_CONCEPT_ASSERT((LvaluePropertyMapConcept<RevWeightMap, RevEdge>));

  // Instantiate data structures
  auto distance_f_vec = std::vector<Length>(num_vertices(G), 6);
  auto predecessor_f_vec =
      std::vector<Vertex>(vertices(G).first, vertices(G).second);
  auto vertex_id = get(vertex_index, G);
  auto distance_f =
      make_iterator_property_map(std::begin(distance_f_vec), vertex_id);
  auto predecessor_f =
      make_iterator_property_map(std::begin(predecessor_f_vec), vertex_id);

  // Reverse tree
  auto vertex_id_b = get(vertex_index, G);
  auto distance_b_vec = std::vector<Length>(num_vertices(rev_G));
  auto predecessor_b_vec =
      std::vector<Vertex>(vertices(rev_G).first, vertices(rev_G).second);
  auto predecessor_b =
      make_iterator_property_map(std::begin(predecessor_b_vec), vertex_id_b);

  auto distance_b =
      make_iterator_property_map(std::begin(distance_b_vec), vertex_id_b);

  // Pruning visitor
  auto pruning_visitor = UninformedBiPrunerVisitor<Vertex>{tau};

  bidirectional_dijkstra(G, s, t, predecessor_f, distance_f, weight_f, rev_G,
                         predecessor_b, distance_b, weight_b, pruning_visitor);

  auto pruned_G = Graph{G};

  auto sp = details::build_edge_list_from_dijkstra(s, t, predecessor_f);
  auto final_distance = details::compute_length_from_edges(sp, G, weight_f);
  for (auto [v_it, v_end] = pruning_visitor.get_pruned_vertices();
       v_it != v_end; ++v_it) {
    bool should_prune =
        pruning_policy(s, t, *v_it, predecessor_f, predecessor_b, distance_f,
                       distance_b, tau, final_distance);
    if (should_prune) {
      for (auto [adj_it, adj_end] = adjacent_vertices(*v_it, pruned_G);
           adj_it != adj_end; ++adj_it) {
        auto u = *adj_it;
        auto v = *v_it;

        auto [uv_e, uv_exists] = edge(u, v, pruned_G);
        if (uv_exists) {
          remove_edge(u, v, pruned_G);
        }

        auto [vu_e, vu_exists] = edge(v, u, pruned_G);
        if (vu_exists) {
          remove_edge(v, u, pruned_G);
        }
      }
    }
  }

  return pruned_G;
}

template <typename PropertyGraph,
          typename Vertex =
              typename boost::graph_traits<PropertyGraph>::vertex_descriptor>
PropertyGraph uninformed_bidirectional_pruner(const PropertyGraph &G, Vertex s,
                                              Vertex t, double tau) {
  using namespace boost;
  using Edge = typename graph_traits<PropertyGraph>::edge_descriptor;

  BOOST_CONCEPT_ASSERT(
      (PropertyGraphConcept<PropertyGraph, Edge, edge_weight_t>));

  auto weight = get(edge_weight, G);
  auto rev = make_reverse_graph(G);
  auto weight_b = get(edge_weight, rev);
  return uninformed_bidirectional_pruner(G, weight, rev, weight_b, s, t, tau);
}
} // namespace arlib

#endif
