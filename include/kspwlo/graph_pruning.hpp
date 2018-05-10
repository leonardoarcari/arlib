#ifndef BOOST_GRAPH_PRUNING_ALGORITHMS_HPP
#define BOOST_GRAPH_PRUNING_ALGORITHMS_HPP

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/properties.hpp>

#include "kspwlo/bidirectional_dijkstra.hpp"
#include "kspwlo/impl/kspwlo_impl.hpp"
#include "kspwlo/visitor.hpp"

#include <limits>
#include <set>
#include <vector>

namespace boost {
template <typename Graph,
          typename Vertex = typename graph_traits<Graph>::vertex_descriptor>
Graph uninformed_bidirectional_pruner(const Graph &G, Vertex s, Vertex t,
                                      double tau) {
  using Length = typename property_traits<
      typename property_map<Graph, edge_weight_t>::type>::value_type;
  // Instantiate data structures
  auto weight_f = get(edge_weight, G);
  auto distance_f_vec = std::vector<kspwlo::Length>(num_vertices(G), 6);
  auto predecessor_f_vec =
      std::vector<Vertex>(vertices(G).first, vertices(G).second);
  auto vertex_id = get(vertex_index, G);
  auto distance_f =
      make_iterator_property_map(std::begin(distance_f_vec), vertex_id);
  auto predecessor_f =
      make_iterator_property_map(std::begin(predecessor_f_vec), vertex_id);

  // Reverse tree
  auto rev = make_reverse_graph(G);
  auto weight_b = get(edge_weight, rev);
  auto vertex_id_b = get(vertex_index, G);
  auto distance_b_vec = std::vector<Length>(num_vertices(rev));
  auto predecessor_b_vec =
      std::vector<Vertex>(vertices(rev).first, vertices(rev).second);
  auto predecessor_b =
      make_iterator_property_map(std::begin(predecessor_b_vec), vertex_id_b);

  auto distance_b =
      make_iterator_property_map(std::begin(distance_b_vec), vertex_id_b);

  // Pruning visitor
  auto pruning_visitor = UninformedBiPrunerVisitor<Vertex>{tau};

  bidirectional_dijkstra(G, s, t, predecessor_f, distance_f, weight_f, rev,
                         predecessor_b, distance_b, weight_b, pruning_visitor);

  auto pruned_G = Graph{G};

  auto sp = kspwlo_impl::build_edge_list_from_dijkstra(s, t, predecessor_f);
  auto final_distance = kspwlo_impl::compute_length_from_edges(sp, G);
  for (auto [v_it, v_end] = pruning_visitor.get_pruned_vertices();
       v_it != v_end; ++v_it) {
    // Source and Target vertices must not be pruned.
    if (*v_it != s && *v_it != t) {
      // If v is not in any shortest path from forward nor backward, prune it.
      bool found_v_f = (predecessor_f[*v_it] != *v_it);
      bool found_v_b = (predecessor_b[*v_it] != *v_it);
      Length inf = std::numeric_limits<Length>::max();
      if (distance_f[*v_it] == inf && !found_v_b) {
        clear_vertex(*v_it, pruned_G);
        remove_vertex(*v_it, pruned_G);
        continue;
      }

      if (distance_b[*v_it] == inf && !found_v_f) {
        clear_vertex(*v_it, pruned_G);
        remove_vertex(*v_it, pruned_G);
        continue;
      }

      if (!found_v_f && !found_v_b) {
        // If a node was not added to SP-Trees but we met it from both
        // directions we must keep it!
        if (distance_f[*v_it] == inf && distance_b[*v_it] == inf) {
          clear_vertex(*v_it, pruned_G);
          remove_vertex(*v_it, pruned_G);
          continue;
        }
      }

      // If distance_s_v + distance_t_v > tau * final_distance, prun it
      if (distance_f[*v_it] != inf && distance_b[*v_it] != inf) {
        if (distance_f[*v_it] + distance_b[*v_it] > tau * final_distance) {
          clear_vertex(*v_it, pruned_G);
          remove_vertex(*v_it, pruned_G);
          continue;
        }
      }
    }
  }

  return pruned_G;
}
} // namespace boost

#endif
