#ifndef KSPWLO_IMPL_HPP
#define KSPWLO_IMPL_HPP

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>

#include <sstream>
#include <stdexcept>
#include <vector>

namespace arlib {
namespace details {
//===----------------------------------------------------------------------===//
//                      kSPwLO algorithms support classes
//===----------------------------------------------------------------------===//

struct target_found {};
struct target_not_found : public std::logic_error {
  explicit target_not_found(const std::string &what_arg)
      : std::logic_error(what_arg) {}
  explicit target_not_found(const char *what_arg)
      : std::logic_error(what_arg) {}
};

template <typename Vertex, typename Tag>
class target_visitor : public boost::base_visitor<target_visitor<Vertex, Tag>> {
public:
  using event_filter = Tag;

  explicit target_visitor(Vertex t) : t{t} {}
  template <typename Graph> void operator()(Vertex u, Graph &) {
    if (u == t) {
      throw target_found{};
    }
  }

private:
  Vertex t;
};
template <typename Vertex, typename Tag>
target_visitor<Vertex, Tag> make_target_visitor(Vertex t, Tag) {
  return target_visitor<Vertex, Tag>{t};
}

//===----------------------------------------------------------------------===//
//                      kSPwLO algorithms routines
//===----------------------------------------------------------------------===//

template <typename CostType, typename Vertex, typename DistMap>
bool exists_path_to(Vertex v, const DistMap &dist) {
  auto inf = std::numeric_limits<CostType>::max();
  return dist[v] != inf;
}

/**
 * @brief Computes the length of the path in @p candidate using weights from @p
 * G
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @param candidate The candidate path
 * @param G The graph
 * @return The length of @p candidate, i.e. @f$\sum_{e \in candidate} weight(e,
 *         G)@f$
 */
template <typename Graph,
          typename Edge = typename boost::graph_traits<Graph>::edge_descriptor,
          typename Length =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
Length compute_length_from_edges(const std::vector<Edge> &candidate,
                                 const Graph &G) {
  using namespace boost;
  Length length = 0;
  auto weight = get(edge_weight, G);

  for (const auto &[u, v] : candidate) {
    auto egde_in_G = edge(u, v, G);
    bool edge_is_shared = egde_in_G.second;

    if (edge_is_shared) {
      length += weight[egde_in_G.first];
    }
  }

  return length;
}

template <typename Graph,
          typename Length =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
Length compute_length_from_edges(const Graph &candidate, const Graph &G) {
  using namespace boost;
  Length length = 0;
  auto weight = get(edge_weight, G);

  for (auto [it, end] = edges(candidate); it != end; ++it) {
    auto u = source(*it, candidate);
    auto v = target(*it, candidate);
    auto egde_in_G = edge(u, v, G);
    bool edge_is_shared = egde_in_G.second;

    if (edge_is_shared) {
      length += weight[egde_in_G.first];
    }
  }

  return length;
}

/**
 * @brief Evaluates the similarity of a candidate path with respect to some
 * alterative path.
 *
 * This measure of similarity is the one from the reference paper, that is: let
 * @c p' be the candidate path and @c p some alternative path, then
 * @f[
 *   Sim(p', p) = \frac{\sum_{\left(n_x,n_y\right) \in p'\cap p} w_xy}{l(p)}
 * @f]
 *
 * @tparam Graph A Boost::PropertyGraph having at least one edge
 *         property with tag boost::edge_weight_t.
 * @param candidate The candidate path @c p'
 * @param alt_path The alternative path @c p
 * @return The similarity between @c p' and @c p
 */
template <typename Graph,
          typename Edge = typename boost::graph_traits<Graph>::edge_descriptor>
double compute_similarity(const std::vector<Edge> &candidate,
                          const Path<Graph> &alt_path) {
  double shared_length = static_cast<double>(
      compute_length_from_edges(candidate, alt_path.graph()));

  return shared_length / alt_path.length();
}

template <typename Graph>
double compute_similarity(const Path<Graph> &candidate,
                          const Path<Graph> &alt_path) {
  double shared_length = static_cast<double>(
      compute_length_from_edges(candidate.graph(), alt_path.graph()));

  return shared_length / alt_path.length();
}

template <
    typename Graph,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename length_type =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
std::vector<length_type> distance_from_target(const Graph &G, Vertex t) {
  // Reverse graph
  auto G_rev = boost::make_reverse_graph(G);
  auto distance = std::vector<length_type>(boost::num_vertices(G_rev));

  // Run dijkstra_shortest_paths and return distance vector
  boost::dijkstra_shortest_paths(G_rev, t, boost::distance_map(&distance[0]));
  return distance;
}

template <typename Graph, typename PredecessorMap, typename Vertex,
          typename Length =
              typename boost::property_traits<typename boost::property_map<
                  Graph, boost::edge_weight_t>::type>::value_type>
Path<Graph> build_path_from_dijkstra(const Graph &G, const PredecessorMap &p,
                                     Vertex s, Vertex t) {
  Length length = 0;
  auto weight = boost::get(boost::edge_weight, G);
  auto edge_list = std::vector<VPair>{};

  auto current = t;
  while (current != s) {
    auto u = p[current];
    edge_list.emplace_back(u, current);

    auto edge_in_G = boost::edge(u, current, G).first;

    length += weight[edge_in_G];
    current = u;
  }

  return {build_graph_from_edges(edge_list, G), length};
}

template <
    typename Graph,
    typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
    typename Length =
        typename boost::property_traits<typename boost::property_map<
            Graph, boost::edge_weight_t>::type>::value_type>
Path<Graph> compute_shortest_path(const Graph &G, Vertex s, Vertex t) {
  using namespace boost;
  auto sp_distances = std::vector<Length>(num_vertices(G));
  auto predecessor = std::vector<Vertex>(num_vertices(G), s);
  auto vertex_id = get(vertex_index, G);
  try {
    dijkstra_shortest_paths(
        G, s,
        distance_map(&sp_distances[0])
            .predecessor_map(make_iterator_property_map(std::begin(predecessor),
                                                        vertex_id, s))
            .visitor(make_dijkstra_visitor(
                make_target_visitor(t, on_examine_vertex{}))));
  } catch (target_found &tf) {
    return build_path_from_dijkstra(G, predecessor, s, t);
  }
  // If target was not found, t is unreachable from s
  auto oss = std::ostringstream{};
  oss << "Vertex " << t << " is unreachable from " << s;
  throw target_not_found{oss.str()};
}

/**
 * @brief Builds a vector of kspwlo::Edge out of a PredecessorMap computed by a
 * shortest path algorithm from @p s to @p t
 *
 * @tparam Vertex
 * @tparam PredecessorMap
 * @param s The source vertex
 * @param t The target vertex
 * @param p The PredecessorMap
 * @return A vector of the shortest path edges from @p s to @p t.
 */
template <typename Vertex, typename PredecessorMap>
std::vector<VPair> build_edge_list_from_dijkstra(Vertex s, Vertex t,
                                                 const PredecessorMap &p) {
  auto edge_list = std::vector<VPair>{};

  auto current = t;
  while (current != s) {
    auto u = p[current];
    if (u == current) {
      // Vertex 'current' is not reachable from source vertex
      break;
    }
    edge_list.emplace_back(u, current);
    current = u;
  }

  return edge_list;
}
} // namespace details
} // namespace arlib

#endif