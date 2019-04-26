#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include "arlib/arlib.hpp"

#include <iostream>
#include <string>
#include <vector>

// Create type-aliases for the Graph type
using Graph = boost::adjacency_list<boost::vecS, boost::vecS,
                                    boost::bidirectionalS, boost::no_property,
                                    boost::property<boost::edge_weight_t, int>>;
using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
using Edge = typename boost::graph_traits<Graph>::edge_descriptor;

std::vector<arlib::Path<Graph>> get_alternative_routes(Graph const &G, Vertex s,
                                                       Vertex t) {
  // Make output MultiPredecessorMap
  auto predecessors = arlib::multi_predecessor_map<Vertex>{};

  int k = 3;                                       // Nb alternative routes
  double theta = 0.5;                              // Overlapping threshold
  auto weight = boost::get(boost::edge_weight, G); // Get Edge WeightMap

  arlib::onepass_plus(G, weight, predecessors, s, t, k, theta);
  auto alt_routes = arlib::to_paths(G, predecessors, weight, s, t);
  return alt_routes;
}

void print_path(arlib::Path<Graph> const &path,
                std::vector<std::string> const &name) {
  using namespace boost;

  for (auto [v_it, v_end] = vertices(path); v_it != v_end; ++v_it) {
    for (auto [e_it, e_end] = out_edges(*v_it, path); e_it != e_end; ++e_it) {
      std::cout << name[source(*e_it, path)] << " -- "
                << name[target(*e_it, path)] << "\n";
    }
  }
}

int main() {
  // Make convenient labels for the vertices
  enum { S, N1, N2, N3, N4, N5, T };
  const long unsigned num_vertices = T;
  const auto name =
      std::vector<std::string>{"s", "n1", "n2", "n3", "n4", "n5", "t"};

  // Writing out the edges in the graph
  const auto edges = std::vector<std::pair<int, int>>{
      {S, N1},  {N1, S},  {S, N2},  {N2, S},  {S, N3},  {N3, S},
      {N1, T},  {T, N1},  {N3, N1}, {N1, N3}, {N3, N5}, {N5, N3},
      {N3, N2}, {N2, N3}, {N3, N4}, {N4, N3}, {N2, N4}, {N4, N2},
      {N5, T},  {T, N5},  {N5, N4}, {N4, N5}, {N4, T},  {T, N4}};

  const auto weights = std::vector<int>{6, 6, 4, 4, 3, 3, 6, 6, 2, 2, 3, 3,
                                        3, 3, 5, 5, 5, 5, 2, 2, 1, 1, 2, 2};
  auto G = Graph{edges.begin(), edges.end(), weights.begin(), num_vertices};

  //=-------------------------------------------------------------------------=

  auto alt_routes = get_alternative_routes(G, S, T);

  for (auto const &route : alt_routes) {
    print_path(route, name);
    std::cout << "--------\n";
  }

  return 0;
}
