#include <catch.hpp>

#include <arlib/multi_predecessor_map.hpp>
#include <test_types.hpp>
#include <utils.hpp>

#include <boost/graph/properties.hpp>

#include <arlib/graph_types.hpp>
#include <arlib/graph_utils.hpp>

TEST_CASE("multi_predecessor_map interface") {
  using namespace boost;
  using namespace arlib::test;

  auto G = arlib::read_graph_from_string<Graph>(std::string(graph_gr_esx));
  auto pmap = arlib::multi_predecessor_map<Vertex>{};

  BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<decltype(pmap), Vertex>));
  for (auto [first, last] = vertices(G); first != last; ++first) {
    REQUIRE(get(pmap, *first).empty());
  }

  auto v = *vertices(G).first;
  auto &pred = get(pmap, v);
  for (auto [first, last] = in_edges(v, G); first != last; ++first) {
    auto u = source(*first, G);
    auto k_th = std::distance(in_edges(v, G).first, first);
    pred.insert({k_th, u});
  }

  REQUIRE(pred.size() == in_degree(v, G));
  graph_traits<Graph>::in_edge_iterator first, last;
  for (std::tie(first, last) = in_edges(v, G); first != last; ++first) {
    REQUIRE(
        std::find_if(pred.begin(), pred.end(), [&first, &G](auto const &entry) {
          return entry.second == source(*first, G);
        }) != pred.end());
  }
}

void fill_kth_predecessors(
    int kth, arlib::test::Graph const &G,
    arlib::multi_predecessor_map<arlib::test::Vertex> &predecessors,
    std::vector<arlib::test::Edge> const &edges) {
  using boost::get;
  for (auto const &e : edges) {
    auto u = boost::source(e, G);
    auto v = boost::target(e, G);
    get(predecessors, v).insert({kth, u});
  }
}

TEST_CASE("to_paths builds correct vector<Path> from multi_predecessor_map") {
  using namespace boost;
  using namespace arlib::test;

  std::pair<int, int> edge_list[] = {{0, 1}, {1, 3}, {0, 2}, {2, 3},
                                     {3, 4}, {4, 6}, {3, 5}, {5, 6}};
  int weights[] = {1, 1, 3, 3, 1, 1, 3, 3};
  auto G = Graph{edge_list,
                 edge_list + sizeof(edge_list) / sizeof(std::pair<int, int>),
                 weights, 7};

  auto pmap = arlib::multi_predecessor_map<Vertex>{};
  fill_kth_predecessors(1, G, pmap,
                        {edge(0, 1, G).first, edge(1, 3, G).first,
                         edge(3, 4, G).first, edge(4, 6, G).first});
  fill_kth_predecessors(2, G, pmap,
                        {edge(0, 2, G).first, edge(2, 3, G).first,
                         edge(3, 5, G).first, edge(5, 6, G).first});

  auto paths = arlib::to_paths(G, pmap, 0lu, 6lu);

  REQUIRE(paths.size() == 2);

  auto lengths = std::vector<Length>{4, 12};
  for (auto l : lengths) {
    REQUIRE(std::any_of(paths.begin(), paths.end(),
                        [l](auto const &p) { return p.length() == l; }));
  }
}
