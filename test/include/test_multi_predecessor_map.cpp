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
    pred.push_back(u);
  }

  REQUIRE(pred.size() == in_degree(v, G));
  for (auto [first, last] = in_edges(v, G); first != last; ++first) {
    REQUIRE(std::find(pred.begin(), pred.end(), source(*first, G)) !=
            pred.end());
  }
}

TEST_CASE("to_paths builds correct vector<Path> from multi_predecessor_map") {
  using namespace boost;
  using namespace arlib::test;

  std::pair<int, int> edge_list[] = {{0, 1}, {1, 3}, {0, 2}, {2, 1}, {2, 3}};
  int weights[] = {1, 1, 3, 1, 3};
  auto G = Graph{edge_list,
                 edge_list + sizeof(edge_list) / sizeof(std::pair<int, int>),
                 weights, 4};

  auto pmap = arlib::multi_predecessor_map<Vertex>{};
  for (auto [v_it, v_end] = vertices(G); v_it != v_end; ++v_it) {
    for (auto [in_it, in_end] = in_edges(*v_it, G); in_it != in_end; ++in_it) {
      get(pmap, *v_it).push_back(source(*in_it, G));
    }
  }

  auto paths = arlib::to_paths(pmap, G, 0lu, 3lu);

  REQUIRE(paths.size() == 3);

  auto lengths = std::vector<Length>{2, 5, 6};
  for (auto l : lengths) {
    REQUIRE(std::any_of(paths.begin(), paths.end(),
                        [l](auto const &p) { return p.length() == l; }));
  }
}
