#include <arlib/graph_utils.hpp>

#include <string>
namespace arlib {
template <typename Graph> std::string dump_edges_weight(const Graph &G) {
  using namespace boost;
  auto ss = std::stringstream{};
  auto weight = get(edge_weight, G);

  for (auto ei = edges(G).first; ei != edges(G).second; ++ei) {
    auto w = weight[*ei];

    ss << "(" << source(*ei, G) << ", " << target(*ei, G) << "; " << w << ") ";
  }

  return ss.str();
}
} // namespace arlib