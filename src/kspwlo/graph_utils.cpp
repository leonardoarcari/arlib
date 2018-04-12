#include "kspwlo/graph_utils.hpp"

namespace boost {
std::string dump_edges_weight(const kspwlo::Graph &G) {
  auto ss = std::stringstream{};
  auto weight = get(edge_weight, G);

  for (auto ei = edges(G).first; ei != edges(G).second; ++ei) {
    auto w = weight[*ei];

    ss << "(" << source(*ei, G) << ", " << target(*ei, G) << "; " << w << ") ";
  }

  return ss.str();
}
} // namespace boost