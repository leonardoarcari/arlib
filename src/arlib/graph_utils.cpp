#include "arlib/graph_utils.hpp"

namespace arlib {
CSRGraph read_csr_graph_from_string(const std::string &graph) {
  auto ss = std::stringstream{graph};
  std::string line{};

  // Drop graph type info
  std::getline(ss, line);

  // Get number of nodes and edges
  std::getline(ss, line);
  auto line_s = std::stringstream{line};
  int nb_nodes, nb_edges;
  line_s >> nb_nodes >> nb_edges;

  // Get edges and weights
  using Length = typename arlib::length_of_t<CSRGraph>;
  auto edges = std::vector<std::pair<long unsigned, long unsigned>>{};
  auto weights = std::vector<Length>{};

  long unsigned s, t;
  Length w;
  while (std::getline(ss, line)) {
    line_s.str(line);
    line_s.seekg(std::ios_base::beg);
    line_s >> s >> t >> w;

    edges.emplace_back(s, t);
    weights.push_back(w);
  }

  // Return a CSR Graph from data
  using VertexSize = boost::graph_traits<CSRGraph>::vertices_size_type;
  auto G =
      CSRGraph{boost::edges_are_unsorted_multi_pass, edges.begin(), edges.end(),
               weights.begin(), static_cast<VertexSize>(nb_nodes)};
  return G;
}

std::optional<CSRGraph> read_csr_graph_from_file(const std::string_view path) {
  namespace fs = std::filesystem;
  auto fs_path = fs::path(path);

  if (!fs::is_regular_file(fs_path)) {
    std::cerr << fs_path << " is not a regular file.\n";
    return {};
  }

  if (fs::is_empty(fs_path)) {
    std::cerr << fs_path << " is empty.\n";
    return {};
  }

  auto buffer = std::stringstream{};
  auto input = std::ifstream{fs_path.string()};
  buffer << input.rdbuf();

  return {read_csr_graph_from_string(buffer.str())};
}
} // namespace arlib