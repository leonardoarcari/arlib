#include "kspwlo/onepass_plus.hpp"
#include "kspwlo/graph_types.hpp"
#include "kspwlo/graph_utils.hpp"

#include <boost/program_options.hpp>

#include <experimental/filesystem>
#include <optional>
#include <string>

#include <margot.hpp>

namespace po = boost::program_options;
namespace fs = std::experimental::filesystem;

struct opplus_options {
  fs::path graph_file;
  int source;
  int destination;
  int k;
  double theta;
};

opplus_options parse_program_options(int argc, char *argv[]);

int main(int argc, char *argv[]) {
  auto options = parse_program_options(argc, argv);

  auto G_opt =
      boost::read_graph_from_file<kspwlo::Graph>(options.graph_file.string());
  if (G_opt) {
    auto &G = G_opt.value();

    // Unpack parameters
    kspwlo::Vertex s = options.source;
    kspwlo::Vertex t = options.destination;
    int k = options.k;
    double theta = options.theta;

    // Run OnePass+
    auto res_paths = boost::onepass_plus(G, s, t, k, theta);
  } else {
    std::cout << "Unable to read graph from " << options.graph_file << "\n";
    return 1;
  }

  return 0;
}

opplus_options parse_program_options(int argc, char *argv[]) {
  try {
    po::options_description desc("OnePass+ program options");
    desc.add_options()("help", "Print this help message")(
        "graph-file,f", po::value<std::string>(),
        "The .gr graph description file")("source,S", po::value<int>(),
                                          "The source node index")(
        "destination,D", po::value<int>(), "The destination node index")(
        "k-paths,k", po::value<int>(), "The number k of alternative paths")(
        "similarity-threshold,s", po::value<double>(),
        "The similarity threshold");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    auto gr_file = fs::path{};
    if (vm.count("graph-file")) {
      gr_file = fs::path(vm["graph-file"].as<std::string>());
      if (!fs::is_regular_file(gr_file)) {
        std::cout << gr_file << " is not a regular file";
        exit(1);
      }
    } else {
      std::cout << "Missing argument: --graph-file\n";
      exit(1);
    }

    int source = -1;
    if (vm.count("source")) {
      source = vm["source"].as<int>();
      if (source < 0) {
        std::cout << source << " is not a valid node index\n";
        exit(1);
      }
    } else {
      std::cout << "Missing argument: --source\n";
      exit(1);
    }

    int destination = -1;
    if (vm.count("destination")) {
      destination = vm["destination"].as<int>();
      if (destination < 0) {
        std::cout << destination << " is not a valid node index\n";
        exit(1);
      }
    } else {
      std::cout << "Missing argument: --destination\n";
      exit(1);
    }

    int k_paths = 0;
    if (vm.count("k-paths")) {
      k_paths = vm["k-paths"].as<int>();
      if (k_paths < 1) {
        std::cout << "Wrong argument: --kpaths=" << k_paths
                  << ". Must be greater than 0\n";
        exit(1);
      }
    } else {
      std::cout << "Missing argument: --k-paths\n";
      exit(1);
    }

    double theta = -1.0;
    if (vm.count("similarity-threshold")) {
      theta = vm["similarity-threshold"].as<double>();
      if (theta < 0.0 || theta > 1.0) {
        std::cout << "Wrong argument: --similarity-threshold=" << theta
                  << ". Must be within interval [0.0, 1.0]\n";
        exit(1);
      }
    } else {
      std::cout << "Missing argument: --similarity-threshold\n";
      exit(1);
    }

    return {gr_file, source, destination, k_paths, theta};

  } catch (std::exception &e) {
    std::cout << e.what() << "\n";
    exit(1);
  }
}