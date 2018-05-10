#include "kspwlo/error_metrics.hpp"
#include "kspwlo/esx.hpp"
#include "kspwlo/graph_pruning.hpp"
#include "kspwlo/graph_types.hpp"
#include "kspwlo/graph_utils.hpp"
#include "kspwlo/onepass_plus.hpp"
#include "kspwlo/penalty.hpp"

#include <boost/program_options.hpp>

#include <experimental/filesystem>
#include <limits>
#include <optional>
#include <string>

#include <margot.hpp>

namespace po = boost::program_options;
namespace fs = std::experimental::filesystem;

enum class Algorithm { invalid = 0, opplus, esx, penalty };

struct opplus_options {
  fs::path graph_file;
  Algorithm algorithm;
  int source;
  int destination;
  int k;
  double theta;
  bool apply_pruning;
  double tau;
};

std::vector<kspwlo::Path<kspwlo::Graph>>
run_kspwlo(kspwlo::Graph &G, kspwlo::Vertex s, kspwlo::Vertex t, int k,
           double theta, Algorithm algorithm, bool apply_pruning, double tau);
opplus_options parse_program_options(int argc, char *argv[]);

int main(int argc, char *argv[]) {
  auto options = parse_program_options(argc, argv);
  margot::init();

  auto G_opt =
      boost::read_graph_from_file<kspwlo::Graph>(options.graph_file.string());
  if (G_opt) {
    auto &G = G_opt.value();

    // Unpack parameters
    Algorithm algo = options.algorithm;
    kspwlo::Vertex s = options.source;
    kspwlo::Vertex t = options.destination;
    int k = options.k;
    double theta = options.theta;
    bool apply_pruning = options.apply_pruning;
    double tau = options.tau;

    // Run kSPwLO algorithm
    margot::parameter_space_exploration::start_monitor();
    auto res_paths = run_kspwlo(G, s, t, k, theta, algo, apply_pruning, tau);
    margot::parameter_space_exploration::stop_monitor();

    // Compute AG error metrics
    auto errors = margot::compute_errors(res_paths, G, s, t);

    float total_distance = errors.total_distance;
    float average_distance = errors.average_distance;
    float decision_edges = errors.decision_edges;

    margot::parameter_space_exploration::monitor::total_distance_monitor.push(
        total_distance);
    margot::parameter_space_exploration::monitor::average_distance_monitor.push(
        average_distance);
    margot::parameter_space_exploration::monitor::decision_edges_monitor.push(
        decision_edges);
    margot::parameter_space_exploration::log();

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
        "The .gr graph description file")(
        "algorithm,a", po::value<std::string>(),
        "The kSPwLO algorithm to use: [opplus|esx]")(
        "source,S", po::value<int>(), "The source node index")(
        "destination,D", po::value<int>(), "The destination node index")(
        "k-paths,k", po::value<int>(), "The number k of alternative paths")(
        "similarity-threshold,s", po::value<double>(),
        "The similarity threshold")(
        "prune,p", po::value<double>(),
        "Apply Uninformed Bidirectional Pruning (tau = arg).");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // Print help message.
    if (vm.count("help")) {
      std::cout << desc << "\n";
      exit(1);
    }

    auto gr_file = fs::path{};
    if (vm.count("graph-file")) {
      gr_file = fs::path(vm["graph-file"].as<std::string>());
      if (!fs::is_regular_file(gr_file)) {
        std::cout << gr_file << " is not a regular file";
        exit(1);
      }
    } else {
      std::cout << "Missing argument: --graph-file\n" << desc << "\n";
      exit(1);
    }

    Algorithm algo = Algorithm::invalid;
    if (vm.count("algorithm")) {
      auto algo_s = vm["algorithm"].as<std::string>();
      if (algo_s == "opplus") {
        algo = Algorithm::opplus;
      } else if (algo_s == "esx") {
        algo = Algorithm::esx;
      } else if (algo_s == "penalty") {
        algo = Algorithm::penalty;
      } else {
        std::cout << algo_s << " is not a valid algorithm name\n";
        exit(1);
      }
    } else {
      std::cout << "Missing argument: --algorithm\n" << desc << "\n";
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
      std::cout << "Missing argument: --source\n" << desc << "\n";
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
      std::cout << "Missing argument: --destination\n" << desc << "\n";
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
      std::cout << "Missing argument: --k-paths\n" << desc << "\n";
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
      std::cout << "Missing argument: --similarity-threshold\n" << desc << "\n";
      exit(1);
    }

    bool apply_pruning = false;
    double tau = 1.0;
    if (vm.count("prune")) {
      apply_pruning = true;
      tau = vm["prune"].as<double>();
      if (tau < 0) {
        apply_pruning = false;
      }
    }

    return {gr_file, algo,  source,        destination,
            k_paths, theta, apply_pruning, tau};

  } catch (std::exception &e) {
    std::cout << e.what() << "\n";
    exit(1);
  }
}

std::vector<kspwlo::Path<kspwlo::Graph>>
run_kspwlo(kspwlo::Graph &G, kspwlo::Vertex s, kspwlo::Vertex t, int k,
           double theta, Algorithm algorithm, bool apply_pruning, double tau) {
  auto &G_prime = G;
  if (apply_pruning) {
    auto G_pruned = boost::uninformed_bidirectional_pruner(G, s, t, tau);
    G_prime = G_pruned;
  }

  switch (algorithm) {
  case Algorithm::opplus:
    return boost::onepass_plus(G_prime, s, t, k, theta);
  case Algorithm::esx:
    return boost::esx(G_prime, s, t, k, theta);
  case Algorithm::penalty:
    return boost::penalty_ag(G_prime, s, t, k, theta, 0.1, 0.1, 100, 1000);
  case Algorithm::invalid:
  default:
    std::cout << "Invalid algorithm. Exiting...\n";
    exit(1);
  }
}