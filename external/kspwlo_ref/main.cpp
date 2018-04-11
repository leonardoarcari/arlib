/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <fstream>
#include <iostream>

#include "kspwlo_ref/algorithms/kspwlo.hpp"
#include "kspwlo_ref/model/graph.hpp"

#include "kspwlo_ref/exploration/error_metrics.hpp"

#include <margot.hpp>

using namespace std;

int main(int argc, char **argv) {
  // Initialize Margot
  margot::init();

  string graphFile = "";
  unsigned int k = 0;
  double theta = -1;
  string algo = "";
  RoadNetwork *rN = 0;

  NodeID source = 0, target = 100;

  int opt = -1;
  while ((opt = getopt(argc, argv, "f:S:D:k:s:a:")) != -1) {
    switch (opt) {
    case 'f':
      graphFile = string(optarg);
      break;
    case 'S':
      source = stoi(string(optarg));
      break;
    case 'D':
      target = stoi(string(optarg));
      break;
    case 'k':
      k = stoi(string(optarg));
      break;
    case 's':
      theta = stof(string(optarg));
      break;
    case 'a':
      algo = string(optarg);
      break;
    }
  }

  // Input checking
  if (graphFile == "") {
    cerr << "Wrong arguments. Define graph file correctly." << endl;
    exit(1);
  }

  if (k < 1) {
    cerr << "Define k between [1,+inf)" << endl;
    exit(2);
  }

  if (theta < 0 || theta > 1) {
    cerr << "Define theta between [0,1]" << endl;
    exit(3);
  }

  if (source == target) {
    cerr << "Source and target are the same node" << endl;
    exit(4);
  }

  // Loading road network
  rN = new RoadNetwork(graphFile.c_str());

  vector<Path> result;

  margot::parameter_space_exploration::start_monitor();
  if (boost::iequals(algo, "op")) {
    result = onepass(rN, source, target, k, theta);
  } else if (boost::iequals(algo, "mp")) {
    result = multipass(rN, source, target, k, theta);
  } else if (boost::iequals(algo, "opplus")) {
    result = onepass_plus(rN, source, target, k, theta);
  } else if (boost::iequals(algo, "svp")) {
    result = svp_plus(rN, source, target, k, theta);
  } else if (boost::iequals(algo, "esx")) {
    result = esx(rN, source, target, k, theta);
  }
  margot::parameter_space_exploration::stop_monitor();

  auto errors = margot::compute_errors(result, *rN, source, target);

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

  cout << source << "\t" << target << "\t[" << result[0].length;
  for (unsigned int j = 1; j < result.size(); j++) {
    cout << "," << result[j].length;
  }
  cout << "]" << endl;

  delete rN;
  return 0;
}
