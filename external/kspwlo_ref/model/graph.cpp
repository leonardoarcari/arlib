/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#include "graph.hpp"

#include <iostream>

RoadNetwork::RoadNetwork(const char *filename) {

  FILE *fp;
  NodeID lnode, rnode, tmp;
  int w;
  char c;

  fp = fopen(filename, "r");
  fscanf(fp, "%c\n", &c);
  fscanf(fp, "%u %u\n", &this->numNodes, &this->numEdges);
  this->adjListOut = vector<EdgeList>(this->numNodes);
  this->adjListInc = vector<EdgeList>(this->numNodes);

  while (fscanf(fp, "%u %u %d %u\n", &lnode, &rnode, &w, &tmp) != EOF) {
    this->adjListOut[lnode].insert(make_pair(rnode, w));
    this->adjListInc[rnode].insert(make_pair(lnode, w));
  }
  fclose(fp);
}

int RoadNetwork::getEdgeWeight(NodeID lnode, NodeID rnode) {
  return this->adjListOut[lnode][rnode];
}

RoadNetwork::~RoadNetwork() {
  // this->adjListOut.clear();
  // this->adjListInc.clear();
}

bool operator==(const Edge &le, const Edge &re) {
  return (le.first == re.first && le.second == re.second) ||
         (le.second == re.first && le.first == re.second);
}

bool Path::containsEdge(Edge e) {
  bool res = false;

  for (unsigned int i = 0; i < this->nodes.size() - 1; i++) {
    if (this->nodes[i] == e.first && this->nodes[i + 1] == e.second) {
      res = true;
      break;
    }
  }

  return res;
}

double Path::overlap_ratio(RoadNetwork *rN, Path &path2) {
  double sharedLength = 0;

  for (unsigned int i = 0; i < path2.nodes.size() - 1; i++) {
    Edge e = make_pair(path2.nodes[i], path2.nodes[i + 1]);
    if (this->containsEdge(e))
      sharedLength += rN->getEdgeWeight(path2.nodes[i], path2.nodes[i + 1]);
  }

  return sharedLength / path2.length;
}

ostream &operator<<(ostream &os, const Path &path) {
  os << "(n_nodes: " << path.nodes.size() << " n_edges: " << path.edges.size()
     << " length: " << path.length << ')';
  return os;
}

vector<Edge> Path::getEdges() const {
  auto edges = vector<Edge>(nodes.size() - 1);

  for (decltype(nodes.size()) i = 0; i < nodes.size() - 1; ++i) {
    edges.emplace_back(nodes[i], nodes[i + 1]);
  }

  return edges;
}
