/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <fstream>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include <boost/functional/hash.hpp>

using namespace std;

typedef unsigned int NodeID;
typedef pair<NodeID, NodeID> Edge;

// the value 'int' is the edge weight
typedef unordered_map<NodeID, int> EdgeList;

class RoadNetwork {
public:
  unsigned int numNodes;
  unsigned int numEdges;
  vector<EdgeList> adjListOut;
  vector<EdgeList> adjListInc;

  RoadNetwork(const char *filename);
  int getEdgeWeight(NodeID lnode, NodeID rnode);
  void print();
  RoadNetwork(){};
  ~RoadNetwork();

  EdgeList ougoingEdgesOf(NodeID);
  EdgeList incomingEdgesOf(NodeID);
};

// This is to ensure that edges are considered in a bidirectional fashion for
// the computation of the overlap.
bool operator==(const Edge &le, const Edge &re);

class Path {
public:
  vector<NodeID> nodes;
  unordered_set<Edge, boost::hash<Edge>> edges;
  int length;

  Path() { length = -1; }

  bool containsEdge(Edge e);
  double overlap_ratio(RoadNetwork *rN, Path &path2);
  friend ostream &operator<<(ostream &os, const Path &path);

  vector<Edge> getEdges() const;
};

#endif