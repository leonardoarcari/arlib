/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#include "kspwlo.hpp"

typedef pair<Label *, Label *> SvpLabel;

class SvpLabelComparator {
  bool reverse;

public:
  SvpLabelComparator(const bool &revparam = false) { reverse = revparam; }
  bool operator()(const SvpLabel lsl, const SvpLabel rsl) const {
    return lsl.first->length + lsl.second->length >
           rsl.first->length + rsl.second->length;
  }
};

typedef priority_queue<SvpLabel, std::vector<SvpLabel>, SvpLabelComparator>
    SvpLabelQueue;

/*
 *
 * svp_plus(RoadNetwork*, int, int, int, double)
 * -----
 * Implementation of the SVP+ algorithm.
 *
 */

vector<Path> svp_plus(RoadNetwork *rN, NodeID source, NodeID target,
                      unsigned int k, double theta) {
  vector<Path> resPathsFinal;
  SvpLabelQueue svpQueue;
  vector<pair<Label *, Label *>> svpLabels(rN->numNodes);

  vector<Path> resPaths(rN->numNodes, Path());
  vector<Path> resSimplePathsOrdered;
  vector<int> resLabels(rN->numNodes, 0);

  PriorityQueue queue;
  unsigned int newLength = 0;
  EdgeList::iterator iterAdj;
  vector<NodeID> distancesF(rN->numNodes, INT_MAX);
  vector<NodeID> distancesB(rN->numNodes, INT_MAX);

  Label *tempLabel = NULL;
  distancesF[source] = 0;
  vector<Label *> allCreatedLabels;
  Label *srcLabel = new Label(source, newLength);
  queue.push(srcLabel);
  svpLabels[source].first =
      new Label(srcLabel->node_id, srcLabel->length, srcLabel->previous);
  allCreatedLabels.push_back(svpLabels[source].first);
  allCreatedLabels.push_back(srcLabel);
  unsigned int nodeCount = 0;

  while (!queue.empty()) {
    Label *curLabel = queue.top();
    queue.pop();

    if (resPaths[curLabel->node_id].length != -1)
      continue;

    resPaths[curLabel->node_id].length = curLabel->length;
    resLabels[curLabel->node_id] = curLabel->length;
    distancesF[curLabel->node_id] = curLabel->length;

    svpLabels[curLabel->node_id].first =
        new Label(curLabel->node_id, curLabel->length, curLabel->previous);
    allCreatedLabels.push_back(svpLabels[curLabel->node_id].first);

    nodeCount++;
    if (nodeCount == rN->numNodes)
      break;

    else { // Expand search
      // For each outgoing edge.
      for (iterAdj = rN->adjListOut[curLabel->node_id].begin();
           iterAdj != rN->adjListOut[curLabel->node_id].end(); iterAdj++) {
        newLength = curLabel->length + iterAdj->second;
        Label *newPrevious = curLabel;
        if (distancesF[iterAdj->first] > newLength) {
          Label *label = new Label(iterAdj->first, newLength, newPrevious);
          allCreatedLabels.push_back(label);
          queue.push(label);
        }
      }
    }
  }

  queue = PriorityQueue();
  assert(queue.size() == 0);

  vector<bool> visited(rN->numNodes, false);
  tempLabel = NULL;
  vector<NodeID> tempVetices;
  nodeCount = 0;
  distancesB[target] = 0;
  Label *trgLabel = new Label(target, 0);
  queue.push(trgLabel);
  svpLabels[target].second =
      new Label(trgLabel->node_id, trgLabel->length, trgLabel->previous);
  allCreatedLabels.push_back(svpLabels[target].second);
  allCreatedLabels.push_back(trgLabel);

  while (!queue.empty()) {
    Label *curLabel = queue.top();
    queue.pop();

    if (visited[curLabel->node_id])
      continue;

    visited[curLabel->node_id] = true;
    distancesB[curLabel->node_id] = curLabel->length;
    svpLabels[curLabel->node_id].second =
        new Label(curLabel->node_id, curLabel->length, curLabel->previous);
    allCreatedLabels.push_back(svpLabels[curLabel->node_id].second);

    nodeCount++;
    if (nodeCount == rN->numNodes)
      break;

    else { // Expand search
      // For each outgoing edge.
      for (iterAdj = rN->adjListInc[curLabel->node_id].begin();
           iterAdj != rN->adjListInc[curLabel->node_id].end(); iterAdj++) {
        newLength = curLabel->length + iterAdj->second;
        Label *newPrevious = curLabel;
        if (distancesB[iterAdj->first] > newLength) {
          Label *label = new Label(iterAdj->first, newLength, newPrevious);
          allCreatedLabels.push_back(label);
          queue.push(label);
        }
      }
    }
  }
  // sort(resSimplePathsOrdered.begin(),resSimplePathsOrdered.end());
  for (unsigned int i = 0; i < rN->numNodes; i++) {
    if (svpLabels[i].first == NULL || svpLabels[i].second == NULL)
      continue;
    svpQueue.push(svpLabels[i]);
  }

  // Adding shortest path to the result set
  Path sp;
  sp.length = svpQueue.top().first->length + svpQueue.top().second->length;
  tempLabel = NULL;
  tempLabel = svpQueue.top().first;
  while (tempLabel != NULL) {
    sp.nodes.push_back(tempLabel->node_id);
    tempLabel = tempLabel->previous;
  }
  reverse(sp.nodes.begin(), sp.nodes.end());
  tempLabel = svpQueue.top().second->previous;
  while (tempLabel != NULL) {
    sp.nodes.push_back(tempLabel->node_id);
    tempLabel = tempLabel->previous;
  }
  resPathsFinal.push_back(sp);
  svpQueue.pop();

  while (resPathsFinal.size() < k && !svpQueue.empty()) {
    Path tempP;
    SvpLabel svpLabelCurrent = svpQueue.top();

    tempP.length =
        svpLabelCurrent.first->length + svpLabelCurrent.second->length;
    tempLabel = NULL;
    tempLabel = svpLabelCurrent.first;
    while (tempLabel != NULL) {
      tempP.nodes.push_back(tempLabel->node_id);
      tempLabel = tempLabel->previous;
    }
    reverse(tempP.nodes.begin(), tempP.nodes.end());
    tempLabel = svpLabelCurrent.second->previous;
    unsigned int index = tempP.nodes.size() - 1;
    bool check = true;
    while (tempLabel != NULL) {
      for (int i = index; i >= 0; i--) {
        if (tempP.nodes[i] == tempLabel->node_id) {
          check = false;
          break;
        }
      }
      if (check)
        tempP.nodes.push_back(tempLabel->node_id);
      else
        break;
      tempLabel = tempLabel->previous;
    }
    svpQueue.pop();

    if (!check)
      continue;

    for (unsigned int k = 0; k < resPathsFinal.size(); k++) {
      if (tempP.overlap_ratio(rN, resPathsFinal[k]) > theta) {
        check = false;
        break;
      }
    }
    if (check) {
      resPathsFinal.push_back(tempP);
      if (resPathsFinal.size() == k)
        break;
    }
  }
  for (unsigned int i = 0; i < allCreatedLabels.size(); i++)
    delete allCreatedLabels[i];
  return resPathsFinal;
}
