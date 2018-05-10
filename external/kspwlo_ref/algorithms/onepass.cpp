/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#include "kspwlo.hpp"

/*
 *
 *	onepass(RoadNetwork*, NodeID, NodeID, int, double)
 *	-----
 *	Implementation of the OnePass algorithm.
 *
 */

vector<Path> onepass(RoadNetwork *rN, NodeID source, NodeID target,
                     unsigned int k, double theta) {
  vector<Path> resPaths;

  unsigned int count = 0;
  NodeID resid;
  PriorityQueueAS2 queue;
  int newLength = 0;
  vector<double> newOverlap;
  EdgeList::iterator iterAdj;
  Edge edge;
  bool check;

  unordered_map<Edge, vector<int>, boost::hash<Edge>> resEdges;
  unordered_map<Edge, vector<int>, boost::hash<Edge>>::iterator iterE;
  vector<OlLabel *> allCreatedLabels;

  pair<Path, vector<int>> resDijkstra =
      dijkstra_path_and_bounds(rN, source, target);
  Path resNext = resDijkstra.first;

  resPaths.push_back(resNext);
  int newLowerBound = resDijkstra.second[source];

  // Only the shortest path is requested
  if (k == 1)
    return resPaths;

  for (unsigned int j = 0; j < resNext.nodes.size() - 1; j++) {
    edge = make_pair(resNext.nodes[j], resNext.nodes[j + 1]);
    if ((iterE = resEdges.find(edge)) == resEdges.end())
      resEdges.insert(make_pair(edge, vector<int>(1, count)));
    else
      iterE->second.push_back(count);
  }
  count++;

  newOverlap.resize(k, 0);
  queue.push(new OlLabel(source, newLength, newLowerBound, newOverlap, -1));

  while (!queue.empty()) {
    OlLabel *curLabel = static_cast<OlLabel *>(queue.top());
    queue.pop();

    if (curLabel->overlapForK < count - 1) {
      check = true;

      OlLabel *tempLabel = curLabel;

      Path tempPath;
      while (tempLabel != NULL) {
        tempPath.nodes.push_back(tempLabel->node_id);
        tempLabel = static_cast<OlLabel *>(tempLabel->previous);
      }

      reverse(tempPath.nodes.begin(), tempPath.nodes.end());

      for (unsigned int j = 0; j < tempPath.nodes.size() - 1 && check; j++) {
        edge = make_pair(tempPath.nodes[j], tempPath.nodes[j + 1]);
        if ((iterE = resEdges.find(edge)) != resEdges.end()) {
          for (unsigned int i = 0; i < iterE->second.size(); i++) {
            resid = iterE->second[i];

            if (resid > curLabel->overlapForK && resid < count) {
              curLabel->overlapList[resid] +=
                  rN->getEdgeWeight(edge.first, edge.second);
              if (curLabel->overlapList[resid] / resPaths[resid].length >
                  theta) {
                check = false;
                break;
              }
            }
          }
        }
      }
      curLabel->overlapForK = count - 1;
      if (!check)
        continue;
    }

    if (curLabel->node_id == target) { // Found target.

      OlLabel *tempLabel = curLabel;
      Path tempPath;
      while (tempLabel != NULL) {
        tempPath.nodes.push_back(tempLabel->node_id);
        tempLabel = static_cast<OlLabel *>(tempLabel->previous);
      }
      reverse(tempPath.nodes.begin(), tempPath.nodes.end());
      tempPath.length = curLabel->length;
      resPaths.push_back(tempPath);

      if (count == k - 1)
        break;

      for (unsigned int j = 0; j < tempPath.nodes.size() - 1; j++) {
        edge = make_pair(tempPath.nodes[j], tempPath.nodes[j + 1]);
        if ((iterE = resEdges.find(edge)) == resEdges.end())
          resEdges.insert(make_pair(edge, vector<int>(1, count)));
        else
          iterE->second.push_back(count);
      }

      count++;
    } else { // Expand Search
      // For each outgoing edge
      for (iterAdj = rN->adjListOut[curLabel->node_id].begin();
           iterAdj != rN->adjListOut[curLabel->node_id].end(); iterAdj++) {
        // Avoid cycles.
        bool containsLoop = false;
        OlLabel *tempLabel = curLabel;
        while (tempLabel != NULL) {
          if (tempLabel->node_id == iterAdj->first) {
            containsLoop = true;
            break;
          }
          tempLabel = static_cast<OlLabel *>(tempLabel->previous);
        }
        if (!containsLoop) {

          newLength = curLabel->length + iterAdj->second;
          ;
          newOverlap = curLabel->overlapList;
          newLowerBound = newLength + resDijkstra.second[iterAdj->first];
          OlLabel *newPrevious = curLabel;
          edge = make_pair(curLabel->node_id, iterAdj->first);
          check = true;

          if ((iterE = resEdges.find(edge)) != resEdges.end()) {
            for (unsigned int j = 0; j < iterE->second.size(); j++) {
              newOverlap[iterE->second[j]] += iterAdj->second;
              if (newOverlap[iterE->second[j]] /
                      resPaths[iterE->second[j]].length >
                  theta) {
                check = false;
                break;
              }
            }
          }

          if (check) {
            OlLabel *label =
                new OlLabel(iterAdj->first, newLength, newLowerBound,
                            newOverlap, (count - 1), newPrevious);
            queue.push(label);
            allCreatedLabels.push_back(label);
          }
        }
      }
    }
  }

  resEdges.clear();
  for (unsigned int i = 0; i < allCreatedLabels.size(); i++)
    delete allCreatedLabels[i];

  return resPaths;
}
