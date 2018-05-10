/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#include "tools.hpp"

/*
 *
 *	astar_limited(RoadNetwork, NodeID, NodeID, vector<int>,
 *unordered_set<Edge,boost::hash<Edge>>)
 *	-----
 *	This algorithm works in a similar fashion with A*. The difference is
 *	that this algorithm avoids expanding the provided deleted edges.
 *  Therefore, the resulting path is not the shortest path, but the shortest
 *	path which at the same time avoids the deleted edges. This function
 *	is used by ESX
 *
 */

Path astar_limited(RoadNetwork *rN, NodeID source, NodeID target,
                   vector<int> &bounds,
                   unordered_set<Edge, boost::hash<Edge>> &deletedEdges) {
  PriorityQueueAS queue;
  Path resPath;
  int newLength = 0;
  EdgeList::iterator iterAdj;
  vector<int> distances(rN->numNodes, INT_MAX);
  vector<bool> visited(rN->numNodes);
  Label *targetLabel = NULL;
  distances[source] = 0;
  vector<Label *> allCreatedLabels;

  int newLowerBound = bounds[source];
  Label *srcLabel = new Label(source, newLength, newLowerBound);
  queue.push(srcLabel);
  allCreatedLabels.push_back(srcLabel);
  int pops = 0;
  while (!queue.empty()) {
    Label *curLabel = queue.top();
    queue.pop();
    pops++;
    if (visited[curLabel->node_id])
      continue;

    visited[curLabel->node_id] = true;
    distances[curLabel->node_id] = curLabel->length;

    if (curLabel->node_id == target) { // Destination has been found
      targetLabel = curLabel;
      resPath.length = curLabel->length;
      break;
    } else { // Expand search
      for (iterAdj = rN->adjListOut[curLabel->node_id].begin();
           iterAdj != rN->adjListOut[curLabel->node_id].end(); iterAdj++) {
        newLength = curLabel->length + iterAdj->second;
        newLowerBound = newLength + bounds[iterAdj->first];
        Label *newPrevious = curLabel;
        Edge e(make_pair(curLabel->node_id, iterAdj->first));

        if (deletedEdges.find(e) != deletedEdges.end())
          continue;

        if (distances[iterAdj->first] > newLength) {
          Label *label =
              new Label(iterAdj->first, newLength, newLowerBound, newPrevious);
          allCreatedLabels.push_back(label);
          queue.push(label);
        }
      }
    }
  }
  while (targetLabel != NULL) {
    resPath.nodes.push_back(targetLabel->node_id);
    targetLabel = targetLabel->previous;
  }
  reverse(resPath.nodes.begin(), resPath.nodes.end());

  distances.clear();
  visited.clear();
  for (unsigned int i = 0; i < allCreatedLabels.size(); i++)
    delete allCreatedLabels[i];

  return resPath;
}
