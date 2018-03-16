#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include <vector>
#include <cassert>

#include "../model/graph.hpp"

/**
 * @brief      Builds an Alternative Graph (AG) out of a vector of Paths.
 *						 Basically, an AG is the union of several paths from source
 *						 to target.
 *	
 * @param[in]  paths  A vector of s-t-paths.
 * @param      g      The original graph G.
 *
 * @return     The AG.
 */
RoadNetwork build_AG(const std::vector<Path>& paths, RoadNetwork& g) {
	RoadNetwork ag{};

	ag.adjListInc.reserve(g.numNodes);
	ag.adjListOut.reserve(g.numNodes);

	std::cerr << "paths size: " << paths.size() << ", first path size = " << paths[0].edges.size() << "\n";

	for (auto& path : paths) {
		for (auto& edge : path.edges) {
			auto lnode = edge.first;
			auto rnode = edge.second;
			auto w = g.getEdgeWeight(lnode, rnode);

			std::cerr << "\n\nHEYYYYYY BUILD_AG\n\n" << "lnode = " << lnode << ", rnode = " << rnode << ", w = " << w << "\n";

			ag.adjListOut[lnode].insert(std::make_pair(rnode, w));
			ag.adjListInc[rnode].insert(std::make_pair(lnode, w));

			ag.numNodes += 1;
			ag.numEdges += 2;
		}
	}

	std::cerr << "\n\nHEYYYYYY BUILD_AG\n\n" <<
   ag.numNodes << " " << ag.numEdges << " " << ag.adjListOut.size() << "\n";

	return ag;
}

#endif