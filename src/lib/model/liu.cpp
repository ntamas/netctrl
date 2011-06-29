/* vim:set ts=4 sw=4 sts=4 et: */

#include <stdexcept>
#include <igraph/cpp/edge_iterator.h>
#include <igraph/cpp/graph.h>
#include <igraph/cpp/matching.h>
#include <igraph/cpp/vector_bool.h>
#include <igraph/cpp/vector_long.h>
#include <netctrl/model/liu.h>

namespace netctrl {

using namespace igraph;

void LiuControllabilityModel::calculate() {
    // Check if we have a graph
    if (m_pGraph == 0)
        throw std::runtime_error("m_pGraph must not be null");

    // Construct the bipartite graph on which we are going to work
    // TODO: handle undirected graphs!
    long int i = 0, n = m_pGraph->vcount(), m = m_pGraph->ecount();
    Vector edges = m_pGraph->getEdgelist();
    Graph bipartiteGraph(2 * n);
    VectorBool types(2 * n);
    for (i = 0; i < n; i++) {
        types[i] = 1;
    }
    for (i = 0; i < 2*m; i += 2) {
        edges[i] += n;
    }
    bipartiteGraph.addEdges(edges);

    // Calculate the maximum bipartite matching
    VectorLong matching;
    maximum_bipartite_matching(bipartiteGraph, types, 0, 0, &matching, 0);
    matching.remove_section(0, n);

    // Create the list of driver nodes
    m_driverNodes.clear();
    for (i = 0; i < n; i++) {
        if (matching[i] == -1)
            m_driverNodes.push_back(i);
    }

    if (m_driverNodes.empty()) {
        m_driverNodes.push_back(0);
    }
}

igraph::Vector LiuControllabilityModel::getDriverNodes() const {
    return m_driverNodes;
}

std::vector<ControlPath*> LiuControllabilityModel::getControlPaths() const {
    throw std::runtime_error("not implemented yet");
}

}          // end of namespace
