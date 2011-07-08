/* vim:set ts=4 sw=4 sts=4 et: */

#include <igraph/cpp/edge_iterator.h>
#include <igraph/cpp/graph.h>
#include <igraph/cpp/matching.h>
#include <igraph/cpp/vector_bool.h>
#include <igraph/cpp/vector_long.h>
#include <netctrl/model/liu.h>

namespace netctrl {

using namespace igraph;

LiuControllabilityModel::~LiuControllabilityModel() {
    clearControlPaths();
}

void LiuControllabilityModel::calculate() {
    // Check if we have a graph
    if (m_pGraph == 0)
        throw std::runtime_error("m_pGraph must not be null");

    // Construct the bipartite graph on which we are going to work
    long int i = 0, n = m_pGraph->vcount(), m = m_pGraph->ecount(), u;
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
    if (!m_pGraph->isDirected()) {
        for (i = 0; i < 2*m; i += 2) {
            edges[i] -= n;
            edges[i+1] += n;
        }
        bipartiteGraph.addEdges(edges);
    }

    // Calculate the maximum bipartite matching
    VectorLong matching;
    maximum_bipartite_matching(bipartiteGraph, types, 0, 0, &matching, 0);
    for (i = 0; i < n; i++) {
        if (matching[i] != -1)
            matching[i] -= n;
    }
    m_matching = DirectedMatching(matching, DirectedMatching::DIRECTION_IN_OUT);

    // Create the list of driver nodes
    m_driverNodes.clear();
    for (i = 0; i < n; i++) {
        if (!m_matching.isMatched(i))
            m_driverNodes.push_back(i);
    }

    // Clear the list of control paths
    clearControlPaths();
    
    // Construct stems from each driver node. At the same time, create a vector that
    // maps vertices to the stems they belong to and another one that marks vertices
    // that have already been assigned to stems or buds.
    std::vector<Stem*> verticesToStems(n);
    VectorBool vertexUsed(n);
    for (Vector::const_iterator it = m_driverNodes.begin(); it != m_driverNodes.end(); it++) {
        Stem* stem = new Stem();

        u = *it;
        while (u != -1) {
            stem->appendNode(u);
            verticesToStems[u] = stem;
            vertexUsed[u] = true;
            u = m_matching.matchOut(u);
        }

        m_controlPaths.push_back(stem);
    }

    // The remaining matched edges form buds
    for (u = 0; u < n; u++) {
        if (vertexUsed[u] || !m_matching.isMatched(u))
            continue;

        Bud* bud = new Bud();
        while (!vertexUsed[u]) {
            bud->appendNode(u);
            vertexUsed[u] = true;
            u = m_matching.matchOut(u);
        }
        if (bud->size() > 1 && bud->nodes().front() == bud->nodes().back()) {
            bud->nodes().pop_back();
        }

        // Check whether we can attach the bud to a stem
        for (Vector::const_iterator it = bud->nodes().begin(), end = bud->nodes().end();
                it != end && bud->stem() == 0; it++) {
            Vector neis = m_pGraph->neighbors(*it, IGRAPH_IN);
            for (Vector::const_iterator it2 = neis.begin(); it2 != neis.end(); it2++) {
                if (verticesToStems[*it2] != 0) {
                    bud->setStem(verticesToStems[*it2]);
                    break;
                }
            }
        }

        m_controlPaths.push_back(bud);
    }

    // Cleanup: if there is no driver node, we must provide at least one
    if (m_driverNodes.empty()) {
        m_driverNodes.push_back(0);
    }
}

void LiuControllabilityModel::clearControlPaths() {
    for (std::vector<ControlPath*>::const_iterator it = m_controlPaths.begin();
            it != m_controlPaths.end(); it++) {
        delete *it;
    }
}

std::vector<ControlPath*> LiuControllabilityModel::controlPaths() const {
    return m_controlPaths;
}

igraph::Vector LiuControllabilityModel::driverNodes() const {
    return m_driverNodes;
}

const DirectedMatching* LiuControllabilityModel::matching() const {
    return &m_matching;
}

DirectedMatching* LiuControllabilityModel::matching() {
    return &m_matching;
}

}          // end of namespace
