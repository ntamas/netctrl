/* vim:set ts=4 sw=4 sts=4 et: */

#include <algorithm>
#include <deque>
#include <sstream>
#include <igraph/cpp/edge_iterator.h>
#include <igraph/cpp/graph.h>
#include <igraph/cpp/matching.h>
#include <igraph/cpp/vector_bool.h>
#include <igraph/cpp/vector_int.h>
#include <igraph/cpp/analysis/components.h>
#include <igraph/cpp/analysis/non_simple.h>
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
    Graph bipartiteGraph = this->constructBipartiteGraph(false);

    // Construct the type vector
    long int i = 0, n = m_pGraph->vcount(), u;
    VectorBool types(2 * n);
    for (i = 0; i < n; i++) {
        types[i] = 1;
    }
    // Calculate the maximum bipartite matching
    VectorInt matching;
    maximum_bipartite_matching(bipartiteGraph, types, 0, 0, &matching, 0, 0);
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
    for (VectorInt::const_iterator it = m_driverNodes.begin(); it != m_driverNodes.end(); it++) {
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
        for (VectorInt::const_iterator it = bud->nodes().begin(), end = bud->nodes().end();
                it != end && bud->stem() == 0; it++) {
            VectorInt neis = m_pGraph->neighbors(*it, IGRAPH_IN);
            for (VectorInt::const_iterator it2 = neis.begin(); it2 != neis.end(); it2++) {
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

ControllabilityModel* LiuControllabilityModel::clone() {
    ControllabilityModel* result = new LiuControllabilityModel(m_pGraph);
    return result;
}

Graph LiuControllabilityModel::constructBipartiteGraph(bool directed) const {
    long int i, n = m_pGraph->vcount(), m = m_pGraph->ecount() * 2, u, v;
    Graph bipartiteGraph(2*n, directed);

    VectorInt edges = m_pGraph->getEdgelist();

    if (directed) {
        for (i = 0; i < m; i += 2) {
            u = edges[i]; v = edges[i+1];
            if (m_matching.matchOut(u) == v) {
                edges[i] = v;
                edges[i+1] = u+n;
            } else {
                edges[i] = u+n;
            }
        }
    } else {
        for (i = 0; i < m; i += 2) {
            edges[i] += n;
        }
    }
    bipartiteGraph.addEdges(edges);

    if (!m_pGraph->isDirected()) {
        if (directed) {
            for (i = 0; i < m; i += 2) {
                if (edges[i] >= n) {
                    edges[i] -= n;
                    edges[i+1] += n;
                } else {
                    edges[i] += n;
                    edges[i+1] -= n;
                }
            }
        } else {
            for (i = 0; i < m; i += 2) {
                edges[i] -= n;
                edges[i+1] += n;
            }
        }
        bipartiteGraph.addEdges(edges);
    }

    return bipartiteGraph;
}

float LiuControllabilityModel::controllability() const {
    return m_driverNodes.size() / static_cast<float>(m_pGraph->vcount());
}

std::vector<ControlPath*> LiuControllabilityModel::controlPaths() const {
    return m_controlPaths;
}

igraph::VectorInt LiuControllabilityModel::driverNodes() const {
    return m_driverNodes;
}

std::vector<EdgeClass> LiuControllabilityModel::edgeClasses() const {
    integer_t from, to, i, n = m_pGraph->vcount(), m = m_pGraph->ecount();
    std::vector<EdgeClass> result(m);
    std::deque<long int> queue;

    // The algorithm implemented here is adapted from Algorithm 2 of the
    // following publication:
    //
    // Regin JC: A filtering algorithm for constraints of difference in CSPs.
    // In: AAAI '94 Proceedings of the 12th national conference on Artificial
    // intelligence (vol. 1), pp. 362-367, 1994.

    // (1) Initially, all the edges are REDUNDANT
    std::fill(result.begin(), result.end(), EDGE_REDUNDANT);

    // (2) Construct the directed bipartite graph where matched edges are
    //     directed from top to bottom and unmatched edges are directed
    //     from bottom to top
    Graph bipartiteGraph = this->constructBipartiteGraph(true);

    // (3a) Start a backward BFS from unmatched nodes, mark all traversed edges
    // as ORDINARY
    VectorBool seen(2*n);
    for (from = 0; from < n; from++) {
        if (!m_matching.isMatched(from)) {
            queue.push_back(from);
            seen[from] = true;
        }
        if (!m_matching.isMatching(from)) {
            queue.push_back(from+n);
            seen[from+n] = true;
        }
    }
    while (!queue.empty()) {
        to = queue.front(); queue.pop_front();

        igraph::VectorInt edges = bipartiteGraph.incident(to, IGRAPH_IN);
        for (igraph::VectorInt::const_iterator it = edges.begin(); it != edges.end(); ++it) {
            long int eid = *it;
            if (eid >= m)    // needed for undirected graphs only
                eid -= m;
            result[eid] = EDGE_ORDINARY;
            bipartiteGraph.edge(*it, &from, &to);
            if (!seen[from]) {
                seen[from] = true;
                queue.push_back(from);
            }
        }
    }
    // (3b) Start a forward BFS
    seen.fill(false);
    for (from = 0; from < n; from++) {
        if (!m_matching.isMatched(from)) {
            queue.push_back(from);
            seen[from] = true;
        }
        if (!m_matching.isMatching(from)) {
            queue.push_back(from+n);
            seen[from+n] = true;
        }
    }
    while (!queue.empty()) {
        from = queue.front(); queue.pop_front();

        igraph::VectorInt edges = bipartiteGraph.incident(from, IGRAPH_OUT);
        for (igraph::VectorInt::const_iterator it = edges.begin(); it != edges.end(); ++it) {
            long int eid = *it;
            if (eid >= m)    // needed for undirected graphs only
                eid -= m;
            result[eid] = EDGE_ORDINARY;
            bipartiteGraph.edge(*it, &from, &to);
            if (!seen[to]) {
                seen[to] = true;
                queue.push_back(to);
            }
        }
    }

    // (4) Compute the strongly connected components of the bipartite
    //     directed graph, mark all edges inside the same component
    //     as ORDINARY
    VectorInt membership(2*n);
    connected_components(bipartiteGraph, &membership, 0, 0, IGRAPH_STRONG);
    for (i = 0; i < m; i++) {
        bipartiteGraph.edge(i, &from, &to);
        if (membership[from] == membership[to]) {
            result[i] = EDGE_ORDINARY;
        }
    }

    // (5) For all edges in the matching: if they are still REDUNDANT,
    //     then they should become CRITICAL
    for (from = 0; from < n; from++) {
        to = m_matching.matchOut(from);
        if (to < 0)
            continue;

        i = m_pGraph->getEid(from, to);
        if (result[i] == EDGE_REDUNDANT) {
            result[i] = EDGE_CRITICAL;
        }
    }

    return result;
}

const DirectedMatching* LiuControllabilityModel::matching() const {
    return &m_matching;
}

DirectedMatching* LiuControllabilityModel::matching() {
    return &m_matching;
}

void LiuControllabilityModel::setGraph(igraph::Graph* graph) {
    ControllabilityModel::setGraph(graph);
    m_driverNodes.clear();
    clearControlPaths();
}


/*************************************************************************/


igraph::VectorInt Stem::edges(const igraph::Graph& graph) const {
    igraph::VectorInt result;
    igraph::VectorInt::const_iterator it = m_nodes.begin(), it2 = it+1;
    igraph::VectorInt::const_iterator end = m_nodes.end();

	while (it2 != end) {
        result.push_back(graph.getEid(*it, *it2));
        it++; it2++;
    }

    return result;
}

std::string Stem::toString() const {
	std::ostringstream oss;

	oss << "Stem:";
	for (igraph::VectorInt::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}

	return oss.str();
}

igraph::VectorInt Bud::edges(const igraph::Graph& graph) const {
    igraph::VectorInt result;

    if (m_nodes.size() == 0)
        return result;
    if (m_nodes.size() == 1) {
        long int eid = graph.getEid(m_nodes.front(), m_nodes.front());
        if (eid >= 0)
            result.push_back(eid);
        return result;
    }

    igraph::VectorInt::const_iterator it = m_nodes.begin(), it2 = it+1;
    igraph::VectorInt::const_iterator end = m_nodes.end();

	while (it2 != end) {
        result.push_back(graph.getEid(*it, *it2));
        it++; it2++;
    }
    result.push_back(graph.getEid(*it, m_nodes.front()));

    return result;
}

std::string Bud::toString() const {
	std::ostringstream oss;

	oss << "Bud:";
	for (igraph::VectorInt::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}
    if (stem() != 0) {
        oss << " (assigned to " << stem()->toString() << ")";
    }

	return oss.str();
}

}          // end of namespace
