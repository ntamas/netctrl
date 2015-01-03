/* vim:set ts=4 sw=4 sts=4 et: */

#include <cstdio>
#include <algorithm>
#include <deque>
#include <set>
#include <sstream>
#include <igraph/cpp/edge_iterator.h>
#include <igraph/cpp/edge_selector.h>
#include <igraph/cpp/graph.h>
#include <igraph/cpp/matching.h>
#include <igraph/cpp/vector_bool.h>
#include <igraph/cpp/vector_long.h>
#include <igraph/cpp/analysis/components.h>
#include <igraph/cpp/analysis/non_simple.h>
#include <netctrl/model/liu.h>

namespace netctrl {

using namespace igraph;

/**
 * \brief Structure that encapsulates a bipartite graph on which the mapping
 * will be calculated as well as some extra information.
 */
typedef struct {
    /// The graph on which the mapping will be calculated
    Graph graph;

    /**
     * \brief Mapping from node indices in the graph to the node indices in the
     *        \em original graph from which the controllability graph was
     *        created.
     *
     * If this vector is empty, it means that the mapping is an identity
     * mapping.
     */
    Vector mapping;

    /** \brief Number of nodes in the graph that correspond to target nodes of the
     * original graph. By convention, the target nodes are always the last ones
     * in the controllability graph.
     */
    long int numTargets;

    /**
     * Returns the type vector of the controllability graph.
     */
    VectorBool getTypeVector() const {
        long int i, n = graph.vcount();

        // Construct the type vector
        VectorBool types(2 * n);
        types.fill(1);
        for (i = n - numTargets; i < n; i++) {
            types[i] = 0;
        }

        return types;
    }

    /**
     * Maps a vertex index of the controllability graph back to the original
     * graph.
     */
    long int mapToOriginalVertexIndex(long int index) const {
        return mapping[index];
    }

} ControllabilityGraph;

LiuControllabilityModel::~LiuControllabilityModel() {
    clearControlPaths();
}

void LiuControllabilityModel::calculate() {
    // Check if we have a graph
    if (m_pGraph == 0)
        throw std::runtime_error("m_pGraph must not be null");

    // Calculate the matching on the graph
    if (m_pTargets != 0) {
        m_matching = calculateTargetedMatching(m_pTargets);
    } else {
        m_matching = calculateUntargetedMatching();
    }

    // Process the calculated matching and creates control paths
    calculateControlPaths();
}

DirectedMatching LiuControllabilityModel::calculateTargetedMatching(
        Vector* pTargets) {
    long int i, n = m_pGraph->vcount(), u, v, iter;
    ControllabilityGraph ctrl;
    Vector currentTargets(*pTargets);
    DirectedMatching result(n);

    iter = 0;
    while (currentTargets.size() > 0) {
        printf("Iteration %ld; targets = ", iter);
        currentTargets.print();
    
        // Construct the bipartite graph on which we are going to work
        ctrl.graph = this->constructBipartiteGraph(&currentTargets, &ctrl.mapping);
        ctrl.numTargets = currentTargets.size();

        // Does the graph have a "left" side?
        if (ctrl.graph.vcount() == ctrl.numTargets) {
            break;
        }

        // Calculate the maximum bipartite matching and find the new targets
        VectorLong matching;
        maximum_bipartite_matching(ctrl.graph, ctrl.getTypeVector(), 0, 0, &matching, 0, 0);
        n = ctrl.graph.vcount();
        currentTargets.clear();
        for (i = n - ctrl.numTargets; i < n; i++) {
            v = ctrl.mapToOriginalVertexIndex(i);
            if (matching[i] == -1) {
                // Temporarily match node to itself so it will appear as
                // matched in subsequent runs. This is required for the
                // algorithm to terminate properly.
                u = v;
            } else {
                u = ctrl.mapToOriginalVertexIndex(matching[i]);
                if (!result.isMatched(u)) {
                    currentTargets.push_back(u);
                }
                // TODO: make sure that currentTargets is unique
            }
            result.setMatch(u, v);
            printf("Matching vertex %ld to %ld\n", u, v);
        }
        iter++;
    }

    // Find all the nodes that have been matched to itself and unmatch
    // them so they become stems
    for (i = 0; i < n; i++) {
        if (result.matchIn(i) == i) {
            result.unmatch(i);
        }
    }

    return result;
}

DirectedMatching LiuControllabilityModel::calculateUntargetedMatching() {
    ControllabilityGraph ctrl;
    long int i, n = m_pGraph->vcount();

    // Construct the bipartite graph on which we are going to work
    ctrl.graph = this->constructBipartiteGraph(0, &ctrl.mapping);
    ctrl.numTargets = m_pGraph->vcount();

    // Calculate the maximum bipartite matching
    VectorLong matching;
    maximum_bipartite_matching(ctrl.graph, ctrl.getTypeVector(), 0, 0, &matching, 0, 0);

    for (i = 0; i < n; i++) {
        if (matching[i] != -1) {
            matching[i] = ctrl.mapToOriginalVertexIndex(matching[i]);
        }
    }
    matching.resize(n);

    return DirectedMatching(matching);
}

void LiuControllabilityModel::clearControlPaths() {
    for (std::vector<ControlPath*>::const_iterator it = m_controlPaths.begin();
            it != m_controlPaths.end(); it++) {
        delete *it;
    }
}

ControllabilityModel* LiuControllabilityModel::clone() {
    ControllabilityModel* result = new LiuControllabilityModel(m_pGraph, m_pTargets);
    return result;
}

Graph LiuControllabilityModel::constructBipartiteGraph(const Vector* pTargets,
        Vector* pMapping) const {
    long int i, u, v;
    long int numNodes;
    long int numTargets = pTargets ? pTargets->size() : 0;
    long int edgeListLength;
    Vector edges;
    Graph bipartiteGraph;

    numNodes = m_pGraph->vcount();
    edgeListLength = m_pGraph->ecount() * 2;

    if (pTargets == 0) {
        // Simple case: all nodes are target nodes
        bipartiteGraph = Graph(2*numNodes, false);

        edges = m_pGraph->getEdgelist();
        for (i = 0; i < edgeListLength; i += 2) {
            edges[i] += numNodes;
        }
        bipartiteGraph.addEdges(edges);

        if (!m_pGraph->isDirected()) {
            for (i = 0; i < edgeListLength; i += 2) {
                edges[i] -= numNodes;
                edges[i+1] += numNodes;
            }
            bipartiteGraph.addEdges(edges);
        }

        if (pMapping != 0) {
            pMapping->clear();
            pMapping->append(Vector::Seq(0, numNodes-1));
            pMapping->append(Vector::Seq(0, numNodes-1));
        }
    } else {
        // Only some nodes are target nodes
        std::set<long int> predecessorSet;
        std::map<long int, long int> forwardMapping;

        // Find all the edges that are predecessors of targets
        for (i = 0; i < numTargets; i++) {
            v = (*pTargets)[i];
            EdgeSelector predecessors = EdgeSelector::Predecessors(v, m_pGraph);
            EdgeIterator it(predecessors);
            while (!it.end()) {
                u = (*it).source();
                predecessorSet.insert(u);
                edges.push_back(u);
                edges.push_back(-i-1);    // will be remapped later
                ++it;
            }
        }

        // Create the forward mapping
        Vector tmp(predecessorSet.begin(), predecessorSet.end());
        u = tmp.size();
        for (i = 0; i < u; i++) {
            forwardMapping[tmp[i]] = i;
        }
        for (i = 0; i < numTargets; i++) {
            forwardMapping[-i-1] = u+i;
        }
        numNodes = tmp.size() + numTargets;

        // Remap the edge list
        edgeListLength = edges.size();
        for (i = 0; i < edgeListLength; i++) {
            edges[i] = forwardMapping[edges[i]];
        }
        bipartiteGraph = Graph(numNodes, false);
        bipartiteGraph.addEdges(edges);

        // Return the reverse mapping if the user needs it
        if (pMapping != 0) {
            pMapping->resize(numNodes);
            for (std::map<long int, long int>::const_iterator it = forwardMapping.begin();
                    it != forwardMapping.end(); it++) {
                u = it->first; v = it->second;
                if (u < 0) {
                    u = (*pTargets)[-u-1];
                }
                (*pMapping)[v] = u;
            }
        }
    }

    return bipartiteGraph;
}

Graph LiuControllabilityModel::constructDirectedBipartiteGraphFromMatching(
        const DirectedMatching& matching) const {
    long int i, n = m_pGraph->vcount(), m = m_pGraph->ecount() * 2, u, v;
    Graph bipartiteGraph(2*n, true);

    Vector edges = m_pGraph->getEdgelist();

    for (i = 0; i < m; i += 2) {
        u = edges[i]; v = edges[i+1];
        if (matching.matchIn(v) == u) {
            edges[i] = v;
            edges[i+1] = u+n;
        } else {
            edges[i] = u+n;
        }
    }
    bipartiteGraph.addEdges(edges);

    if (!m_pGraph->isDirected()) {
        for (i = 0; i < m; i += 2) {
            if (edges[i] >= n) {
                edges[i] -= n;
                edges[i+1] += n;
            } else {
                edges[i] += n;
                edges[i+1] -= n;
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

igraph::Vector LiuControllabilityModel::driverNodes() const {
    return m_driverNodes;
}

std::vector<EdgeClass> LiuControllabilityModel::edgeClasses() const {
    if (!supportsEdgeClasses()) {
        throw std::runtime_error("edge classification not supported when the "
                "set of target nodes is restricted");
    }

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
    Graph bipartiteGraph = this->constructDirectedBipartiteGraphFromMatching(m_matching);

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

        igraph::Vector edges = bipartiteGraph.incident(to, IGRAPH_IN);
        for (igraph::Vector::const_iterator it = edges.begin(); it != edges.end(); ++it) {
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

        igraph::Vector edges = bipartiteGraph.incident(from, IGRAPH_OUT);
        for (igraph::Vector::const_iterator it = edges.begin(); it != edges.end(); ++it) {
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
    Vector membership(2*n);
    clusters(bipartiteGraph, &membership, 0, 0, IGRAPH_STRONG);
    for (i = 0; i < m; i++) {
        bipartiteGraph.edge(i, &from, &to);
        if (membership[from] == membership[to]) {
            result[i] = EDGE_ORDINARY;
        }
    }

    // (5) For all edges in the matching: if they are still REDUNDANT,
    //     then they should become CRITICAL
    for (to = 0; to < n; to++) {
        from = m_matching.matchIn(to);
        if (from < 0)
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

void LiuControllabilityModel::calculateControlPaths() {
    long int i, n = m_pGraph->vcount(), u, v, w;
    long int m;

    // Create the list of driver nodes
    m_driverNodes.clear();
    if (m_pTargets == 0) {
        // Every node is a potential driver node if it is not matched
        for (i = 0; i < n; i++) {
            if (!m_matching.isMatched(i)) {
                m_driverNodes.push_back(i);
            }
        }
    } else {
        // Start from target nodes and follow the matchings backwards
        // to the drivers
        std::set<long int> driverNodeSet;
        std::set<long int> seen;
        m = m_pTargets->size();
        for (i = 0; i < m; i++) {
            w = v = (*m_pTargets)[i];
            seen.clear();
            seen.insert(v);
            while (true) {
                u = m_matching.matchIn(v);
                if (u == w) {
                    // This is a bud.
                    break;
                } else if (u == -1) {
                    // v is a driver node
                    driverNodeSet.insert(v);
                    break;
                } else if (seen.find(u) != seen.end()) {
                    // This is a stem looping back to itself.
                    m_matching.unmatch(v);
                    driverNodeSet.insert(v);
                    break;
                } else {
                    seen.insert(u);
                    v = u;
                }
            }
        }
        m_driverNodes.update(driverNodeSet.begin(), driverNodeSet.end());
    }

    // Clear the list of control paths
    clearControlPaths();

    // Construct stems from each driver node. At the same time, create a vector that
    // maps vertices to one of the stems they belong to (which will be need to
    // attach buds to stems) and another one that marks vertices that have already
    // been assigned to stems or buds.
    std::vector<Stem*> verticesToStems(n);
    std::deque<Stem*> stemQueue;
    Stem* stem;
    Stem* stemClone;
    VectorBool vertexUsed(n);
    const VectorLong* matchedNodes;

    for (Vector::const_iterator it = m_driverNodes.begin(); it != m_driverNodes.end(); it++) {
        stemQueue.push_back(new Stem(*it));
        while (!stemQueue.empty()) {
            stem = stemQueue.front();
            stemQueue.pop_front();

            u = stem->tip();
            verticesToStems[u] = stem;
            vertexUsed[u] = true;

            matchedNodes = m_matching.matchOut(u);
            if (matchedNodes == NULL) {
                m_controlPaths.push_back(stem);
            } else {
                m = matchedNodes->size();
                for (i = 1; i < m; i++) {
                    stemClone = stem->clone();
                    stemClone->appendNode((*matchedNodes)[i]);
                    stemQueue.push_back(stemClone);
                }
                stem->appendNode((*matchedNodes)[0]);
                stemQueue.push_back(stem);
            }
        }
    }

    // The remaining matched edges form buds
    for (u = 0; u < n; u++) {
        if (vertexUsed[u] || !m_matching.isMatched(u))
            continue;

        Bud* bud = new Bud();
        v = u;
        while (!vertexUsed[v]) {
            bud->appendNode(v);
            vertexUsed[v] = true;
            v = m_matching.matchIn(v);
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

void LiuControllabilityModel::setGraph(igraph::Graph* graph) {
    ControllabilityModel::setGraph(graph);
    m_driverNodes.clear();
    clearControlPaths();
}

bool LiuControllabilityModel::supportsEdgeClasses() const {
    return m_pTargets == 0;
}

/*************************************************************************/


igraph::Vector Stem::edges(const igraph::Graph& graph) const {
    igraph::Vector result;
    igraph::Vector::const_iterator it = m_nodes.begin(), it2 = it+1;
    igraph::Vector::const_iterator end = m_nodes.end();

	while (it2 != end) {
        result.push_back(graph.getEid(*it, *it2));
        it++; it2++;
    }

    return result;
}

std::string Stem::toString() const {
	std::ostringstream oss;

	oss << "Stem:";
	for (igraph::Vector::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}

	return oss.str();
}

igraph::Vector Bud::edges(const igraph::Graph& graph) const {
    igraph::Vector result;

    if (m_nodes.size() == 0)
        return result;
    if (m_nodes.size() == 1) {
        long int eid = graph.getEid(m_nodes.front(), m_nodes.front());
        if (eid >= 0)
            result.push_back(eid);
        return result;
    }

    igraph::Vector::const_iterator it = m_nodes.begin(), it2 = it+1;
    igraph::Vector::const_iterator end = m_nodes.end();

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
	for (igraph::Vector::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}
    if (stem() != 0) {
        oss << " (assigned to " << stem()->toString() << ")";
    }

	return oss.str();
}

}          // end of namespace
