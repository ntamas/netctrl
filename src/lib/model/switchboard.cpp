/* vim:set ts=4 sw=4 sts=4 et: */

#include <deque>
#include <set>
#include <sstream>
#include <stdexcept>
#include <igraph/cpp/edge.h>
#include <igraph/cpp/edge_iterator.h>
#include <igraph/cpp/edge_selector.h>
#include <igraph/cpp/vector_bool.h>
#include <igraph/cpp/analysis/components.h>
#include <igraph/cpp/generators/line_graph.h>
#include <netctrl/model/switchboard.h>


namespace netctrl {

using namespace igraph;

SwitchboardControllabilityModel::~SwitchboardControllabilityModel() {
    clearControlPaths();
}

void SwitchboardControllabilityModel::calculate() {
    Vector inDegrees, outDegrees, incs;
    long int i, j, n = m_pGraph->vcount();
    long int balancedCount = 0;

#define IS_BALANCED(i) ((outDegrees[i] == inDegrees[i]) && outDegrees[i] > 0)

    m_driverNodes.clear();
    m_pGraph->degree(&inDegrees,  V(m_pGraph), IGRAPH_IN,  true);
    m_pGraph->degree(&outDegrees, V(m_pGraph), IGRAPH_OUT, true);
    
    // Find divergent nodes, count balanced nodes
    for (i = 0; i < n; i++) {
        if (outDegrees[i] > inDegrees[i])
            m_driverNodes.push_back(i);
        else if (IS_BALANCED(i)) {
            balancedCount++;
        }
    }

    if (balancedCount > 0) {
        // Find the connected components consisting of balanced nodes
        // only.
        Vector membership;
        integer_t cluster_count;
        clusters(*m_pGraph, &membership, 0, &cluster_count, IGRAPH_WEAK);

        VectorBool balancedCluster(cluster_count);
        balancedCluster.fill(true);
        for (i = 0; i < n; i++) {
            if (!IS_BALANCED(i)) {
                balancedCluster[(long int)membership[i]] = false;
            }
        }

        for (i = 0; i < n; i++) {
            j = membership[i];
            if (balancedCluster[j]) {
                m_driverNodes.push_back(i);
                balancedCluster[j] = false;
            }
        }
    }

#undef IS_BALANCED

    // Clear the list of control paths
    clearControlPaths();

    // Start a stem from each driver node found so far. Note that we have to
    // watch out here: the last few items of m_driverNodes might be driver
    // nodes but they do not have corresponding stems since they are in
    // balanced components.
    VectorBool edgeUsed(m_pGraph->ecount());
    for (Vector::const_iterator it = m_driverNodes.begin(); it != m_driverNodes.end(); it++) {
        Vector nodesInPath;
        ControlPath* path;
        long int v, w;
        bool firstPathFromThisNode = true;

        while (firstPathFromThisNode || outDegrees[*it] > inDegrees[*it]) {
            // Select an arbitrary outgoing edge and follow it until we get stuck.
            v = *it;
            nodesInPath.clear();
            while (v != -1) {
                nodesInPath.push_back(v);

                w = -1;
                m_pGraph->incident(&incs, v, IGRAPH_OUT);
                for (Vector::const_iterator it2 = incs.begin(); it2 != incs.end(); it2++) {
                    if (edgeUsed[*it2])
                        continue;
                    w = *it2;
                    break;
                }
                if (w == -1)
                    break;

                edgeUsed[w] = true;
                outDegrees[v]--;
                v = IGRAPH_TO(m_pGraph->c_graph(), w);
                inDegrees[v]--;
            }

            if (v == *it && nodesInPath.size() > 1) {
                nodesInPath.pop_back();
                path = new ClosedWalk(nodesInPath);
            } else {
                path = new OpenWalk(nodesInPath);
            }
            m_controlPaths.push_back(path);
            firstPathFromThisNode = false;
        }
    }
}

void SwitchboardControllabilityModel::clearControlPaths() {
    for (std::vector<ControlPath*>::const_iterator it = m_controlPaths.begin();
            it != m_controlPaths.end(); it++) {
        delete *it;
    }
}

ControllabilityModel* SwitchboardControllabilityModel::clone() {
    ControllabilityModel* result = new SwitchboardControllabilityModel(m_pGraph);
    return result;
}

float SwitchboardControllabilityModel::controllability() const {
    size_t numPaths;

    switch (m_controllabilityMeasure) {
        case NODE_MEASURE:
            return m_driverNodes.size() / static_cast<float>(m_pGraph->vcount());

        case EDGE_MEASURE:
            numPaths = 0;
            for (std::vector<ControlPath*>::const_iterator it = m_controlPaths.begin();
                    it != m_controlPaths.end(); ++it) {
                if (dynamic_cast<OpenWalk*>(*it) != 0) {
                    numPaths++;
                }
            }
            // TODO: add balanced components
            return numPaths / static_cast<float>(m_pGraph->ecount());

        default:
            return 0;
    }
}

SwitchboardControllabilityModel::ControllabilityMeasure
SwitchboardControllabilityModel::controllabilityMeasure() const {
    return m_controllabilityMeasure;
}

std::vector<ControlPath*> SwitchboardControllabilityModel::controlPaths() const {
    return m_controlPaths;
}

igraph::Vector SwitchboardControllabilityModel::driverNodes() const {
    return m_driverNodes;
}

igraph::Vector SwitchboardControllabilityModel::changesInDriverNodesAfterEdgeRemoval() const {
    Vector degreeDiffs, outDegrees;
    Vector result(m_pGraph->ecount());
    EdgeSelector es = E(m_pGraph);
    EdgeIterator eit(es);

    m_pGraph->degree(&outDegrees, V(m_pGraph), IGRAPH_OUT, true);
    m_pGraph->degree(&degreeDiffs,  V(m_pGraph), IGRAPH_IN,  true);
    degreeDiffs -= outDegrees;

    while (!eit.end()) {
        Edge edge = *eit;
        long int i = eit.get();
        long int u = edge.source(), v = edge.destination();

        if (degreeDiffs[u] == -1) {
            /* source vertex will become balanced instead of divergent */
            result[i]--;
        }
        if (degreeDiffs[v] == 0) {
            /* target vertex will become divergent instead of balanced */
            result[i]++;
        }

        /* Treating special cases */
        if (degreeDiffs[u] == 0 && degreeDiffs[v] == 0) {
            /* u and v may potentially have been part of a balanced component.
             * In this case, the component already has a driver node before
             * the removal, so we will have to decrease result[i] by 1 */
            if (isInBalancedComponent(u, degreeDiffs))
                result[i]--;
        }
        if (degreeDiffs[v] == 1) {
            /* v is convergent but will become balanced. If all its neighbors
             * are balanced (except u), we may suspect that it becomes part
             * of a balanced component, which will require one more driver
             * node, so we will have to increase result[i] by 1 */
            degreeDiffs[v]--; degreeDiffs[u]++;
            if (isInBalancedComponentExcept(v, u, degreeDiffs))
                result[i]++;
            degreeDiffs[v]++; degreeDiffs[u]--;
        }
        if (degreeDiffs[u] == -1) {
            /* u is divergent but will become balanced. If all its neighbors
             * are balanced (except v), we may suspect that it becomes part
             * of a balanced component, which will require one more driver
             * node, so we will have to increase result[i] by 1 */
            degreeDiffs[v]--; degreeDiffs[u]++;
            if (isInBalancedComponentExcept(u, v, degreeDiffs))
                result[i]++;
            degreeDiffs[v]++; degreeDiffs[u]--;
        }

        ++eit;
    }

    return result;
}

std::vector<EdgeClass> SwitchboardControllabilityModel::edgeClasses() const {
    Vector diffs = changesInDriverNodesAfterEdgeRemoval();
    size_t i, n = diffs.size();
    std::vector<EdgeClass> result(n);
    for (i = 0; i < n; i++) {
        if (diffs[i] < 0)
            result[i] = EDGE_DISTINGUISHED;
        else if (diffs[i] == 0)
            result[i] = EDGE_REDUNDANT;
        else
            result[i] = EDGE_CRITICAL;
    }
    return result;
}

bool SwitchboardControllabilityModel::isInBalancedComponent(
        long int v, const Vector& degreeDiffs) const {
    return isInBalancedComponentExcept(v, -1, degreeDiffs);
}

bool SwitchboardControllabilityModel::isInBalancedComponentExcept(
        long int v, long int u, const Vector& degreeDiffs) const {
    Vector neis;
    int i, j;
    bool result = true;

    /* Is v balanced? If not, we can return early */
    if (degreeDiffs[v] != 0)
        return false;

    /* Does v have any neighbors apart from u? If not, v is in a
     * _trivial_ balanced component, so we return false */
    neis = m_pGraph->neighbors(v, IGRAPH_ALL);
    if (neis.empty() || (neis.size() == 1 && neis[0] == u))
        return false;

    /* Prepare the queue */
    VectorBool visited(m_pGraph->vcount());
    std::deque<long int> q;
    q.push_back(v); visited[v] = true;
    if (u >= 0)
        visited[u] = true;
    
    while (!q.empty()) {
        v = q.front(); q.pop_front();
        neis = m_pGraph->neighbors(v, IGRAPH_ALL);
        j = neis.size();
        for (i = 0; i < j; i++) {
            u = neis[i];
            if (visited[u])
                continue;
            if (degreeDiffs[u] != 0) {
                result = false;
                q.clear();
                break;
            }
            q.push_back(u);
            visited[u] = true;
        }
    }

    return result;
}

void SwitchboardControllabilityModel::setControllabilityMeasure(
        SwitchboardControllabilityModel::ControllabilityMeasure measure) {
    m_controllabilityMeasure = measure;
}

void SwitchboardControllabilityModel::setGraph(igraph::Graph* graph) {
    ControllabilityModel::setGraph(graph);
    m_driverNodes.clear();
    clearControlPaths();
}


/*************************************************************************/


igraph::Vector OpenWalk::edges(const igraph::Graph& graph) const {
    igraph::Vector result;
    igraph::Vector::const_iterator it = m_nodes.begin(), it2 = it+1;
    igraph::Vector::const_iterator end = m_nodes.end();

	while (it2 != end) {
        result.push_back(graph.get_eid(*it, *it2));
        it++; it2++;
    }

    return result;
}

std::string OpenWalk::toString() const {
	std::ostringstream oss;

	oss << "Open walk:";
	for (igraph::Vector::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}

	return oss.str();
}


igraph::Vector ClosedWalk::edges(const igraph::Graph& graph) const {
    igraph::Vector result;

    if (m_nodes.size() == 0)
        return result;
    if (m_nodes.size() == 1) {
        long int eid = graph.get_eid(m_nodes.front(), m_nodes.front());
        if (eid >= 0)
            result.push_back(eid);
        return result;
    }

    igraph::Vector::const_iterator it = m_nodes.begin(), it2 = it+1;
    igraph::Vector::const_iterator end = m_nodes.end();

	while (it2 != end) {
        result.push_back(graph.get_eid(*it, *it2));
        it++; it2++;
    }
    result.push_back(graph.get_eid(*it, m_nodes.front()));

    return result;
}

std::string ClosedWalk::toString() const {
	std::ostringstream oss;

	oss << "Closed walk:";
	for (igraph::Vector::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}

	return oss.str();
}

}        // end of namespaces

