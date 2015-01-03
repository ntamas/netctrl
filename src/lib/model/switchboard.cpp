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
    Vector inDegrees, outDegrees;
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

	// Declare some more variables that we will need.
    VectorBool edgeUsed(m_pGraph->ecount());
	std::auto_ptr<ControlPath> path;

    // Start stems from each divergent node until there are no more divergent
	// nodes. We are lucky here because m_driverNodes contains all the divergent
	// nodes by now -- the only catch is that the last few nodes in m_driverNodes
	// are balanced, but we simply skip those for the time being.
    for (Vector::const_iterator it = m_driverNodes.begin(); it != m_driverNodes.end(); it++) {
		// While the node is divergent...
        while (outDegrees[*it] > inDegrees[*it]) {
            // Select an arbitrary outgoing edge and follow it until we get stuck,
			// then store it as a closed or open walk
			path = createControlPathFromNode(*it, edgeUsed, outDegrees, inDegrees);
			m_controlPaths.push_back(path.release());
        }
    }

	// At this point, all the nodes are balanced (w.r.t. their remaining
	// degrees), so we can form closed walks from them without watching their
	// degrees.
    for (i = 0; i < n; i++) {
		// While the node still has any outbound edges left...
        while (outDegrees[i] > 0) {
            // Select an arbitrary outgoing edge and follow it until we get stuck,
			// then store it as a closed or open walk
			path = createControlPathFromNode(i, edgeUsed, outDegrees, inDegrees);
			m_controlPaths.push_back(path.release());
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
    SwitchboardControllabilityModel* result =
        new SwitchboardControllabilityModel(m_pGraph);
    result->setControllabilityMeasure(this->controllabilityMeasure());
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
                if ((*it)->needsInputSignal()) {
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

std::auto_ptr<ControlPath> SwitchboardControllabilityModel::createControlPathFromNode(
		long int start, VectorBool& edgeUsed, Vector& outDegrees, Vector& inDegrees) const {
	long int v, w;
	Vector walk, incs;
	ControlPath* path;

	v = start;
	while (v != -1) {
		// Find an outbound edge that has not been used yet
		w = -1;
		m_pGraph->incident(&incs, v, IGRAPH_OUT);
		for (Vector::const_iterator it2 = incs.begin(); it2 != incs.end(); it2++) {
			if (!edgeUsed[*it2]) {
				w = *it2;
				break;
			}
		}

		// Did we get stuck? If so, break out of the loop.
		if (w == -1) {
			break;
		}

		// Add v to the walk
		walk.push_back(v);

		// Mark edge w as used and update v to the node edge w is pointing to.
		// Also update the degree vectors
		edgeUsed[w] = true;
		outDegrees[v]--;
		v = IGRAPH_TO(m_pGraph->c_graph(), w);
		inDegrees[v]--;
	}

	// Add v to the walk unless it is equal to the starting point (in which case
	// we have a closed walk)
	if (v != start) {
		walk.push_back(v);
		path = new OpenWalk(walk);
	} else if (walk.size() == 0) {
		// There were no available outbound edges from the start node so we just
		// return NULL
		path = NULL;
	} else {
		// This is a closed walk.
		path = new ClosedWalk(walk);
	}

	return std::auto_ptr<ControlPath>(path);
}

Vector SwitchboardControllabilityModel::driverNodes() const {
    return m_driverNodes;
}

Vector SwitchboardControllabilityModel::changesInDriverNodesAfterEdgeRemoval() const {
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

void SwitchboardControllabilityModel::setGraph(Graph* graph) {
    ControllabilityModel::setGraph(graph);
    m_driverNodes.clear();
    clearControlPaths();
}


/*************************************************************************/


Vector OpenWalk::edges(const Graph& graph) const {
    Vector result;
    Vector::const_iterator it = m_nodes.begin(), it2 = it+1;
    Vector::const_iterator end = m_nodes.end();

	while (it2 != end) {
        result.push_back(graph.getEid(*it, *it2));
        it++; it2++;
    }

    return result;
}

std::string OpenWalk::toString() const {
	std::ostringstream oss;

	oss << "Open walk:";
	for (Vector::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}

	return oss.str();
}


Vector ClosedWalk::edges(const Graph& graph) const {
    Vector result;

    if (m_nodes.size() == 0)
        return result;
    if (m_nodes.size() == 1) {
        long int eid = graph.getEid(m_nodes.front(), m_nodes.front());
        if (eid >= 0)
            result.push_back(eid);
        return result;
    }

    Vector::const_iterator it = m_nodes.begin(), it2 = it+1;
    Vector::const_iterator end = m_nodes.end();

	while (it2 != end) {
        result.push_back(graph.getEid(*it, *it2));
        it++; it2++;
    }
    result.push_back(graph.getEid(*it, m_nodes.front()));

    return result;
}

std::string ClosedWalk::toString() const {
	std::ostringstream oss;

	oss << "Closed walk:";
	for (Vector::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}

	return oss.str();
}

}        // end of namespaces

