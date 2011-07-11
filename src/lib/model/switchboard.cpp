/* vim:set ts=4 sw=4 sts=4 et: */

#include <set>
#include <stdexcept>
#include <igraph/cpp/edge.h>
#include <igraph/cpp/generators/line_graph.h>
#include <netctrl/model/liu.h>
#include <netctrl/model/switchboard.h>


namespace netctrl {

using namespace igraph;

namespace {

/// Finds a stem with a given root in a control path vector
Stem* findStemWithRoot(std::vector<ControlPath*>& controlPaths, long int root) {
    std::vector<ControlPath*>::iterator it = controlPaths.begin();
    for (; it != controlPaths.end(); it++) {
        Stem* pStem = dynamic_cast<Stem*>(*it);
        if (pStem == 0)
            continue;
        if (pStem->root() == root)
            return pStem;
    }
    return 0;
}

/// Finds a stem with a given tip in a control path vector
Stem* findStemWithTip(std::vector<ControlPath*>& controlPaths, long int tip) {
    std::vector<ControlPath*>::iterator it = controlPaths.begin();
    for (; it != controlPaths.end(); it++) {
        Stem* pStem = dynamic_cast<Stem*>(*it);
        if (pStem == 0)
            continue;
        if (pStem->tip() == tip)
            return pStem;
    }
    return 0;
}

}


SwitchboardControllabilityModel::~SwitchboardControllabilityModel() {
    clearControlPaths();
}

void SwitchboardControllabilityModel::calculate() {
    // Construct the line graph first
    Graph lg(line_graph(*m_pGraph));

    // Find the driver nodes in the line graph; these are the driver edges
    // in our model
    LiuControllabilityModel model(&lg);
    model.calculate();

    // Get the control paths
    std::vector<ControlPath*> controlPaths = model.controlPaths();
    std::vector<ControlPath*>::iterator it;
    Vector::iterator nodeIt, nodeIt2, nodeIt3;
    DirectedMatching* pMatching = model.matching();

    // Bud pruning. Buds without a stem may cause problems (see below), so
    // we try to circumvent the problem here by converting buds without
    // stems to a stem. This is done by iterating over the vertices of the
    // bud and checking the following:
    //
    // - If a vertex in the bud has a predecessor (in-neighbor) without
    //   an outgoing edge in the present matching, we can break the bud
    //   there and connect it to the predecessor. E.g., for a bud
    //   a -> b -> c -> d and an external vertex v that has no matched
    //   outgoing edge and is connected to b, we can replace the a -> b
    //   edge with v -> b. Then, the node that drives v will be the
    //   driver node of the entire bud (which became a stem).
    //
    // - If a vertex in the bud has a successor (out-neighbor) without
    //   an incoming edge in the present matching (in this case, the
    //   successor is always a driver node), we can break the bud and
    //   connect it to the successor. E.g., a bud a -> b -> c -> d and
    //   an external vertex v s.t. that v has no matched incoming edge
    //   and there exists b -> v, we can replace b -> c with b -> v
    //   and make c -> d -> a -> b -> v -> ... a stem.
    it = controlPaths.begin();
    while (it != controlPaths.end()) {
        // don't cache controlPaths.end() as we will modify it on the
        // fly as we remove resolved buds
        ControlPath* path = *it;
        bool resolved = false;

        Bud* bud = dynamic_cast<Bud*>(path);
        if (bud == 0 /* || bud->stem() != 0 */) {
            it++;
            continue;       // not a bud without stem
        }

        // Iterate over the vertices
        Vector::const_iterator end = bud->nodes().end();
        for (nodeIt = bud->nodes().begin(); nodeIt != end; nodeIt++) {
            long int u = *nodeIt, v;
            Vector neis;

            // Get the predecessors of the node
            neis = lg.neighbors(u, IGRAPH_IN);
            for (nodeIt2 = neis.begin(); nodeIt2 != neis.end(); nodeIt2++) {
                // Does this node have an outgoing matched edge?
                v = *nodeIt2;
                if (pMatching->isMatching(v))
                    continue;

                // Nope, so we can theoretically use it to break the bud.
                // At this point, there *must* be some stem which terminates
                // in v. We look up this stem in the list of stems.
                Stem* stem = findStemWithTip(controlPaths, v);
                assert(stem != 0);

                // Extend the stem with the nodes of the bud and adjust the
                // matching.
                pMatching->setMatch(v, u);
                for (nodeIt3 = nodeIt; nodeIt3 != end; nodeIt3++) {
                    stem->appendNode(*nodeIt3);
                }
                for (nodeIt3 = bud->nodes().begin(); nodeIt3 != nodeIt; nodeIt3++) {
                    stem->appendNode(*nodeIt3);
                }

                // We mark the bud as resolved.
                resolved = true;
            }

            if (!resolved) {
                // Get the successors of the node
                neis = lg.neighbors(*nodeIt, IGRAPH_OUT);
                for (Vector::iterator nodeIt2 = neis.begin();
                        !resolved && nodeIt2 != neis.end(); nodeIt2++) {
                    // Does this node have an incoming matched edge?
                    v = *nodeIt2;
                    if (pMatching->isMatched(v))
                        continue;

                    // Nope, so we can theoretically use it to break the bud.
                    // At this point, there *must* be some stem which starts with
                    // v. We look up this stem in the list of stems.
                    Stem* stem = findStemWithRoot(controlPaths, v);
                    assert(stem != 0);
                    
                    // Extend the stem with the nodes of the bud and adjust the
                    // matching.
                    Vector newNodes;
                    pMatching->setMatch(u, v);
                    nodeIt3 = nodeIt; nodeIt3++;
                    while (nodeIt3 != end) {
                        newNodes.push_back(*nodeIt3);
                        nodeIt3++;
                    }
                    for (nodeIt3 = bud->nodes().begin(); nodeIt3 != nodeIt; nodeIt3++) {
                        newNodes.push_back(*nodeIt3);
                    }
                    newNodes.push_back(u);

                    // We mark the bud as resolved.
                    resolved = true;
                }
            }
        }

        if (resolved) {
            // Remove the bud
            it = controlPaths.erase(it);
        } else {
            // Move to the next element
            it++;
        }
    }

    // Loop through the stems and buds of the line graph. The root of each
    // stem represents an edge in the original graph and the tail of this
    // edge will become a driver node.
    std::set<long int> driverNodeSet;

    for (it = controlPaths.begin(); it != controlPaths.end(); it++) {
        ControlPath* path = *it;
        Stem* stem = dynamic_cast<Stem*>(path);
        if (stem != 0) {
            // Store the source vertex of the root of the stem
            driverNodeSet.insert(m_pGraph->edge(stem->root()).tail());
        }
    }
    
    // Now, check the buds. Each bud that is not attached to a stem in the line
    // graph is problematic. For such buds, we have to iterate over its edges,
    // map the endpoints of the edges back to the original graph, and see if
    // any of these is already a driver node (because of a stem above).
    // If so, we are okay. If not, we have to make one of the nodes a
    // driver node.
    for (std::vector<ControlPath*>::const_iterator it = controlPaths.begin();
            it != controlPaths.end(); it++) {
        ControlPath* path = *it;
        Bud* bud = dynamic_cast<Bud*>(path);
        if (bud != 0 && bud->stem() == 0) {
            Vector::const_iterator end = bud->nodes().end();
            bool foundDriverNode = false;

            for (nodeIt = bud->nodes().begin(); nodeIt != end && !foundDriverNode; nodeIt++) {
                if (driverNodeSet.find(*nodeIt) != driverNodeSet.end())
                    foundDriverNode = true;
            }

            if (!foundDriverNode) {
                // TODO: better strategy; what if there is another node which
                // could control more than one bud? Is it possible?
                long int edgeID = bud->nodes().front();
                driverNodeSet.insert(m_pGraph->edge(edgeID).tail());
            }
        }
    }

    m_driverNodes = Vector(driverNodeSet.begin(), driverNodeSet.end());
}

void SwitchboardControllabilityModel::clearControlPaths() {
    for (std::vector<ControlPath*>::const_iterator it = m_controlPaths.begin();
            it != m_controlPaths.end(); it++) {
        delete *it;
    }
}

std::vector<ControlPath*> SwitchboardControllabilityModel::controlPaths() const {
    return m_controlPaths;
}

igraph::Vector SwitchboardControllabilityModel::driverNodes() const {
    return m_driverNodes;
}

}        // end of namespaces

