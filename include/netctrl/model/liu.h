/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef NETCTRL_MODEL_LIU_H
#define NETCTRL_MODEL_LIU_H

#include <igraph/cpp/vector_long.h>
#include <netctrl/model/controllability.h>
#include <netctrl/util/directed_matching.h>

namespace netctrl {

/// Controllability model of Liu et al
class LiuControllabilityModel : public ControllabilityModel {
private:
    /// The list of driver nodes that was calculated
    igraph::Vector m_driverNodes;

    /// The matching that corresponds to the current driver node configuration
    DirectedMatching m_matching;

    /// The list of control paths that was calculated
    std::vector<ControlPath*> m_controlPaths;

public:
    /// Constructs a model that will operate on the given graph
    LiuControllabilityModel(igraph::Graph* pGraph = 0, igraph::Vector* pTargets = 0)
        : ControllabilityModel(pGraph, pTargets), m_driverNodes(), m_matching(),
        m_controlPaths() {
    }

    /// Destroys the model
    virtual ~LiuControllabilityModel();

    virtual void calculate();
    virtual ControllabilityModel* clone();
    virtual float controllability() const;
    virtual std::vector<ControlPath*> controlPaths() const;
    virtual igraph::Vector driverNodes() const;
    virtual std::vector<EdgeClass> edgeClasses() const;

    DirectedMatching* matching();
    const DirectedMatching* matching() const;

    virtual void setGraph(igraph::Graph* graph);
    virtual bool supportsEdgeClasses() const;

protected:
    /// Processes the current matching of the model and calculates control paths.
    /**
     * This function assumes that \c m_matching is already set to a valid
     * matching.
     */
    void calculateControlPaths();

    /// Calculates a matching that matches the given target nodes to controllers.
    DirectedMatching calculateTargetedMatching(igraph::Vector* pTargets);

    /// Calculates a matching that matches all the nodes to controllers.
    DirectedMatching calculateUntargetedMatching();

    /// Removes all the control paths from the previous run (if any)
    void clearControlPaths();

    /// Constructs an undirected bipartite graph on which the matching will be searched.
    /**
     * \param  pTargets   pointer to a vector containing the list of target nodes
     *                    to be controlled, or \c NULL if all the nodes are to be
     *                    controlled
     * \param  pMapping   pointer to a vector that will contain a mapping from
     *                    the node indices of the bipartite graph back to the
     *                    node indices of the original graph. \c NULL is
     *                    allowed; in this case the mapping will not be
     *                    calculated
     */
    igraph::Graph constructBipartiteGraph(const igraph::Vector* pTargets=0,
            igraph::Vector* pMapping=0) const;

    /// Constructs a directed bipartite graph from the given matching.
    /**
     * \param  matching  the matching that defines the edge directions in the
     *                   bipartite graph. Matched edges are oriented from top
     *                   to bottom and the rest will be oriented from bottom to
     *                   top.
     */
    igraph::Graph constructDirectedBipartiteGraphFromMatching(
            const DirectedMatching& matching) const;
};

/// Control path that represents a stem
class Stem : public ControlPath {
public:
    /// Creates an empty stem
    Stem() : ControlPath() {}

    /// Creates a stem with a single node
    explicit Stem(long int node) : ControlPath(node) {}

    /// Creates a stem with the given nodes
    explicit Stem(const igraph::Vector& nodes) : ControlPath(nodes) {}

    /// Creates another stem that is semantically identical to the current stem
    Stem* clone() const {
        return new Stem(m_nodes);
    }

    /// Returns the edges involved in the stem
    virtual igraph::Vector edges(const igraph::Graph& graph) const;

    /// Returns a user-friendly name for the control path type
    virtual std::string name() const {
        return "stem";
    }

    /// Returns \c true since each stem needs an independent input signal
    virtual bool needsInputSignal() const {
        return true;
    }

    /// Returns the root of the stem (i.e. the first vertex)
    long int root() const {
        return m_nodes.front();
    }

    /// Returns the tip of the stem (i.e. the last vertex)
    long int tip() const {
        return m_nodes.back();
    }

    /// Returns a string representation of the stem
    virtual std::string toString() const;
};

/// Control path that represents a bud
class Bud : public ControlPath {
protected:
    /// The stem this bud is attached to.
    /**
     * When this is null, it means that the bud is attached to an input node
     * directly.
     */
    const Stem* m_pStem;

public:
    /// Creates an empty bud
    Bud() : ControlPath(), m_pStem(0) {}

    /// Creates a bud with the given nodes
    explicit Bud(const igraph::Vector& nodes, const Stem* pStem = 0)
        : ControlPath(nodes), m_pStem(pStem) {}

    /// Returns the edges involved in the bud
    virtual igraph::Vector edges(const igraph::Graph& graph) const;

    /// Returns a user-friendly name for the control path type
    virtual std::string name() const {
        return "bud";
    }

    /// Returns \c false since buds do not need independent input signals
    virtual bool needsInputSignal() const {
        return false;
    }

    /// Attaches the bud to a stem
    void setStem(Stem* pStem) {
        m_pStem = pStem;
    }

    /// Returns the stem the bud is attached to
    const Stem* stem() const {
        return m_pStem;
    }

    /// Returns a string representation of the bud
    virtual std::string toString() const;
};

}       // end of namespace

#endif  // NETCTRL_MODEL_LIU_H

