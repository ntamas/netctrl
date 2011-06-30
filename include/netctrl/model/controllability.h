/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef NETCTRL_MODEL_CONTROLLABILITY_H
#define NETCTRL_MODEL_CONTROLLABILITY_H

#include <igraph/cpp/graph.h>
#include <igraph/cpp/vertex_selector.h>

namespace netctrl {

class ControlPath;

/// Abstract superclass for controllability models
class ControllabilityModel {
protected:
    /// The graph on which the controllability model will operate
    igraph::Graph* m_pGraph;

public:
    /// Constructs a controllability model that will operate on the given graph
    ControllabilityModel(igraph::Graph* pGraph = 0) : m_pGraph(pGraph) {
    }

    /// Virtual destructor that does nothing
    virtual ~ControllabilityModel() {}

    /// Calculates the set of driver nodes and control paths
    /**
     * This is an abstract method that must be overridden in subclasses.
     */
    virtual void calculate() = 0;

    /// Returns the set of driver nodes after a successful calculation
    virtual igraph::Vector getDriverNodes() const = 0;

    /// Returns a vector of control paths after a successful calculation
    /**
     * Pointers returned in this vector are owned by the model; they should
     * \em not be destroyed by the caller.
     */
    virtual std::vector<ControlPath*> getControlPaths() const = 0;

};


/// Abstract superclass for control paths (stems and buds)
class ControlPath {
protected:
    igraph::Vector m_nodes;

    /// Creates a control path with the given nodes
    ControlPath(const igraph::Vector& nodes) : m_nodes(nodes) {}

public:
    /// Returns the nodes involved in the control path
    igraph::Vector nodes() const {
        return m_nodes;
    }
};

/// Control path that represents a stem
class Stem : public ControlPath {
public:
    /// Creates a stem with the given nodes
    Stem(const igraph::Vector& nodes) : ControlPath(nodes) {}

    /// Returns the root of the stem (i.e. the first vertex)
    long int root() const {
        return m_nodes.front();
    }

    /// Returns the tip of the stem (i.e. the last vertex)
    long int tip() const {
        return m_nodes.back();
    }
};

/// Control path that represents a bud
class Bud : public ControlPath {
protected:
    /// The stem this bud is attached to.
    /**
     * When this is null, it means that the bud is attached to an input node
     * directly.
     */
    const Stem* m_stem;

public:
    /// Creates a bud with the given nodes
    Bud(const igraph::Vector& nodes, const Stem* stem = 0)
        : ControlPath(nodes), m_stem(stem) {}
};

}       // end of namespace

#endif  // NETCTRL_MODEL_CONTROLLABILITY_H

