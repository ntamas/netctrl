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

    /// Returns a vector of control paths after a successful calculation
    /**
     * Pointers returned in this vector are owned by the model; they should
     * \em not be destroyed by the caller.
     */
    virtual std::vector<ControlPath*> controlPaths() const = 0;

    /// Returns the set of driver nodes after a successful calculation
    virtual igraph::Vector driverNodes() const = 0;

};


/// Abstract superclass for control paths (stems and buds)
class ControlPath {
protected:
    igraph::Vector m_nodes;

    /// Creates an empty control path
    ControlPath() : m_nodes() {}

    /// Creates a control path with the given nodes
    explicit ControlPath(const igraph::Vector& nodes) : m_nodes(nodes) {}

public:
    /// Virtual destructor that does nothing
    virtual ~ControlPath() {}

    /// Appends a new node to the control path
    void appendNode(long int node) {
        m_nodes.push_back(node);
    }

    /// Returns the nodes involved in the control path
    igraph::Vector& nodes() {
        return m_nodes;
    }

    /// Returns the nodes involved in the control path (const variant)
    const igraph::Vector& nodes() const {
        return m_nodes;
    }

    /// Returns the number of nodes involved
    size_t size() const {
        return m_nodes.size();
    }
};

/// Control path that represents a stem
class Stem : public ControlPath {
public:
    /// Creates an empty stem
    Stem() : ControlPath() {}

    /// Creates a stem with the given nodes
    explicit Stem(const igraph::Vector& nodes) : ControlPath(nodes) {}

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
    const Stem* m_pStem;

public:
    /// Creates an empty bud
    Bud() : ControlPath(), m_pStem(0) {}

    /// Creates a bud with the given nodes
    explicit Bud(const igraph::Vector& nodes, const Stem* pStem = 0)
        : ControlPath(nodes), m_pStem(pStem) {}

    /// Attaches the bud to a stem
    void setStem(Stem* pStem) {
        m_pStem = pStem;
    }

    /// Returns the stem the bud is attached to
    const Stem* stem() const {
        return m_pStem;
    }
};

}       // end of namespace

#endif  // NETCTRL_MODEL_CONTROLLABILITY_H

