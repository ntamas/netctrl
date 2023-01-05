/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef NETCTRL_MODEL_CONTROLLABILITY_H
#define NETCTRL_MODEL_CONTROLLABILITY_H

#include <igraph/cpp/graph.h>
#include <igraph/cpp/vertex_selector.h>
#include <string>

namespace netctrl {

class ControlPath;

/// Edge classes in controllability models
typedef enum {
    EDGE_ORDINARY,
    EDGE_REDUNDANT,
    EDGE_CRITICAL,
    EDGE_DISTINGUISHED
} EdgeClass;

/// Function to convert an edge class into its name
std::string edgeClassToString(EdgeClass klass);

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

    /// Creates an exact copy of this model
    virtual ControllabilityModel* clone() = 0;

    /// Calculates the set of driver nodes and control paths
    /**
     * This is an abstract method that must be overridden in subclasses.
     */
    virtual void calculate() = 0;

    /**
     * \brief Returns a vector which shows how would the number of driver nodes
     *        change after the removal of a given edge.
     *
     * This is an abstract method that \em may be overridden in subclasses.
     *
     * \return a vector containing a number for each edge of the graph, showing
     *         the difference in the number of driver nodes before and after the
     *         removal of the edge. Positive numbers indicate edges whose removal
     *         increases the number of driver nodes. The function may also return
     *         an empty vector indicating that such a calculation is not implemented
     *         or not feasible.
     */
    virtual igraph::VectorInt changesInDriverNodesAfterEdgeRemoval() const;

    /// Returns the controllability measure of the model after a successful calculation
    virtual float controllability() const = 0;

    /// Returns a vector of control paths after a successful calculation
    /**
     * Pointers returned in this vector are owned by the model; they should
     * \em not be destroyed by the caller.
     */
    virtual std::vector<ControlPath*> controlPaths() const = 0;

    /// Returns the set of driver nodes after a successful calculation
    virtual igraph::VectorInt driverNodes() const = 0;

    /**
     * \brief Returns a vector that classifies edges into four classes: redundant,
     *        ordinary, critical or distinguished.
     *
     * An edge is redundant if its removal does not change the set of driver nodes
     * in \em any control configuration; critical if its removal always requires
     * us to add at least one extra driver node in \em any control configuration,
     * and \em distinguished if its removal decreases the number of driver nodes.
     * Otherwise it is ordinary.
     *
     * Note that the Liu controllability model contains no distinguished edges,
     * and the switchboard model contains no ordinary edges.
     *
     * \returns  a vector classifying the edges into classes, or an empty vector
     *           if the operation is not implemented for a given model.
     */
    virtual std::vector<EdgeClass> edgeClasses() const;

    /// Returns the graph on which the controllability model will operate
    virtual igraph::Graph* graph() const {
        return m_pGraph;
    }

    /// Sets the graph on which the controllability model will operate
    virtual void setGraph(igraph::Graph* graph) {
        m_pGraph = graph;
    }
};


/// Abstract superclass for control paths (stems and buds)
class ControlPath {
protected:
    igraph::VectorInt m_nodes;

    /// Creates an empty control path
    ControlPath() : m_nodes() {}

    /// Creates a control path with the given nodes
    explicit ControlPath(const igraph::VectorInt& nodes) : m_nodes(nodes) {}

public:
    /// Virtual destructor that does nothing
    virtual ~ControlPath() {}

    /// Appends a new node to the control path
    void appendNode(long int node) {
        m_nodes.push_back(node);
    }

    /// Returns the edges involved in the control path
    virtual igraph::VectorInt edges(const igraph::Graph& graph) const = 0;

    /// Returns a user-friendly name for the control path type
    virtual std::string name() const = 0;

    /// Returns whether the control path needs an independent input signal
    virtual bool needsInputSignal() const = 0;

    /// Returns the nodes involved in the control path
    igraph::VectorInt& nodes() {
        return m_nodes;
    }

    /// Returns the nodes involved in the control path (const variant)
    const igraph::VectorInt& nodes() const {
        return m_nodes;
    }

    /// Prepends a node to the control path
    void prependNode(long int node) {
        m_nodes.insert(0, node);
    }

    /// Returns the number of nodes involved
    size_t size() const {
        return m_nodes.size();
    }

    /// Returns a string representation of the control path
    virtual std::string toString() const = 0;
};

}       // end of namespace

#endif  // NETCTRL_MODEL_CONTROLLABILITY_H

