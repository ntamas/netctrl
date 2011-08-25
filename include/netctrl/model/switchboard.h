/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef NETCTRL_MODEL_SWITCHBOARD_H
#define NETCTRL_MODEL_SWITCHBOARD_H

#include <netctrl/model/controllability.h>

namespace netctrl {

/// Switchboard controllability model
class SwitchboardControllabilityModel : public ControllabilityModel {
private:
    /// The list of driver nodes that was calculated
    igraph::Vector m_driverNodes;

    /// The list of control paths that was calculated
    std::vector<ControlPath*> m_controlPaths;

public:
    /// Constructs a model that will operate on the given graph
    SwitchboardControllabilityModel(igraph::Graph* pGraph = 0)
        : ControllabilityModel(pGraph), m_driverNodes(), m_controlPaths() {
    }

    /// Destroys the model
    virtual ~SwitchboardControllabilityModel();

    virtual void calculate();
    igraph::Vector changesInDriverNodesAfterEdgeRemoval() const;
    virtual ControllabilityModel* clone();
    virtual std::vector<ControlPath*> controlPaths() const;
    virtual igraph::Vector driverNodes() const;
    virtual std::vector<EdgeClass> edgeClasses() const;
    virtual void setGraph(igraph::Graph* graph);

protected:
    /// Removes all the control paths from the previous run (if any)
    void clearControlPaths();

private:
    /**
     * Checks whether the given vertex v is part of a non-trivial
     * balanced component.
     */
    bool isInBalancedComponent(long int v, const igraph::Vector& degreeDiffs) const;

    /**
     * Checks whether the given vertex v will be part of a non-trivial
     * balanced component after removing its edge to vertex u.
     */
    bool isInBalancedComponentExcept(long int v, long int u,
            const igraph::Vector& degreeDiffs) const;
};

/// Control path that represents a directed open walk
class OpenWalk : public ControlPath {
public:
    /// Creates an empty open walk
    OpenWalk() : ControlPath() {}

    /// Creates an open walk with the given nodes
    explicit OpenWalk(const igraph::Vector& nodes) : ControlPath(nodes) {}

    /// Returns the edges involved in the open walk
    virtual igraph::Vector edges(const igraph::Graph& graph) const;

    /// Returns a string representation of the open walk
    virtual std::string toString() const;
};

/// Control path that represents a closed walk
class ClosedWalk : public ControlPath {
public:
    /// Creates a closed walk
    ClosedWalk() : ControlPath() {}

    /// Creates a closed walk with the given nodes
    explicit ClosedWalk(const igraph::Vector& nodes) : ControlPath(nodes) {}

    /// Returns the edges involved in the closed walk
    virtual igraph::Vector edges(const igraph::Graph& graph) const;

    /// Returns a string representation of the closed walk
    virtual std::string toString() const;
};

}       // end of namespace

#endif  // NETCTRL_MODEL_SWITCHBOARD_H

