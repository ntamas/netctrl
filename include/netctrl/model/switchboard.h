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
     * Old, unused implementation of \ref calculate().
     *
     * This implementation explicitly constructs the line graph, which is
     * prohibitive for large networks. Also, we do not need the line graph
     * because we can find the driver nodes just by looking at the original
     * network.
     */
    void calculateOld();

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

}       // end of namespace

#endif  // NETCTRL_MODEL_SWITCHBOARD_H

