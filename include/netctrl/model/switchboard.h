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
    virtual igraph::Vector getDriverNodes() const;
    virtual std::vector<ControlPath*> getControlPaths() const;

protected:
    /// Removes all the control paths from the previous run (if any)
    void clearControlPaths();
};

}       // end of namespace

#endif  // NETCTRL_MODEL_SWITCHBOARD_H

