/* vim:set ts=4 sw=4 sts=4 et: */

#include <stdexcept>
#include <netctrl/model/liu.h>
#include <netctrl/model/switchboard.h>

namespace netctrl {

using namespace igraph;

SwitchboardControllabilityModel::~SwitchboardControllabilityModel() {
    clearControlPaths();
}

void SwitchboardControllabilityModel::calculate() {
    throw std::runtime_error("not implemented yet");
}

void SwitchboardControllabilityModel::clearControlPaths() {
    for (std::vector<ControlPath*>::const_iterator it = m_controlPaths.begin();
            it != m_controlPaths.end(); it++) {
        delete *it;
    }
}

igraph::Vector SwitchboardControllabilityModel::getDriverNodes() const {
    return m_driverNodes;
}

std::vector<ControlPath*> SwitchboardControllabilityModel::getControlPaths() const {
    return m_controlPaths;
}

}        // end of namespaces

