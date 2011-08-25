/* vim:set ts=4 sw=4 sts=4 et: */

#include <netctrl/model/controllability.h>

namespace netctrl {

igraph::Vector ControllabilityModel::changesInDriverNodesAfterEdgeRemoval() const {
    return igraph::Vector();
}

std::vector<EdgeClass> ControllabilityModel::edgeClasses() const {
    return std::vector<EdgeClass>();
}

}          // end of namespace
