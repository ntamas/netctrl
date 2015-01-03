/* vim:set ts=4 sw=4 sts=4 et: */

#include <netctrl/model/controllability.h>

namespace netctrl {

igraph::Vector ControllabilityModel::changesInDriverNodesAfterEdgeRemoval() const {
    return igraph::Vector();
}

void ControllabilityModel::checkParameters() const {
}

std::vector<EdgeClass> ControllabilityModel::edgeClasses() const {
    return std::vector<EdgeClass>();
}

std::string edgeClassToString(EdgeClass klass) {
    switch (klass) {
        case EDGE_ORDINARY:
            return "ordinary";
        case EDGE_REDUNDANT:
            return "redundant";
        case EDGE_CRITICAL:
            return "critical";
        case EDGE_DISTINGUISHED:
            return "distinguished";
    }

    return "";
}

}          // end of namespace
