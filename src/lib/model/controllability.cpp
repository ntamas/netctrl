/* vim:set ts=4 sw=4 sts=4 et: */

#include <sstream>
#include <netctrl/model/controllability.h>

namespace netctrl {

igraph::Vector ControllabilityModel::changesInDriverNodesAfterEdgeRemoval() const {
    return igraph::Vector();
}

std::string Stem::toString() const {
	std::ostringstream oss;

	oss << "Stem:";
	for (igraph::Vector::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}

	return oss.str();
}

std::string Bud::toString() const {
	std::ostringstream oss;

	oss << "Bud: ";
	for (igraph::Vector::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}
    if (stem() != 0) {
        oss << " (assigned to " << stem()->toString() << ")";
    }

	return oss.str();
}

}          // end of namespace
