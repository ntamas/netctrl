/* vim:set ts=4 sw=4 sts=4 et: */

#include <sstream>
#include <netctrl/model/controllability.h>

namespace netctrl {

igraph::Vector ControllabilityModel::changesInDriverNodesAfterEdgeRemoval() const {
    return igraph::Vector();
}

std::vector<EdgeClass> ControllabilityModel::edgeClasses() const {
    return std::vector<EdgeClass>();
}

igraph::Vector Stem::edges(const igraph::Graph& graph) const {
    igraph::Vector result;
    igraph::Vector::const_iterator it = m_nodes.begin(), it2 = it+1;
    igraph::Vector::const_iterator end = m_nodes.end();

	while (it2 != end) {
        result.push_back(graph.get_eid(*it, *it2));
        it++; it2++;
    }

    return result;
}

std::string Stem::toString() const {
	std::ostringstream oss;

	oss << "Stem:";
	for (igraph::Vector::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++) {
		oss << ' ' << *it;
	}

	return oss.str();
}

igraph::Vector Bud::edges(const igraph::Graph& graph) const {
    igraph::Vector result;

    if (m_nodes.size() == 0)
        return result;
    if (m_nodes.size() == 1) {
        long int eid = graph.get_eid(m_nodes.front(), m_nodes.front());
        if (eid >= 0)
            result.push_back(eid);
        return result;
    }

    igraph::Vector::const_iterator it = m_nodes.begin(), it2 = it+1;
    igraph::Vector::const_iterator end = m_nodes.end();

	while (it2 != end) {
        result.push_back(graph.get_eid(*it, *it2));
        it++; it2++;
    }
    result.push_back(graph.get_eid(*it, m_nodes.front()));

    return result;
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
