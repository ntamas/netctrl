/* vim:set ts=4 sw=4 sts=4 et: */

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <netctrl/util/directed_matching.h>

namespace netctrl {

DirectedMatching::DirectedMatching(const igraph::VectorLong& mapping) {
	long int i, n;
	igraph::VectorLong::const_iterator it, end;

    n = mapping.size();
    m_inMapping = mapping;

    end = m_inMapping.end();
    for (i = 0, it = m_inMapping.begin(); it != end; i++, it++) {
        if (*it == -1)
            continue;
        m_outMapping[*it].push_back(i);
    }
}

const igraph::VectorLong* DirectedMatching::matchOut(long int u) const {
    OutMapping::const_iterator it = m_outMapping.find(u);
    const igraph::VectorLong* result = NULL;

    if (it != m_outMapping.end()) {
        result = &it->second;
    }

    return (result && result->empty()) ? NULL : result;
}

void DirectedMatching::setMatch(long int u, long int v) {
    if (v == -1 || u == -1 || m_inMapping[v] == u)
        return;

    unmatch(v);

    m_inMapping[v] = u;
    m_outMapping[u].push_back(v);
}

void DirectedMatching::unmatch(long int v) {
    long int u;
    long int index;

    if (v == -1)
        return;
    
    u = m_inMapping[v];
    if (u == -1)
        return;

    m_inMapping[v] = -1;
    m_outMapping[u].search(0, v, &index);
    m_outMapping[u].remove(index);
}

}          // end of namespace
