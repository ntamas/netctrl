/* vim:set ts=4 sw=4 sts=4 et: */

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <netctrl/util/directed_matching.h>

namespace netctrl {


DirectedMatching::DirectedMatching(const igraph::VectorInt& mapping,
		Direction direction) {
	long int i, n;
	igraph::VectorInt::const_iterator it, end;

	switch (direction) {
		case DIRECTION_OUT_IN:
			assert(mapping.size() % 2 == 0);
			n = mapping.size() / 2;
			m_inMapping.resize(n);
			m_outMapping.resize(n);
			std::copy(mapping.begin(), mapping.begin() + n, m_outMapping.begin());
			std::copy(mapping.begin()+n, mapping.end(), m_inMapping.begin());
			break;

		case DIRECTION_IN_OUT:
			assert(mapping.size() % 2 == 0);
			n = mapping.size() / 2;
			m_inMapping.resize(n);
			m_outMapping.resize(n);
			std::copy(mapping.begin(), mapping.begin() + n, m_inMapping.begin());
			std::copy(mapping.begin()+n, mapping.end(), m_outMapping.begin());
			break;

		case DIRECTION_IN:
			n = mapping.size();
			m_inMapping = mapping;
			m_outMapping.resize(n);
			std::fill(m_outMapping.begin(), m_outMapping.end(), -1);

			end = m_inMapping.end();
			for (i = 0, it = m_inMapping.begin(); it != end; i++, it++) {
				if (*it == -1)
					continue;
				m_outMapping[*it] = i;
			}
			break;

		case DIRECTION_OUT:
			n = mapping.size();
			m_outMapping = mapping;
			m_inMapping.resize(n);
			std::fill(m_inMapping.begin(), m_outMapping.end(), -1);

			end = m_outMapping.end();
			for (i = 0, it = m_outMapping.begin(); it != end; i++, it++) {
				if (*it == -1)
					continue;
				m_inMapping[*it] = i;
			}
			break;

		default:
			throw std::runtime_error("invalid direction argument");
	}
}


}          // end of namespace
