/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef NETCTRL_UTIL_DIRECTED_MATCHING_H
#define NETCTRL_UTIL_DIRECTED_MATCHING_H

#include <igraph/cpp/vector_long.h>
#include <map>

namespace netctrl {

/// Represents a matching in a directed graph as defined by Liu et al
/**
 * This class implements a one-to-many directed matching where each node may
 * be \em "matched by" at most one other node, but any node may \em match zero,
 * one or more than one other node.
 *
 * Note that when we are working with a controllability problem where each node
 * of the graph is a target node, the resulting matching will be more
 * restrictive since each node may match zero or one other node only. However,
 * when not all of the nodes are control targets, it may happen that a driver
 * node controls multiple other nodes via paths of different lengths.
 */
class DirectedMatching {
private:
    /// Type alias for the out-mapping
    typedef std::map<long int, igraph::VectorLong> OutMapping;

    /// Stores the mapping from matching nodes to lists of matched nodes
    OutMapping m_outMapping;

    /// Stores the mapping from matched nodes to matching nodes
    igraph::VectorLong m_inMapping;

public:
    /// Constructs an empty matching with no nodes
    DirectedMatching() : m_outMapping(), m_inMapping() {}

    /// Constructs an empty matching with the given number of nodes
    DirectedMatching(long int n) : m_outMapping(), m_inMapping(n) {
        m_inMapping.fill(-1);
    }

    /// Constructs a matching
    /**
     * \param  vector     the vector that describes the matching.
     *                    Element i of the vector must contain the index of
     *                    the node that *matches* i.
     */
    DirectedMatching(const igraph::VectorLong& mapping);

    /**
     * Returns whether the given node is matched by another node.
     */
    bool isMatched(long int v) const {
        return matchIn(v) != -1;
    }

    /**
     * Returns whether the given node matches \em "at least" one other node.
     */
    bool isMatching(long int u) const {
        OutMapping::const_iterator it = m_outMapping.find(u);
        return it != m_outMapping.end() && !it->second.empty();
    }

    /**
     * Returns whether the given node matches \em exactly one other node.
     */
    bool isMatchingExactlyOne(long int u) const {
        OutMapping::const_iterator it = m_outMapping.find(u);
        return it != m_outMapping.end() && it->second.size() == 1;
    }

    /**
     * Returns the index of the node a given node is matched by.
     *
     * \param    v  the index of the node we are interested in.
     * \returns  the index of the node that matches node v, or -1 if node
     *           v is unmatched.
     */
    long int matchIn(long int v) const {
        return m_inMapping[v];
    }

    /**
     * Returns the indices of the nodes a given node is matched to.
     *
     * \param    u  the index of the node we are interested in.
     * \returns  pointer to a vector containing the indices of the nodes that
     *           node u is matched to, or a null pointer if u is unmatched.
     *           This vector \em "must not" be modified or freed.
     */
    const igraph::VectorLong* matchOut(long int u) const;

    /**
     * Establishes a matching between the two given nodes such that
     * node \em u is matched to node \em v.
     *
     * This method also takes care of erasing any existing matching
     * related to node v.
     *
     * \param  u  the matching node that matches v.
     * \param  v  the node that is matched by u.
     */
    void setMatch(long int u, long int v);

    /**
     * Destroys the matching between the given node and the node it is
     * matched by.
     *
     * \param  v  the node that is matched by some other node.
     */
    void unmatch(long int v);

    void print() { m_inMapping.print(); }
};

}          // end of namespace

#endif


