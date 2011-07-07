/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef NETCTRL_UTIL_DIRECTED_MATCHING_H
#define NETCTRL_UTIL_DIRECTED_MATCHING_H

#include <igraph/cpp/vector_long.h>

namespace netctrl {

/// Represents a matching in a directed graph as defined by Liu et al
class DirectedMatching {
private:
    /// Stores the mapping from matching nodes to matched nodes
    igraph::VectorLong m_outMapping;

    /// Stores the mapping from matched nodes to matching nodes
    igraph::VectorLong m_inMapping;

public:
    /// Enum for the constructor that denotes the format of the incoming vector
    enum Direction { DIRECTION_OUT, DIRECTION_IN, DIRECTION_OUT_IN,
                     DIRECTION_IN_OUT };

    /// Constructs an empty matching
    DirectedMatching() : m_outMapping(), m_inMapping() {}

    /// Constructs a matching
    /**
     * \param  vector     the vector that describes the matching
     * \param  direction  denotes the format of the vector.
     *                    \c DIRECTION_OUT means that element i of the
     *                    vector contains the index of the node that
     *                    vertex i is *matched to*. \c DIRECTION_IN means
     *                    that element i of the vector contains the
     *                    node that *matches* i. \c DIRECTION_OUT_IN and
     *                    \c DIRECTION_IN_OUT mean that both sides of the
     *                    mapping are provided in out-in or in-out order,
     *                    concatenated.
     */
    DirectedMatching(const igraph::VectorLong& mapping, Direction direction);

    /**
     * Returns whether the given node is matched by another node.
     */
    bool isMatched(long int v) const {
        return matchIn(v) != -1;
    }

    /**
     * Returns whether the given node matches another node.
     */
    bool isMatching(long int u) const {
        return matchOut(u) != -1;
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
     * Returns the index of the node a given node is matched to.
     *
     * \param    u  the index of the node we are interested in.
     * \returns  the index of the node that node u is matched to, or -1 if node
     *           u is unmatched.
     */
    long int matchOut(long int u) const {
        return m_outMapping[u];
    }

    /**
     * Establishes a matching between the two given nodes.
     *
     * This method also takes care of erasing any existing matching
     * related to the nodes.
     */
    void setMatch(long int u, long int v) {
        if (v == -1 || u == -1)
            return;

        if (m_outMapping[u] == v)
            return;

        unmatch(u, m_outMapping[u]);
        unmatch(m_inMapping[v], v);
        m_outMapping[u] = v;
        m_inMapping[v] = u;
    }

    /**
     * Destroys the matching between the two given nodes.
     *
     * It is \em not checked whether the two nodes are really matched
     * to each other.
     *
     * \param  u  the matching node that matches v.
     * \param  v  the node that is matched by u.
     */
    void unmatch(long int u, long int v) {
        if (v == -1 || u == -1)
            return;
        assert(m_inMapping[v] == u);
        assert(m_outMapping[u] == v);
        m_inMapping[v] = -1;
        m_outMapping[u] = -1;
    }
};

}          // end of namespace

#endif


