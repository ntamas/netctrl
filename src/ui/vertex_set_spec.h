/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef _VERTEX_SET_SPEC_H
#define _VERTEX_SET_SPEC_H

#include <exception>
#include <set>
#include <igraph/cpp/graph.h>

/// Error thrown when a vertex set specification cannot be parsed
class vertex_set_spec_parse_error : public std::runtime_error {

public:
	vertex_set_spec_parse_error(const std::string& spec) :
		std::runtime_error("Cannot parse vertex set specification: '" + spec + "'")
	{}
	virtual ~vertex_set_spec_parse_error() throw() {}
};

/**
 * \brief Parser that interprets strings as specifications of sets of vertices.
 */
class VertexSetSpecificationParser {

private:
	/**
	 * Graph that the parser is related to. Specifications are interpreted in
	 * the context of this graph.
	 */
	igraph::Graph* const m_pGraph;

    /**
     * Mapping of the vertex names of the graph to the vertex indices.
     */
    mutable std::map<std::string, int> m_vertexNameMapping;

    /**
     * Whether the vertex name mapping has been initialized already or not.
     */
    mutable bool m_vertexNameMappingValid;

public:
	/**
	 * Constructs a parser that uses the given graph.
	 *
	 * \param  graph  the graph that the parser is associated to.
	 */
	VertexSetSpecificationParser(igraph::Graph* const pGraph)
		: m_pGraph(pGraph), m_vertexNameMapping(), m_vertexNameMappingValid(false) {}

	/**
	 * \brief Parses a string that specifies a set of vertices in a graph.
	 *
	 * \param  spec  the string specification; see the accepted formats above
	 * \return the set of vertex indices that match the specification
	 *
	 * \throw vertex_set_spec_parse_error if the specification cannot be parsed
	 */
	std::set<int> parse(const std::string& spec) const;

private:
    /**
     * Returns a numeric vector containing the values of the given structural
     * property for each vertex. It is assumed that the given structural
     * property name is valid.
     *
     * \param  prop  the name of the structural property to retrieve
     * \return a numeric vector containing the value of the given structural
     *         property for all the vertices
     * \see  isValidStructuralProperty()
     */
    igraph::Vector getStructuralPropertyVector(const std::string& prop) const;

    /**
     * Returns the mapping of vertex names to vertex indices and constructs it
     * if needed.
     */
    const std::map<std::string, int>& getVertexNameMapping() const;

    /**
     * Returns whether the given string is a valid structural property name.
     *
     * Currently we recognize \c indegree, \c outdegree, \c degree and
     * \c betweenness only.
     *
     * \param  prop  the name of the structural property to check
     * \return \c true if \em prop is the name of a structural property,
     *         \c false otherwise.
     */
    bool isValidStructuralProperty(const std::string& prop) const;

    /**
     * \brief Parses a string containing a comma-separated list of vertex names.
     *
     * \param  spec    the string to parse
     * \param  result  the set where the parsed indices should be added
     * \return \c true if the parsing was successful, \c false otherwise
	 */
    bool parseAsVertexNames(const std::string& spec, std::set<int>& result) const;

    /**
     * \brief Parses a string containing the name of a structural property and
     *        a number that describes the number of vertices to take based on
     *        the structural property.
     *
     * This function accepts strings of the following format: \c "prop:num" or
     * \c "prop:num%" where \c prop is the name of a structural property
     * (currently we support \c degree and \c betweenness) and \c num is
     * a positive or a negative number. When \c num is positive and has no
     * percentage sign after it, the vertex set specification will match the
     * vertices that have the \em highest values for the given structural property
     * such that exactly the given number of vertices will be returned. (Ties
     * are resolved randomly). When \c num is negative and has no percentage
     * sign after it, the vertex set specification will match the vertices that
     * have the \em lowest values for the given structural property. E.g.,
     * \c "degree:20" will return 20 vertices with the highest degrees, and
     * \c "betweenness:-10" will return 10 vertices with the lowest betweenness
     * centrality. Appending a percentage sign after the specification will
     * interpret the number as a \em percentage instead of an absolute count,
     * e.g., \c "degree:20%" will return 20% of the vertices with the highest
     * degrees.
     *
     * \param  spec    the string to parse
     * \param  result  the set where the parsed indices should be added
     * \return \c true if the parsing was successful, \c false otherwise
	 */
    bool parseAsStructuralProperty(const std::string& spec, std::set<int>& result) const;
};


#endif
