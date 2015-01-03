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

public:
	/**
	 * Constructs a parser that uses the given graph.
	 *
	 * \param  graph  the graph that the parser is associated to.
	 */
	VertexSetSpecificationParser(igraph::Graph* const pGraph)
		: m_pGraph(pGraph) {}

	/**
	 * \brief Parses a string that specifies a set of vertices in a graph.
	 *
	 * \param  spec  the string specification; see the accepted formats above
	 * \param  graph the graph on which the specification is to be interpreted
	 * \return the set of vertex indices that match the specification
	 *
	 * \throw vertex_set_spec_parse_error if the specification cannot be parsed
	 */
	std::set<int> parse(const std::string& spec) const;
};


#endif
