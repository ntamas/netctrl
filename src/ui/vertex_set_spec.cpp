/* vim:set ts=4 sw=4 sts=4 et: */

#include "vertex_set_spec.h"

using namespace igraph;
using namespace std;

set<int> VertexSetSpecificationParser::parse(const string& spec) const {
	throw vertex_set_spec_parse_error(spec);
}

