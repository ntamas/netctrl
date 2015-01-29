/* vim:set ts=4 sw=4 sts=4 et: */

#include <igraph/cpp/centrality.h>
#include <igraph/cpp/vector_long.h>
#include <igraph/cpp/vertex_selector.h>
#include <sstream>
#include "vertex_set_spec.h"

using namespace igraph;
using namespace std;

/**
 * Helper class that compares integers based on the corresponding values in
 * some vector.
 */
template <typename T>
struct VectorComparator {
    const T& m_vector;
    bool m_reversed;

    VectorComparator(const T& vector, bool reversed=false)
        : m_vector(vector), m_reversed(reversed) {}

    template <typename V>
    bool operator()(V first, V second) {
        if (m_reversed) {
            return m_vector[first] >= m_vector[second];
        } else {
            return m_vector[first] < m_vector[second];
        }
    }
};

igraph::Vector VertexSetSpecificationParser::getStructuralPropertyVector(
        const string& prop) const {
    if (prop == "degree") {
        return m_pGraph->degree(V(m_pGraph), IGRAPH_ALL, true);
    } else if (prop == "indegree") {
        return m_pGraph->degree(V(m_pGraph), IGRAPH_IN, true);
    } else if (prop == "outdegree") {
        return m_pGraph->degree(V(m_pGraph), IGRAPH_OUT, true);
    } else if (prop == "betweenness") {
        return betweenness(*m_pGraph, V(m_pGraph));
    }
    return Vector(m_pGraph->vcount());
}

const map<string, int>& VertexSetSpecificationParser::getVertexNameMapping() const {
    if (!m_vertexNameMappingValid) {
        AttributeValueVector names = V(m_pGraph).getAttribute("name");
        AttributeValueVector::const_iterator it = names.begin();
        AttributeValueVector::const_iterator end = names.end();
        int index = 0;

        while (it != end) {
            string name = *(it->as<string>());
            m_vertexNameMapping[name] = index;
            it++;
            index++;
        }

        m_vertexNameMappingValid = true;
    }
    return m_vertexNameMapping;
}

bool VertexSetSpecificationParser::isValidStructuralProperty(const string& prop) const {
    return (prop == "degree" || prop == "indegree" || prop == "outdegree" ||
            prop == "betweenness");
}

set<int> VertexSetSpecificationParser::parse(const string& spec) const {
    set<int> result;
    bool success = false;

    success = success || parseAsStructuralProperty(spec, result);
    success = success || parseAsVertexNames(spec, result);

    if (success) {
        return result;
    } else {
        throw vertex_set_spec_parse_error(spec);
    }
}

bool VertexSetSpecificationParser::parseAsVertexNames(const string& spec,
        set<int>& result) const {
    const map<string, int>& vertexNameMapping = getVertexNameMapping();
    map<string, int>::const_iterator it;
    stringstream ss(spec);
    string item;

    result.clear();
    while (getline(ss, item, ',')) {
        it = vertexNameMapping.find(item);
        if (it == vertexNameMapping.end())
            return false;

        result.insert(it->second);
    }

    return true;
}

bool VertexSetSpecificationParser::parseAsStructuralProperty(const string& spec,
        set<int>& result) const {
    stringstream ss(spec);
    string propertyName;
    double number;
    long int count;

    result.clear();
    if (!getline(ss, propertyName, ':'))
        return false;

    if (!isValidStructuralProperty(propertyName))
        return false;

    if (!(ss >> number))
        return false;

    if (ss.peek() == '%') {
        // Use percentages
        count = static_cast<long int>(round(number / 100.0 * m_pGraph->vcount()));
    } else if (ss.eof()) {
        // Use an absolute count
        // count must be an integer so let's validate it
        count = static_cast<long int>(number);
        if (number != count) {
            return false;
        }
    } else {
        // Something else is left in the stream; this is a parse error
        return false;
    }

    // Okay, get the values of the structural property
    Vector values = getStructuralPropertyVector(propertyName);

    // Construct the index vector
    VectorLong indices = VectorLong::Seq(0, m_pGraph->vcount()-1);

    // Make a heap out of the index vector, using the values as a comparator
    VectorComparator<Vector> cmp(values, count < 0);
    make_heap(indices.begin(), indices.end(), cmp);
    count = abs(count);
    
    // Pop the items from the heap
    while (count > 0) {
        pop_heap(indices.begin(), indices.end(), cmp);
        result.insert(indices.pop_back());
        count--;
    }

    return true;
}

