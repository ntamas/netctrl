/* vim:set ts=4 sw=4 sts=4 et: */

#include <algorithm>
#include <sstream>
#include <igraph/cpp/io.h>
#include "graph_util.h"

using namespace std;
using namespace igraph;

GraphFormat GraphUtil::detectFormat(const string& filename) {
    string::size_type idx = filename.rfind('.');

    if (idx == std::string::npos)
        return GRAPH_FORMAT_UNKNOWN;
    
    string extension = filename.substr(idx + 1);
    transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    if (extension == "gml")
        return GRAPH_FORMAT_GML;
    if (extension == "ncol")
        return GRAPH_FORMAT_NCOL;
    if (extension == "lgl")
        return GRAPH_FORMAT_LGL;
    if (extension == "txt")
        return GRAPH_FORMAT_EDGELIST;
    if (extension == "graphml")
        return GRAPH_FORMAT_GRAPHML;

    return GRAPH_FORMAT_UNKNOWN;
}

Graph GraphUtil::readGraph(const string& filename, GraphFormat format, bool directed) {
    if (format == GRAPH_FORMAT_AUTO || format == GRAPH_FORMAT_UNKNOWN)
        format = GraphUtil::detectFormat(filename);

    FILE* fptr = fopen(filename.c_str(), "r");
    if (fptr == NULL) {
        ostringstream oss;
        oss << "File not found: " << filename;
        throw runtime_error(oss.str());
    }

    try {
        Graph result = readGraph(fptr, format, directed);
        fclose(fptr);
        return result;
    } catch (const UnknownGraphFormatException& ex) {
        throw UnknownGraphFormatException(filename);
    }
}

Graph GraphUtil::readGraph(FILE* fptr, GraphFormat format, bool directed) {
    Graph result;

    switch (format) {
        case GRAPH_FORMAT_EDGELIST:
            result = read_edgelist(fptr, 0, directed);
            break;

        case GRAPH_FORMAT_NCOL:
            result = read_ncol(fptr, true, IGRAPH_ADD_WEIGHTS_IF_PRESENT, directed);
            break;

        case GRAPH_FORMAT_LGL:
            result = read_lgl(fptr, false, IGRAPH_ADD_WEIGHTS_IF_PRESENT, directed);
            break;

        case GRAPH_FORMAT_GRAPHML:
            result = read_graphml(fptr);
            break;

        case GRAPH_FORMAT_GML:
            result = read_gml(fptr);
            break;

        default:
            throw UnknownGraphFormatException();
    }

    return result;
}

void GraphUtil::writeGraph(FILE* fptr, const Graph& graph, GraphFormat format) {
    switch (format) {
        case GRAPH_FORMAT_GRAPHML:
            write_graphml(graph, fptr);
            break;

        case GRAPH_FORMAT_GML:
            write_gml(graph, fptr);
            break;

        default:
            throw UnknownGraphFormatException();
    }
}

