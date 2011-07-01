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
    if (extension == "ncol")
        return GRAPH_FORMAT_NCOL;
    if (extension == "txt")
        return GRAPH_FORMAT_EDGELIST;
    if (extension == "graphml")
        return GRAPH_FORMAT_GRAPHML;

    return GRAPH_FORMAT_UNKNOWN;
}

Graph GraphUtil::readGraph(const string& filename, GraphFormat format) {
    if (format == GRAPH_FORMAT_AUTO || format == GRAPH_FORMAT_UNKNOWN)
        format = GraphUtil::detectFormat(filename);

    FILE* fptr = fopen(filename.c_str(), "r");
    if (fptr == NULL) {
        ostringstream oss;
        oss << "File not found: " << filename;
        throw runtime_error(oss.str());
    }

    try {
        Graph result = readGraph(fptr, format);
        fclose(fptr);
        return result;
    } catch (const UnknownGraphFormatException& ex) {
        throw UnknownGraphFormatException(filename);
    }
}

Graph GraphUtil::readGraph(FILE* fptr, GraphFormat format) {
    Graph result;
    bool directed = false;

    switch (format) {
        case GRAPH_FORMAT_EDGELIST:
            result = read_edgelist(fptr, 0, directed);
            break;

        case GRAPH_FORMAT_NCOL:
            result = read_ncol(fptr, true, IGRAPH_ADD_WEIGHTS_IF_PRESENT, directed);
            break;

        case GRAPH_FORMAT_GRAPHML:
            result = read_graphml(fptr);
            break;

        default:
            throw UnknownGraphFormatException();
    }

    return result;
}
