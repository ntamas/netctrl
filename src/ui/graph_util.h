/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef _GRAPH_UTIL_H
#define _GRAPH_UTIL_H

#include <stdexcept>
#include <igraph/cpp/graph.h>

/// Supported formats
typedef enum {
    GRAPH_FORMAT_AUTO,
    GRAPH_FORMAT_UNKNOWN,
    GRAPH_FORMAT_EDGELIST,
    GRAPH_FORMAT_NCOL,
    GRAPH_FORMAT_LGL,
    GRAPH_FORMAT_GRAPHML,
    GRAPH_FORMAT_GML
} GraphFormat;

/// Exception thrown when the format of a graph is unknown
class UnknownGraphFormatException : public std::runtime_error {
private:
    std::string m_filename;

public:
    explicit UnknownGraphFormatException(const std::string& filename = "") :
        std::runtime_error("unknown graph format"), m_filename(filename) {}
    ~UnknownGraphFormatException() throw() {}
};

class GraphUtil {
public:
    /// Tries to detect the format of a graph from its filename
    static GraphFormat detectFormat(const std::string& filename);

    /// Converts a string into the corresponding GraphFormat constant
    static GraphFormat formatFromString(const std::string& str);

    /// Reads a graph without having to know what format it is in
    static igraph::Graph readGraph(const std::string& filename,
            GraphFormat format = GRAPH_FORMAT_AUTO,
            bool directed = true);

    /// Reads a graph from the given stream using the given format
    static igraph::Graph readGraph(FILE* fptr, GraphFormat format,
            bool directed = true);

    /// Writes a graph to the given stream using the given format
    static void writeGraph(FILE* fptr, const igraph::Graph& graph, GraphFormat format);
};

#endif       // _GRAPH_UTIL_H
