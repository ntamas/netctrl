/* vim:set ts=4 sw=4 sts=4 et: */

#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <igraph/cpp/graph.h>
#include <igraph/cpp/edge.h>
#include <igraph/cpp/vertex.h>
#include <igraph/cpp/vertex_selector.h>
#include <igraph/cpp/generators/degree_sequence.h>
#include <igraph/cpp/generators/erdos_renyi.h>
#include <netctrl/model.h>

#include "cmd_arguments.h"
#include "graph_util.h"
#include "logging.h"

using namespace igraph;
using namespace netctrl;


/// Helper function to split a string around a delimiter character
std::vector<std::string> &split(const std::string& s, char delim,
        std::vector<std::string>& elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/// Helper function to split a string around a delimiter character
std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> elems;
    return split(s, delim, elems);
}

class NetworkControllabilityApp {
private:
    /// Parsed command line arguments
    CommandLineArguments m_args;

    /// The C-style output file object where the results will be written
    FILE* m_outputFileObject;

    /// Graph being analyzed by the UI
    std::unique_ptr<Graph> m_pGraph;

    /// Controllability model being calculated on the graph
    std::unique_ptr<ControllabilityModel> m_pModel;

    /// The C++-style output stream where the results will be written
    std::ostream* m_pOutputStream;

public:
    LOGGING_FUNCTION(debug, 2);
    LOGGING_FUNCTION(info, 1);
    LOGGING_FUNCTION(error, 0);

    /// Constructor
    NetworkControllabilityApp() : m_outputFileObject(0), m_pOutputStream(0) {}

    /// Destructor
    ~NetworkControllabilityApp() {
        if (m_outputFileObject != 0 && m_outputFileObject != stdout) {
            fclose(m_outputFileObject);
        }

        if (m_pOutputStream != 0 && m_pOutputStream != &std::cout) {
            delete m_pOutputStream;
        }
    }

    /// Returns the C-style output file object where the results should be written
    FILE* getOutputFileObject() {
        if (m_outputFileObject == 0) {
            if (isWritingToStandardOutput()) {
                m_outputFileObject = stdout;
            } else {
                m_outputFileObject = fopen(m_args.outputFile.c_str(), "w");
                if (m_outputFileObject == 0) {
                    error("cannot open output file for writing: %s",
                            m_args.outputFile.c_str());
                    exit(3);
                }
            }
        }
        return m_outputFileObject;
    }

    /// Returns the C++-style output stream where the results should be written
    std::ostream& getOutputStream() {
        if (m_pOutputStream == 0) {
            if (isWritingToStandardOutput()) {
                m_pOutputStream = &std::cout;
            } else {
                m_pOutputStream = new std::ofstream(m_args.outputFile.c_str());
                if (m_pOutputStream->fail()) {
                    error("cannot open output file for writing: %s",
                            m_args.outputFile.c_str());
                    exit(3);
                }
            }
        }
        return *m_pOutputStream;
    }

    /// Returns whether we are writing to the standard output
    bool isWritingToStandardOutput() {
        return m_args.outputFile.empty() || m_args.outputFile == "-";
    }

    /// Returns whether we are running in quiet mode
    bool isQuiet() {
        return m_args.verbosity < 1;
    }

    /// Returns whether we are running in verbose mode
    bool isVerbose() {
        return m_args.verbosity > 1;
    }

    /// Loads a graph from the given file
    /**
     * If the name of the file is "-", the file is assumed to be the
     * standard input.
     */
    std::unique_ptr<Graph> loadGraph(const std::string& filename, GraphFormat format) {
        std::unique_ptr<Graph> result;

        if (filename == "-") {
            // Loading graph from standard input
            if (format == GRAPH_FORMAT_AUTO)
                format = GRAPH_FORMAT_EDGELIST;
            result.reset(new Graph(GraphUtil::readGraph(stdin, format)));
        } else if (filename.find("://") != filename.npos) {
            // Generating graph from model
            size_t pos = filename.find("://");
            std::string model = filename.substr(0, pos);
            std::vector<std::string> params = split(filename.substr(pos+3), ',');

            if (model == "er") {
                // Erdos-Renyi network
                if (params.size() < 2) {
                    error("ER generator requires two or three arguments: number of nodes, "
                            "average degree and directedness (optional)");
                    return result;    // points to null
                }

                long int n = atoi(params[0].c_str());
                float k = atof(params[1].c_str());
                bool directed = true;
                if (params.size() >= 3) {
                    directed = atoi(params[2].c_str()) != 0;
                }
                return erdos_renyi_game_gnm(n, directed ? n*k : n*k/2.0,
                        directed, false);
            } else {
                error("Unknown graph generator: %s", model.c_str());
                return result;   // points to null
            }
        } else {
            // Loading graph from file
            result.reset(new Graph(GraphUtil::readGraph(filename, format)));
            result->setAttribute("filename", filename);
        }

        return result;
    }

    /// Runs the user interface
    int run(int argc, char** argv) {
        int retval;

        m_args.parse(argc, argv);

        info(">> loading graph: %s", m_args.inputFile.c_str());
        m_pGraph = loadGraph(m_args.inputFile, m_args.inputFormat);
        if (m_pGraph.get() == NULL)
            return 2;

        info(">> graph is %s and has %ld vertices and %ld edges",
             m_pGraph->isDirected() ? "directed" : "undirected",
             (long)m_pGraph->vcount(), (long)m_pGraph->ecount());

        switch (m_args.modelType) {
            case LIU_MODEL:
                m_pModel.reset(new LiuControllabilityModel(m_pGraph.get()));
                break;
            case SWITCHBOARD_MODEL:
                {
                    SwitchboardControllabilityModel* sbdModel;
                    sbdModel = new SwitchboardControllabilityModel(m_pGraph.get());
                    sbdModel->setControllabilityMeasure(
                            m_args.useEdgeMeasure ?
                            SwitchboardControllabilityModel::EDGE_MEASURE :
                            SwitchboardControllabilityModel::NODE_MEASURE
                    );
                    m_pModel.reset(sbdModel);
                }
                break;
        }

        switch (m_args.operationMode) {
            case MODE_CONTROL_PATHS:
                retval = runControlPaths();
                break;

            case MODE_DRIVER_NODES:
                retval = runDriverNodes();
                break;

            case MODE_GRAPH:
                retval = runGraph();
                break;

            case MODE_STATISTICS:
                retval = runStatistics();
                break;

            case MODE_SIGNIFICANCE:
                retval = runSignificance();
                break;

            default:
                retval = 1;
        }

        if (!retval && !isWritingToStandardOutput()) {
            info(">> results were written to %s", m_args.outputFile.c_str());
        }

        return retval;
    }

    /// Runs the control path calculation mode
    int runControlPaths() {
        info(">> calculating control paths");
        m_pModel->calculate();

        std::vector<ControlPath*> paths = m_pModel->controlPaths();
        std::ostream& out = getOutputStream();

        info(">> found %d control path(s)", paths.size());
        for (std::vector<ControlPath*>::const_iterator it = paths.begin();
                it != paths.end(); it++) {
            out << (*it)->toString() << '\n';
        }

        return 0;
    }

    /// Runs the driver node calculation mode
    int runDriverNodes() {
        info(">> calculating control paths and driver nodes");
        m_pModel->calculate();

        VectorInt driver_nodes = m_pModel->driverNodes();
        std::ostream& out = getOutputStream();

        info(">> found %d driver node(s)", driver_nodes.size());
        for (VectorInt::const_iterator it = driver_nodes.begin(); it != driver_nodes.end(); it++) {
            any name(m_pGraph->vertex(*it).getAttribute("name", (long int)*it));
            if (name.type() == typeid(std::string)) {
                out << name.as<std::string>() << '\n';
            } else {
                out << name.as<long int>() << '\n';
            }
        }

        return 0;
    }

    /// Runs the annotated graph output mode
    int runGraph() {
        long int i, j, n;

        info(">> calculating control paths and driver nodes");
        m_pModel->calculate();

        VectorInt driver_nodes = m_pModel->driverNodes();
        std::vector<ControlPath*> paths = m_pModel->controlPaths();
        info(">> found %d driver node(s) and %d control path(s)",
                driver_nodes.size(), paths.size());

        info(">> classifying edges");
        std::vector<EdgeClass> edge_classes = m_pModel->edgeClasses();

        // Mark the driver nodes
        for (VectorInt::const_iterator it = driver_nodes.begin(); it != driver_nodes.end(); it++) {
            m_pGraph->vertex(*it).setAttribute("is_driver", true);
        }

        // Mark the edge types and path indices
        j = 0;
        for (std::vector<ControlPath*>::const_iterator it = paths.begin();
                it != paths.end(); it++, j++) {
            const igraph::VectorInt& vec = (*it)->edges(*m_pGraph.get());
            n = vec.size();
            for (i = 0; i < n; i++) {
                igraph::Edge edge = m_pGraph->edge(vec[i]);
                edge.setAttribute("path_type", (*it)->name());
                edge.setAttribute("path_indices", j);
                edge.setAttribute("path_order", i);
            }
        }

        // Mark the edge classes
        n = m_pGraph->ecount();
        for (i = 0; i < n; i++) {
            igraph::Edge edge = m_pGraph->edge(i);
            edge.setAttribute("edge_class", edgeClassToString(edge_classes[i]));
        }

        // Print the graph
        GraphUtil::writeGraph(getOutputFileObject(), (*m_pGraph.get()), m_args.outputFormat);

        return 0;
    }

    /// Runs the signficance calculation mode
    int runSignificance() {
        size_t observedDriverNodeCount;
        size_t numTrials = 100, i;
        float numNodes = m_pGraph->vcount();
        float controllability;
        Vector counts;
        std::ostream& out = getOutputStream();
        
        info(">> calculating control paths and driver nodes");
        m_pModel->calculate();

        observedDriverNodeCount = m_pModel->driverNodes().size();
        controllability = m_pModel->controllability();

        info(">> found %d driver node(s)", observedDriverNodeCount);
        out << "Observed\t" << controllability << '\n';

        // Testing Erdos-Renyi null model
        info(">> testing Erdos-Renyi null model");
        counts.clear();
        for (i = 0; i < numTrials; i++) {
            std::unique_ptr<Graph> graph = igraph::erdos_renyi_game_gnm(
                    numNodes, m_pGraph->ecount(),
                    m_pGraph->isDirected(), false);

            std::unique_ptr<ControllabilityModel> pModel(m_pModel->clone());
            pModel->setGraph(graph.get());
            pModel->calculate();

            counts.push_back(pModel->controllability());
        }
        counts.sort();
        out << "ER\t" << counts.sum() / counts.size() << '\n';

        // Testing configuration model
        VectorInt inDegrees, outDegrees;
        m_pGraph->degree(&outDegrees, V(m_pGraph.get()), IGRAPH_OUT, true);
        m_pGraph->degree(&inDegrees,  V(m_pGraph.get()), IGRAPH_IN,  true);

        info(">> testing configuration model (preserving joint degree distribution)");
        counts.clear();
        for (i = 0; i < numTrials; i++) {
            std::unique_ptr<Graph> graph =
                igraph::degree_sequence_game(outDegrees, inDegrees,
                    IGRAPH_DEGSEQ_CONFIGURATION);

            std::unique_ptr<ControllabilityModel> pModel(m_pModel->clone());
            pModel->setGraph(graph.get());
            pModel->calculate();

            counts.push_back(pModel->controllability());
        }
        counts.sort();
        out << "Configuration\t" << counts.sum() / counts.size() << '\n';

        // Testing configuration model
        info(">> testing configuration model (destroying joint degree distribution)");
        counts.clear();
        for (i = 0; i < numTrials; i++) {
            inDegrees.shuffle();
            outDegrees.shuffle();

            std::unique_ptr<Graph> graph =
                igraph::degree_sequence_game(outDegrees, inDegrees,
                    IGRAPH_DEGSEQ_CONFIGURATION);

            std::unique_ptr<ControllabilityModel> pModel(m_pModel->clone());
            pModel->setGraph(graph.get());
            pModel->calculate();

            counts.push_back(pModel->controllability());
        }
        counts.sort();
        out << "Configuration_no_joint\t" << counts.sum() / counts.size() << '\n';

        return 0;
    }

    /// Runs the general statistics calculation mode
    int runStatistics() {
        float n = m_pGraph->vcount();
        float m = m_pGraph->ecount();
        long int num_driver;
        long int num_redundant = 0, num_ordinary = 0, num_critical = 0, num_distinguished = 0;
        std::ostream& out = getOutputStream();
 
        info(">> calculating control paths and driver nodes");
        m_pModel->calculate();
        num_driver = m_pModel->driverNodes().size();

        info(">> classifying edges");
        std::vector<EdgeClass> edge_classes = m_pModel->edgeClasses();
        if (edge_classes.size() == m && !edge_classes.empty()) {
            for (long int i = 0; i < m; i++) {
                if (edge_classes[i] == EDGE_REDUNDANT)
                    num_redundant++;
                else if (edge_classes[i] == EDGE_ORDINARY)
                    num_ordinary++;
                else if (edge_classes[i] == EDGE_DISTINGUISHED)
                    num_distinguished++;
                else
                    num_critical++;
            }
        }

        info(">> order is as follows:");
        info(">> driver nodes; distinguished, redundant, ordinary, critical edges");

        out << num_driver << ' '
            << num_distinguished << ' '
            << num_redundant << ' '
            << num_ordinary << ' '
            << num_critical << '\n';
        out << num_driver / n << ' '
            << num_distinguished / m << ' '
            << num_redundant / m << ' '
            << num_ordinary / m << ' '
            << num_critical / m << '\n';

        return 0;
    }

};

int main(int argc, char** argv) {
    NetworkControllabilityApp app;

    igraph::AttributeHandler::attach();
    return app.run(argc, argv);
}

