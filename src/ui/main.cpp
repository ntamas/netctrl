/* vim:set ts=4 sw=4 sts=4 et: */

#include <memory>
#include <sstream>
#include <igraph/cpp/graph.h>
#include <igraph/cpp/vertex.h>
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

    /// Graph being analyzed by the UI
    std::auto_ptr<Graph> m_pGraph;

    /// Controllability model being calculated on the graph
    std::auto_ptr<ControllabilityModel> m_pModel;

public:
    LOGGING_FUNCTION(debug, 2);
    LOGGING_FUNCTION(info, 1);
    LOGGING_FUNCTION(error, 0);

    /// Constructor
    NetworkControllabilityApp() : m_pGraph(0), m_pModel(0) {}

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
    std::auto_ptr<Graph> loadGraph(const std::string& filename) {
        std::auto_ptr<Graph> result;

        if (filename == "-") {
            // Loading graph from standard input
            result.reset(new Graph(GraphUtil::readGraph(stdin, GRAPH_FORMAT_EDGELIST)));
        } else if (filename.find("://") != filename.npos) {
            // Generating graph from model
            size_t pos = filename.find("://");
            std::string model = filename.substr(0, pos);
            std::vector<std::string> params = split(filename.substr(pos+3), ',');

            if (model == "er") {
                // Erdos-Renyi network
                if (params.size() < 2) {
                    error("ER generator requires two arguments: number of nodes "
                            "and average degree");
                    return result;    // points to null
                }

                long int n = atoi(params[0].c_str());
                float k = atof(params[1].c_str());
                return erdos_renyi_game_gnm(n, n*k, true, false);
            } else {
                error("Unknown graph generator: %s", model.c_str());
                return result;   // points to null
            }
        } else {
            // Loading graph from file
            result.reset(new Graph(GraphUtil::readGraph(filename)));
            result->setAttribute("filename", filename);
        }

        return result;
    }

    /// Runs the user interface
    int run(int argc, char** argv) {
        m_args.parse(argc, argv);

        info(">> loading graph: %s", m_args.inputFile.c_str());
        m_pGraph = loadGraph(m_args.inputFile);
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
                m_pModel.reset(new SwitchboardControllabilityModel(m_pGraph.get()));
                break;
        }

        switch (m_args.operationMode) {
            case MODE_CONTROL_PATHS:
                return runControlPaths();

            case MODE_DRIVER_NODES:
                return runDriverNodes();

            case MODE_STATISTICS:
                return runStatistics();

            case MODE_SIGNIFICANCE:
                return runSignificance();

            default:
                return 1;
        }
    }

    /// Runs the control path calculation mode
    int runControlPaths() {
        info(">> calculating control paths");
        m_pModel->calculate();

        std::vector<ControlPath*> paths = m_pModel->controlPaths();
        info(">> found %d control path(s)", paths.size());
        for (std::vector<ControlPath*>::const_iterator it = paths.begin();
                it != paths.end(); it++) {
            std::cout << (*it)->toString() << '\n';
        }

        return 0;
    }

    /// Runs the driver node calculation mode
    int runDriverNodes() {
        info(">> calculating control paths and driver nodes");
        m_pModel->calculate();

        Vector driver_nodes = m_pModel->driverNodes();
        info(">> found %d driver node(s)", driver_nodes.size());
        for (Vector::const_iterator it = driver_nodes.begin(); it != driver_nodes.end(); it++) {
            any name(m_pGraph->vertex(*it).getAttribute("name", (long int)*it));
            if (name.type() == typeid(std::string)) {
                std::cout << name.as<std::string>() << '\n';
            } else {
                std::cout << name.as<long int>() << '\n';
            }
        }

        return 0;
    }

    /// Runs the signficance calculation mode
    int runSignificance() {
        size_t observedDriverNodeCount;
        size_t numTrials = 100, i;
        float numNodes = m_pGraph->vcount();
        Vector counts;
        
        info(">> calculating control paths and driver nodes");
        m_pModel->calculate();

        observedDriverNodeCount = m_pModel->driverNodes().size();
        info(">> found %d driver node(s)", observedDriverNodeCount);
        std::cout << "Observed\t" << observedDriverNodeCount / numNodes << '\n';

        // Testing Erdos-Renyi null model
        info(">> testing Erdos-Renyi null model");
        counts.clear();
        for (i = 0; i < numTrials; i++) {
            std::auto_ptr<Graph> graph = igraph::erdos_renyi_game_gnm(
                    numNodes, m_pGraph->ecount(),
                    m_pGraph->isDirected(), false);

            std::auto_ptr<ControllabilityModel> pModel(m_pModel->clone());
            pModel->setGraph(graph.get());
            pModel->calculate();

            counts.push_back(pModel->driverNodes().size() / numNodes);
        }
        counts.sort();
        std::cout << "ER\t" << counts.sum() / counts.size() << '\n';

        // Testing configuration model
        Vector inDegrees, outDegrees;
        m_pGraph->degree(&outDegrees, V(m_pGraph.get()), IGRAPH_OUT, true);
        m_pGraph->degree(&inDegrees,  V(m_pGraph.get()), IGRAPH_IN,  true);

        info(">> testing configuration model (preserving joint degree distribution)");
        counts.clear();
        for (i = 0; i < numTrials; i++) {
            std::auto_ptr<Graph> graph =
                igraph::degree_sequence_game(outDegrees, inDegrees,
                    IGRAPH_DEGSEQ_SIMPLE);

            std::auto_ptr<ControllabilityModel> pModel(m_pModel->clone());
            pModel->setGraph(graph.get());
            pModel->calculate();

            counts.push_back(pModel->driverNodes().size() / numNodes);
        }
        counts.sort();
        std::cout << "Configuration\t" << counts.sum() / counts.size() << '\n';

        // Testing configuration model
        info(">> testing configuration model (destroying joint degree distribution)");
        counts.clear();
        for (i = 0; i < numTrials; i++) {
            inDegrees.shuffle();
            outDegrees.shuffle();

            std::auto_ptr<Graph> graph =
                igraph::degree_sequence_game(outDegrees, inDegrees,
                    IGRAPH_DEGSEQ_SIMPLE);

            std::auto_ptr<ControllabilityModel> pModel(m_pModel->clone());
            pModel->setGraph(graph.get());
            pModel->calculate();

            counts.push_back(pModel->driverNodes().size() / numNodes);
        }
        counts.sort();
        std::cout << "Configuration_no_joint\t" << counts.sum() / counts.size() << '\n';

        return 0;
    }

    /// Runs the general statistics calculation mode
    int runStatistics() {
        float n = m_pGraph->vcount();
        float m = m_pGraph->ecount();
        long int num_driver;
        long int num_redundant = 0, num_ordinary = 0, num_critical = 0, num_distinguished = 0;

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

        std::cout << num_driver << ' '
                  << num_distinguished << ' '
                  << num_redundant << ' '
                  << num_ordinary << ' '
                  << num_critical << '\n';
        std::cout << num_driver / n << ' '
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

