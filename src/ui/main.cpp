/* vim:set ts=4 sw=4 sts=4 et: */

#include <memory>
#include <igraph/cpp/graph.h>
#include <igraph/cpp/vertex.h>
#include <netctrl/model.h>

#include "cmd_arguments.h"
#include "graph_util.h"
#include "logging.h"

using namespace igraph;
using namespace netctrl;
using namespace std;

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
            result.reset(new Graph(GraphUtil::readGraph(stdin, GRAPH_FORMAT_EDGELIST)));
        } else {
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
        info(">> graph has %ld vertices and %ld edges",
             (long)m_pGraph->vcount(), (long)m_pGraph->ecount());

        switch (m_args.modelType) {
            case LIU_MODEL:
                m_pModel.reset(new LiuControllabilityModel(m_pGraph.get()));
                break;
            case SWITCHBOARD_MODEL:
                m_pModel.reset(new SwitchboardControllabilityModel(m_pGraph.get()));
                break;
        }

        info(">> calculating control paths and driver nodes");
        m_pModel->calculate();

        Vector driver_nodes = m_pModel->getDriverNodes();
        info(">> found %d driver node(s)", driver_nodes.size());
        for (Vector::const_iterator it = driver_nodes.begin(); it != driver_nodes.end(); it++) {
            any name(m_pGraph->vertex(*it).getAttribute("name", (long int)*it));
            if (name.type() == typeid(std::string)) {
                cout << name.as<std::string>() << '\n';
            } else {
                cout << name.as<long int>() << '\n';
            }
        }

        return 0;
    }
};

int main(int argc, char** argv) {
    NetworkControllabilityApp app;

    igraph::AttributeHandler::attach();
    return app.run(argc, argv);
}

