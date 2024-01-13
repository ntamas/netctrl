/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef NETCTRL_MODEL_SWITCHBOARD_H
#define NETCTRL_MODEL_SWITCHBOARD_H

#include <netctrl/model/controllability.h>
#include <igraph/cpp/vector.h>
#include <igraph/cpp/vector_bool.h>

namespace netctrl {

class SwitchboardControlPath;

/// Switchboard controllability model
class SwitchboardControllabilityModel : public ControllabilityModel {
public:
    /// The different types of controllability measures in this model
    typedef enum {
        NODE_MEASURE,
        EDGE_MEASURE
    } ControllabilityMeasure;

private:
    /// The list of driver nodes that was calculated
    igraph::VectorInt m_driverNodes;

    /// The list of control paths that was calculated
    std::vector<ControlPath*> m_controlPaths;

    /// Whether we are using the node-based or the edge-based measure
    ControllabilityMeasure m_controllabilityMeasure;

public:
    /// Constructs a model that will operate on the given graph
    SwitchboardControllabilityModel(igraph::Graph* pGraph = 0)
        : ControllabilityModel(pGraph), m_driverNodes(), m_controlPaths(),
          m_controllabilityMeasure(NODE_MEASURE)
    {
    }

    /// Destroys the model
    virtual ~SwitchboardControllabilityModel();

    virtual void calculate();
    igraph::VectorInt changesInDriverNodesAfterEdgeRemoval() const;
    virtual ControllabilityModel* clone();
    virtual float controllability() const;
    virtual std::vector<ControlPath*> controlPaths() const;
    virtual igraph::VectorInt driverNodes() const;
    virtual std::vector<EdgeClass> edgeClasses() const;
    virtual void setGraph(igraph::Graph* graph);

    /// Returns the controllability measure used by the model
    ControllabilityMeasure controllabilityMeasure() const;

    /// Sets the controllability measure used by the model
    /**
     * When using the node-based measure (\c SBD_NODE_MEASURE), the
     * controllability measure is the number of driver nodes divided by the
     * number of nodes. When using the edge-based measure (\c SBD_EDGE_MEASURE),
     * the controllability measure is the number of open control paths plus the
     * number of balanced components, divided by the number of edges.
     */
    void setControllabilityMeasure(ControllabilityMeasure measure);

protected:
    /// Removes all the control paths from the previous run (if any)
    void clearControlPaths();

private:
    /**
     * \brief Starts a walk from the given node following arbitrary edges and
     * creates a control path out of it.
     *
     * \param  start      the node to start the walk from
     * \param  edgeUsed   a vector where we can mark edges that have been used
     *                    up for the current walk (or previous ones)
     * \param  outDegrees the number of unused outbound edges for each node.
     *                    Must be consistent with \c edgeUsed and is updated
     *                    accordingly.
     * \param  inDegrees  the number of unused inbound edges for each node.
     *                    Must be consistent with \c edgeUsed and is updated
     *                    accordingly.
     * \return a newly allocated control path whose ownership is transferred to
     *         the caller
     */
    std::unique_ptr<SwitchboardControlPath> createControlPathFromNode(long int start,
            igraph::VectorBool& edgeUsed, igraph::VectorInt& outDegrees,
            igraph::VectorInt& inDegrees) const;
    
    /**
     * Checks whether the given vertex v is part of a non-trivial
     * balanced component.
     */
    bool isInBalancedComponent(long int v, const igraph::VectorInt& degreeDiffs) const;

    /**
     * Checks whether the given vertex v will be part of a non-trivial
     * balanced component after removing its edge to vertex u.
     */
    bool isInBalancedComponentExcept(long int v, long int u,
            const igraph::VectorInt& degreeDiffs) const;
};

class ClosedWalk;

/// Superclass for all the control paths that occur in the switchboard dynamics
class SwitchboardControlPath : public ControlPath {
public:
    /// Creates an empty control path
    SwitchboardControlPath() : ControlPath() {}

    /// Creates a control path with the given nodes
    explicit SwitchboardControlPath(const igraph::VectorInt& nodes)
        : ControlPath(nodes) {}

    /// Extends the control path with a closed walk.
    /**
     * \param   walk  the closed walk to extend this path with
     * \throws  runtime_error  if the control path and the closed walk share no
     *                         common nodes
     */
    void extendWith(const ClosedWalk* walk);
};

/// Control path that represents a directed open walk
class OpenWalk : public SwitchboardControlPath {
public:
    /// Creates an empty open walk
    OpenWalk() : SwitchboardControlPath() {}

    /// Creates an open walk with the given nodes
    explicit OpenWalk(const igraph::VectorInt& nodes) : SwitchboardControlPath(nodes) {}

    /// Returns the edges involved in the open walk
    virtual igraph::VectorInt edges(const igraph::Graph& graph) const;

    /// Returns a user-friendly name for the control path type
    virtual std::string name() const {
        return "open walk";
    }

    /// Returns \c true since each open walk needs an input signal
    virtual bool needsInputSignal() const {
        return true;
    }

    /// Returns a string representation of the open walk
    virtual std::string toString() const;
};

/// Control path that represents a closed walk
class ClosedWalk : public SwitchboardControlPath {
public:
    /// Creates a closed walk
    ClosedWalk() : SwitchboardControlPath() {}

    /// Creates a closed walk with the given nodes
    explicit ClosedWalk(const igraph::VectorInt& nodes) : SwitchboardControlPath(nodes) {}

    /// Returns \c false since closed walks do not require independent input signals
    virtual bool needsInputSignal() const {
        return false;
    }

    /// Returns the edges involved in the closed walk
    virtual igraph::VectorInt edges(const igraph::Graph& graph) const;

    /// Returns a user-friendly name for the control path type
    virtual std::string name() const {
        return "closed walk";
    }

    /// Returns a string representation of the closed walk
    virtual std::string toString() const;
};

}       // end of namespace

#endif  // NETCTRL_MODEL_SWITCHBOARD_H

