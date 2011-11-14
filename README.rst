=======
netctrl
=======
---------------------------------------------------------------
Controllability of complex networks with node and edge dynamics
---------------------------------------------------------------

:Author: Tamas Nepusz
:Version: 0.1
:License: MIT

This program implements algorithms that search for driver nodes in complex
networks in order to make them (structurally) controllable. The program
currently implements the controllability model of Liu et al [1]_ and the
switchboard dynamics model of Nepusz and Vicsek [2]_. Other models might be
added later.

Requirements
============

- The most recent, up-to-date, bleeding-edge version of igraph_. Really.
You need at least igraph_ 0.6 because that's where the bipartite
matching algorithm was added. At the time of writing, igraph_ 0.6
has not been released yet, and even the bipartite matching algorithm is
implemented in a separate branch only, so you have to check out the
``0.6-matching`` branch from their `Launchpad repository`_ and compile
it yourself. In case there is no such branch, this means that the branch
has already been merged into ``0.6-main``, so check out ``0.6-main``
instead.

- CMake_ to generate the makefiles (or the project file if you are using
Visual Studio).

.. _igraph: http://igraph.sourceforge.net
.. _Launchpad repository: http://launchpad.net/igraph/
.. _CMake: http://www.cmake.org

Compiling from source code
==========================

These instructions are for Linux or Mac OS X and assume that igraph_ is
installed in a way that CMake can figure out automatically where it is.
(This usually involves using ``pkg-config``; if you run ``pkg-config --cflags igraph``
and it works, then it should work with CMake as well)::

    $ git submodule update --init
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

The first command is required only after you have checked out the source code
from GitHub for the first time. The command fetches the source code of the
C++ interface of igraph_ from GitHub and adds it to the source tree.

Usage
=====

The program may operate in one of the following four modes at the moment:

1. Finding driver nodes (``--mode driver_nodes``; this is the default). This mode
   lists the driver nodes of the network being analyzed, one node per line.
   Note that the algorithm finds a single feasible control configuration and
   lists the driver nodes of this configuration only; in other words, if you do
   not see a node in the list of driver nodes, it does not mean that the node
   may not become a driver node in an *alternative* control configuration. E.g.,
   if the network contains a Hamiltonian cycle and you are working with the
   linear nodal dynamics of Liu et al [1]_, *any* node may become a driver node.

2. Finding control paths (``--mode control_paths``). This mode is similar to
   ``driver_nodes``, but provides a more detailed output where each control
   path is listed. Control paths are stems and buds in the Liu et al [1]_
   model and open/closed walks in the switchboard model [2]_; see the respective
   publications for more details.

3. Printing general statistics (``--mode statistics``). This mode prints
   the number/fraction of driver nodes and the  different edge types
   (redundant, ordinary or critical for the linear nodal dynamics;
   distinguished, ordinary or critical for the switchboard dynamics).
   The first row contains the absolute numbers, the second row contains
   the relative fractions. The order of numbers within a row are as follows:
   driver nodes, distinguished edges, redundant edges, ordinary edges and
   critical edges. The linear nodal dynamics contains no distinguished edges;
   the switchboard dynamics contians no redundant edges.

4. Testing the significance of the observed fraction of driver nodes by
   comparing it to null models (``--mode significance``). This mode generates
   100 random instances of different null models for the given network and
   calculates the fraction of driver nodes for all the randomized instances.
   The average values are then listed for each null model and for the actual
   network. The following null models are tested:

   - Erdos-Renyi random networks (``ER``).

   - Configuration model preserving the joint degree distribution
     (``Configuration``).

   - Configuration model that preserves the in- and out-degree sequences but
     not the joint degree distribution (``Configuration_no_joint``).

The mode can be selected with the ``--mode`` (or ``-M``) command line option.
You should also select the controllability model with the ``--model`` (or ``-m``)
option as follows:

- ``switchboard`` selects the switchboard model of Nepusz and Vicsek [2]_
(this is the default).

- ``liu`` selects the linear nodal dynamic model of Liu et al [1]_.

Finally, you may specify an output file (``--output``, ``--o``), suppress most
of the output of the program (``--quiet``, ``--q``) or ask for the command
line help (``-help``, ``--h``).

Input formats
=============

``netctrl`` supports the following input formats:

- Simple edge list format (``.txt``) where each line contains two *numbers*
  corresponding to the source and target vertex IDs. Vertex IDs must be from
  0 to *n*-1, where *n* is the total number of vertices.

- Symbolic edge list format (``.ncol``, also known as the NCOL_ format). In
  this format, each line contains the name of the source and target vertex.
  Names may be arbitrary strings that do not contain whitespaces.

- LGL_ format (``.lgl``)

- GraphML_ format (``.graphml``)

- GML_ format (``.gml``)

.. _LGL: http://lgl.sourceforge.net/#FileFormat
.. _NCOL: http://lgl.sourceforge.net/#FileFormat
.. _GraphML: http://graphml.graphdrawing.org
.. _GML: http://www.fim.uni-passau.de/en/fim/faculty/chairs/theoretische-informatik/projects.html

The input format of the graph will be detected from the extension of the file
name; see above for the recognised extensions.  For the GraphML and GML
formats, vertex names must be provided in the ``name`` vertex attribute. If no
such attribute is present, vertices will use numeric IDs from 0 to *n*-1, where
*n* is the total number of vertices.

Bugs, questions?
================

Have you found a bug in the code? Do you have questions? Let me know.
I think you are smart enough to figure out my email address by googling
for my name. Or just drop me a message on GitHub.

BIbliography
============

.. [1] Liu YY, Slotine JJ and Barabási AL: Controllability of complex
       networks. Nature 473:167-173, 2011.

.. [2] Nepusz T and Vicsek T: Controlling edge dynamics in complex
       networks. In preparation.
