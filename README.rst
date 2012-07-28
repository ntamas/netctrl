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

Precompiled binaries
====================

Follow `this link <http://hal.elte.hu/~nepusz/files/netctrl>`_ for precompiled
packages for Linux systems running on 32-bit or 64-bit processors.

If you are running a different system (e.g., Windows or Mac OS X), you have to
compile ``netctrl`` yourself; please proceed to the `Compiling from source
code`_ section. You must also compile ``netctrl`` yourself if you need the
bleeding edge version as the packages at the above URL are not guaranteed to be
updated regularly.  However, they could safely be used to check out ``netctrl``
quickly without having to go through all the hassle with compiling ``netctrl``
from source.

If you are using a precompiled binary, please proceed to the Usage_ section
for usage instructions.

Compiling from source code
==========================

Requirements
------------

- The most recent, up-to-date, bleeding-edge version of igraph_. Really.
  You need at least igraph_ 0.6 because that's where the bipartite
  matching algorithm was added.

- CMake_ to generate the makefiles (or the project file if you are using
  Visual Studio).

.. _igraph: http://igraph.sourceforge.net
.. _Launchpad repository: http://launchpad.net/igraph/
.. _CMake: http://www.cmake.org

Compiling using ``cmake`` and ``make``
--------------------------------------

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

The program may operate in one of the following five modes at the moment:

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

5. Annotating the edges and nodes of the input graph with several attributes.
   For each node, ``netctrl`` will determine whether the node is a driver node
   or not. For each edge, ``netctrl`` will determine whether the edge is
   distinguished, redundant, ordinary or critical (see also ``--mode statistcs``
   above), indicate which control path it is a part of (if any), and also
   determines the position of each edge in its control path. The results are
   printed in either GraphML or GML format, depending on the value of the
   ``-F`` (or ``--output-format``) argument.

The mode can be selected with the ``--mode`` (or ``-M``) command line option.
You should also select the controllability model with the ``--model`` (or ``-m``)
option as follows:

- ``switchboard`` selects the switchboard model of Nepusz and Vicsek [2]_
  (this is the default).

- ``liu`` selects the linear nodal dynamic model of Liu et al [1]_.

Finally, you may specify an output file (``--output``, ``-o``), suppress most
of the output of the program (``--quiet``, ``-q``) or ask for the command
line help (``--help``, ``-h``).

Input formats
=============

``netctrl`` supports the following input formats:

- Simple edge list format (``.txt``) where each line contains two *numbers*
  corresponding to the source and target vertex IDs. Vertex IDs must be from
  0 to *n*-1, where *n* is the total number of vertices.

- Symbolic edge list format (``.ncol``, also known as the NCOL_ format). In
  this format, each line contains the name of the source and target vertex.
  Names may be arbitrary strings that do not contain whitespace.

- LGL_ format (``.lgl``)

- GraphML_ format (``.graphml``)

- GML_ format (``.gml``)

.. _LGL: http://lgl.sourceforge.net/#FileFormat
.. _NCOL: http://lgl.sourceforge.net/#FileFormat
.. _GraphML: http://graphml.graphdrawing.org
.. _GML: http://www.fim.uni-passau.de/en/fim/faculty/chairs/theoretische-informatik/projects.html

The input format of the graph will be detected from the extension of the file
name by deafult; see above for the recognised extensions.  For the GraphML and GML
formats, vertex names must be provided in the ``name`` vertex attribute. If no
such attribute is present, vertices will use numeric IDs from 0 to *n*-1, where
*n* is the total number of vertices.

If the format autodetection fails (i.e. ``netctrl`` detects the format incorrectly
or it is not able to decide on the format at all), you can help ``netctrl`` out
by specifying the input format manually using the ``-f`` or ``--input-format``
option.

Output formats
==============

The output format is relevant only if ``netctrl`` is running with ``--mode graph``.
In this case, you can choose between the GraphML_ and GML_ output formats; the
annotated graph will be printed in whichever format you choose and the
node and edge metadata will be attached as attributes in the chosen format.
Note that the other formats listed in the `Input formats`_ section do not support
node and edge attributes, hence they are not suitable as output formats.

Bugs, questions?
================

Have you found a bug in the code? Do you have questions? Let me know.
I think you are smart enough to figure out my email address by googling
for my name. Or just drop me a message on GitHub.

Bibliography
============

.. [1] Liu YY, Slotine JJ and Barabási AL: Controllability of complex
       networks. *Nature* **473**:167-173, 2011.

.. [2] Nepusz T and Vicsek T: Controlling edge dynamics in complex
       networks. *Nature Physics*, **8**:568-573, 2012.

