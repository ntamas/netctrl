=======
netctrl
=======
--------------------------------------------------------
Experiments with the controllability of complex networks
--------------------------------------------------------

:Author: Tamas Nepusz
:Version: 0.1
:License: MIT

This program implements algorithms that search for driver nodes in complex
networks in order to make them (structurally) controllable. The program
currently implements the controllability model of Liu et al [1]_. Other
models might be added later.

Requirements
============

  - The most recent, up-to-date, bleeding-edge version of igraph_. Really.
    You need at least igraph_ 0.6 because that's where the bipartite
    matching algorithm was added. At the time of writing, igraph_ 0.6
    has not been released yet,, so you have to download and compile it
    yourself from their `Launchpad repository`_.

  - CMake_ to generate the makefiles (or the project file if you are using
    Visual Studio).

.. _igraph: http://igraph.sourceforge.net
.. _Launchpad repository: http://launchpad.net/igraph/
.. _CMake: http://www.cmake.org

Compiling from source code
==========================

These instructions are for Linux or Mac OS X and assume that igraph_ is
installed in a way that CMake can figure out automatically where it is.
(This usually involves using ``pkg-config``)::

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

Usage
=====

TODO

Bugs, questions?
================

Have you found a bug in the code? Do you have questions? Let me know.
I think you are smart enough to figure out my email address by googling
for my name. Or just drop me a message on Github.

BIbliography
============

.. [1] Liu YY, Slotine JJ and Barabási AL: Controllability of complex
       networks. Nature 473:167-173, 2011.
