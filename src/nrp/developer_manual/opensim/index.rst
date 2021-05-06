.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

.. _opensim-developer-manual:

OpenSim Developer pages
===========================

:term:`OpenSim` is a freely available, user extensible software system that lets users develop models of musculoskeletal structures and create dynamic simulations of movement.

The NRP ``opensim`` :ref:`repository <nrp-repos>` contains the fork of the https://github.com/opensim-org/opensim-core project.

Installation
------------


Installation from source
+++++++++++++++++++++++++

Installation of the OpenSim for the NRP can be performed as a part of :ref:`NRP source installation <source-installation>`. In case the dependencies are installed, then run:

.. code-block:: bash

    # from the ``opensim`` repository directory
    mkdir build/
    cd build/
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local -DNRP_SKIP_INSTALL_THIRDPARTY_LIB=1 ..
    make install


Debian package
+++++++++++++++++++++++++

For speed-up of the Docker images build and deploy, we pre-build DEB package for NRP version of OpenSim. We do not distribute this package, because it has limited usage, as soon as we prepare it to be installed into NRP Docker image and consider image's file system structure. You can read :ref:`here <debs-developer-manual>` about the procedure of Debian packages building that we use.

You can also create Debian package for your purposes:

.. code-block:: bash

    # from the ``opensim`` repository directory
    mkdir build/
    cd build/
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local -DNRP_SKIP_INSTALL_THIRDPARTY_LIB=1 ..
    make package

The resulting DEB package will be created in the build directory. Note, that the further installation path is determined by ``CMAKE_INSTALL_PREFIX``

.. seealso:: :ref:`NRP Debian packages for C++ projects <debs-developer-manual>`