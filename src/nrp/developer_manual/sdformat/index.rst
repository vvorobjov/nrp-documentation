.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

.. _sdformat-developer-manual:

Format Developer pages
===========================

SDFormat - :abbr:`SDF (Simulation Description Format)` parser.

SDF is an XML file format that describes environments, objects, and robots in a manner suitable for robotic applications. SDF is capable of representing and describing different physic engines, lighting properties, terrain, static or dynamic objects, and articulated robots with various sensors, and acutators. The format of SDF is also described by XML, which facilitates updates and allows conversion from previous versions. A parser is also contained within this package that reads SDF files and returns a C++ interface.

The NRP ``sdformat`` :ref:`repository <nrp-repos>` contains the fork of the http://sdformat.org project.

Installation
------------

Installation from source
+++++++++++++++++++++++++

Installation of the parser for the NRP can be performed in UNIX systems using the following steps :

.. code-block:: bash

    # from the ``sdformat`` repository directory
    mkdir build/
    cd build/
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local ..
    sudo make install

SDFormat supported ``cmake`` parameters at configuring time: 

- ``USE_EXTERNAL_URDF`` (bool) [default False] Do not use the internal copy of :term:`urdfdom` and use the one installed in the system instead. Recommended if you have a working installation of :term:`urdfdom`. 
- ``USE_UPSTREAM_CFLAGS`` (bool) [default True] Use the SDFormat team compilation flags instead of the common set defined by cmake.

Uninstallation
++++++++++++++

To uninstall the software installed with the previous steps: 

.. code-block:: bash

    #from the ``sdformat`` repository directory
    cd build/ 
    sudo make uninstall

Debian package
+++++++++++++++++++++++++

For speed-up of the Docker images build and deploy, we pre-build DEB package for SDFormat. These package has limited usage, as soon as we prepare it to be installed into NRP Docker image and consider image's file system structure. You can read :ref:`here <debs-developer-manual>` about the procedure of Debian packages building that we use.

You can also create Debian package for your purposes:

.. code-block:: bash

    # from the ``sdformat`` repository directory
    mkdir build/
    cd build/
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local ..
    sudo make package

The resulting DEB package will be created in the build directory. Note, that the further installation path is determined by ``CMAKE_INSTALL_PREFIX```

.. seealso:: :ref:`NRP Debian packages for C++ projects <debs-developer-manual>`