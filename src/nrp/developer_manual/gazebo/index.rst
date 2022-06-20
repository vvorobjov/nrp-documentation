.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

.. _gazebo-developer-manual:

Gazebo Developer pages
===========================

:term:`Gazebo` simulates multiple robots in a 3D environment, with extensive dynamic interaction between objects.
This is a **core** physics simulator that is integrated deeply into NRP at this moment.

The NRP ``gazebo`` :ref:`repository <nrp-repos>` contains the fork of the http://gazebosim.org project of version 11.

Installation
------------

The detailed instructions for stand-alone installation with all the dependencies are located at
http://gazebosim.org/install


Installation from source
+++++++++++++++++++++++++

Installation of the Gazebo for the NRP can be performed as a part of :ref:`NRP source installation <source-installation>`. In case the dependencies are installed, then run:

.. code-block:: bash

    # from the ``gazebo`` repository directory
    mkdir build/
    cd build/
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local ..
    make install

SDFormat supported ``cmake`` parameters at configuring time: 

- ``USE_HOST_CFLAGS`` (bool) [default True] Check the building machine for supported compiler optimizations and use them to build the software.
- ``USE_UPSTREAM_CFLAGS`` (bool) [default True] Use the recommend gazebo developers compiler optimizations flags
- ``ENABLE_TESTS_COMPILATION`` (bool) [default True] Enabled or disable the test suite compilation.
- ``USE_LOW_MEMORY_TEST`` (bool) [default False] Use reduced version of tests which need less quantity of RAM memory available.
- ``FORCE_GRAPHIC_TESTS_COMPILATION`` (bool) [default False] Ignore system checks to look for graphic and acceleration support and compile all the test suite.
- ``ENABLE_SCREEN_TESTS`` (bool) [default True] Enable or disable tests that need screen rendering to run properly. Headless machines or machines with the screen turned off should set this to False

Uninstallation
++++++++++++++

To uninstall the software installed with the previous steps: 

.. code-block:: bash

    #from the ``gazebo`` repository directory
    cd build/ 
    sudo make uninstall

Debian package
+++++++++++++++++++++++++

For speed-up of the Docker images build and deploy, we pre-build DEB package for NRP version of Gazebo. We do not distribute this package, because it has limited usage, as soon as we prepare it to be installed into NRP Docker image and consider image's file system structure. You can read :ref:`here <debs-developer-manual>` about the procedure of Debian packages building that we use.

You can also create Debian package for your purposes:

.. code-block:: bash

    # from the ``gazebo`` repository directory
    mkdir build/
    cd build/
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local ..
    make package

The resulting DEB package will be created in the build directory. Note, that the further installation path is determined by ``CMAKE_INSTALL_PREFIX``

.. seealso:: :ref:`NRP Debian packages for C++ projects <debs-developer-manual>`