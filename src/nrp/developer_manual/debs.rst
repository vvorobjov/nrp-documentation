.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

.. _debs-developer-manual:

NRP Debian packages
======================================

NRP uses several heavy for compilation C++ projects, that are changed quite rarely. In order to speed-up Docker images build process, we archive the precompiled binaries of these projects into Debian package. Since NRP widely uses Ubuntu as a host OS, these Debian packages are maintained then by Apt manager.

Generation of the package
--------------------------

For the generation of the ``.deb`` archieve, the ``CPack`` extension of the ``CMake`` is used. Read more about ``CPack`` in the `official documentation <https://cmake.org/cmake/help/latest/module/CPack.html>`__.

We use only CPack DEB generator, although many others are available. 

CPack uses the CMake variables and git repository state to create the full package name, which consists of the project name, version and architecture. 

Versioning of the package
+++++++++++++++++++++++++

The most tricky is the determination of the package version. It consists of the project version itself (i.e. Gazebo v. 11), the NRP version, the NRP fork branch name and the number of the commits in this branch. Such complicated way is used to be able to utilize the generated packages in the development CI/CD process.

Thus, the schema of the version is the following::

    <project version>.hbp.<nrp version>-<distace to tag>-<branch name>[-tmp][-pr]

where

* ``<project version>`` is the version of the forked project itself (i.e. ``11`` for Gazebo v. 11);
* ``<nrp version>-<distace to tag>`` is the version of the NRP, which is taken from the git repository tag, and the number of commits to this tag (distace), produced by command ``git describe --tags --always | sed 's/-[^-]*$//'``;
* ``<branch name>`` is the branch name in the forked git repository;
* ``-tmp`` suffix is optional, which is used only for non-master and non-development branches;
* ``-pr`` suffix is optional, which is used only for the builds, corresponding to Bitbucket pull requests.

For example, the valid ``Gazebo`` version is ``11.0.0.hbp.3.1.0-1-development``.


Package contents
+++++++++++++++++

CPack includes into the archive all the files copied by ``install`` target. 

The description of the resulting package is defined by corresponding variables inside CMake (see `official documentation <https://cmake.org/cmake/help/latest/module/CPack.html>`__).

.. warning:: the destination of the files inside the DEB package is the same as specified in ``install`` target, i.e. determined by ``CMAKE_INSTALL_PREFIX`` parameter.



Storage of the package
--------------------------

The packages built during the CI process are then in NRP Apt repository, that is utilized then during the Docker image build process.
