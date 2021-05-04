.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

.. _debs-developer-manual:

NRP Debian packages
======================================

NRP uses several heavy for compilation C++ projects, that are changed quite rarely. In order to speed-up Docker images build process, we archive the precompiled binaries of these projects into Debian package. Since NRP widely uses Ubuntu as a host OS, these Debian packages are maintained then by Apt manager.

.. todo:: add description