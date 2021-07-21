============
Welcome
============

.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

Thank you for giving an interest in the :term:`Neurorobotics Platform`. In this documentation, we will try to make you familiar with the :abbr:`NRP(Neurorobotics Platform)` and guide you to start working with the platform.

.. rubric:: General goals

The :term:`Neurorobotics Platform`'s purpose is to offer neuroscientists a tool to perform in-silico cognitive or lower-level neural experiments on virtual Guinea pigs, be them biologically inspired or not. It will, on the other side, provide roboticists with the possibility to experiment on their robots with brain models instead of classical controllers.

The platform will provide them with:

- Virtual bodies (robots, bio-inspired)
- Virtual environments to make them live
- Interactive tools to connect these to a brain model
- Interactive tools to design and program their experiment
- Graphical monitors and loggers

.. rubric:: NRP usage options


You can either use the online NRP, getting benefits from the high-performance computing and cloud services, or install NRP locally on your machine (from the source or in :term:`Docker`).

Criteria for version selection:

+------------------+----------------------------------------------------------------------------+-------------------------------------------------------------------------------------------+-------------------------------------------+
|                  |Online platform                                                             |Local install (docker)                                                                     |Source install                             |
+==================+============================================================================+===========================================================================================+===========================================+
|Pros              |* Online, no installation                                                   |* Easy installation                                                                        |* Full flexibility                         |
|                  |* Always up-to-date, no maintenance                                         |* No software dependencies                                                                 |* Completely extendable and tunable        |
|                  |* A number of template experiments                                          |* Works on Linux, Windows and Mac                                                          |* Easily updatable                         |
|                  |* Servers available 24/7                                                    |* Isolated from your other software                                                        |                                           |
|                  |                                                                            |* Runs 100% on your computer                                                               |                                           |
|                  |                                                                            |* Auto-updatable                                                                           |                                           |
|                  |                                                                            |* Access to lower-level tools still possible by accessing containers                       |                                           |
+------------------+----------------------------------------------------------------------------+-------------------------------------------------------------------------------------------+-------------------------------------------+
|Cons              |* Impossible to add additional python packages for use in transfer functions|* Changes in containers are lost on updates (not in Models or Experiments folders though)  |* Tedious installation, many dependencies  |
|                  |* Impossible to upload custom Gazebo plugins (yet)                          |* Update might fail if Models or Experiments have been too much hacked                     |* Works only on Linux Ubuntu 20.04         |
|                  |                                                                            |* Customization is possible but tedious inside container                                   |* Might conflict with existing software    |
|                  |                                                                            |                                                                                           |* Has to be kept up-to-date manually       |
+------------------+----------------------------------------------------------------------------+-------------------------------------------------------------------------------------------+-------------------------------------------+
|Installation      |0                                                                           |5-10 min                                                                                   |3 hours                                    |
|Time              |                                                                            |                                                                                           |                                           |
+------------------+----------------------------------------------------------------------------+-------------------------------------------------------------------------------------------+-------------------------------------------+
|Recommended       |Seminars, courses, prototyping                                              |Most users with a powerful computer                                                        |Code contributors                          |
|for               |                                                                            |                                                                                           |                                           |
+------------------+----------------------------------------------------------------------------+-------------------------------------------------------------------------------------------+-------------------------------------------+

Firefox and Chrome are currently actively supported on Windows, Mac and Linux, for both the Online platform and the local install (Docker). IOS and Android and not officially supported yet, though it mostly works with the online platform.
+----------------------------+---------------+----------------+--------------+--------------+
|Supported OS\\Browser       |Chrome         |Firefox         |Safari        |IE - Edge     |
+============================+===============+================+==============+==============+
|Windows (8/10)              |OK             |OK              |NO            |NO            |
+----------------------------+---------------+----------------+--------------+--------------+
|Linux (Ubuntu 20.04)        |OK             |OK              |n.a.          |n.a.          |
+----------------------------+---------------+----------------+--------------+--------------+
|Mac OS (X)                  |OK             |OK              |NO            |n.a.          |
+----------------------------+---------------+----------------+--------------+--------------+
|Android (11)                |NO             |NO              |n.a.          |n.a.          |
+----------------------------+---------------+----------------+--------------+--------------+
|IOS (IPad)                  |NO             |n.a.            |NO            |n.a.          |
+----------------------------+---------------+----------------+--------------+--------------+

Online service
++++++++++++++++++++++++

In order to access the online :abbr:`NRP (Neurorobotics Platform)`, you need to have the EBRAINS account.
These accounts can be requested on the `account page`_ on our website.

The online version has the least hardware requirements. The computer with 4GB of RAM, slow processor and low-end graphics chipset is enought for running the NRP in a minimal graphic mode.

Local installation
++++++++++++++++++++++++

If you still want to install the NRP on your machine, use :ref:`our pre-built Docker images <docker-installation>`. You can also :ref:`build the NRP from the source <source-installation>` (for advanced users and developers).

The local installation will require at least 8GB of RAM and a large drive space (20GB minimum). For a better performance we recommend to use more than 16GB of RAM, fast processor and good graphic chipset.


.. rubric::Organization of the manual

This manual will help the users to get comfortable with the Experiment Simulation Viewer. It is divided into chapters that cover the main features. The manual does not have to be read linearly.

.. include:: contact-and-support.rst

.. rubric:: Release Notes
  
..  toctree::
    :maxdepth: 1

    release_notes.rst
