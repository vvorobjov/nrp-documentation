============
Welcome
============

.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

General goals
-------------

The :term:`Neurorobotics Platform`'s purpose is to offer neuroscientists a tool to perform in-silico cognitive or lower-level neural experiments on virtual Guinea pigs, be them biologically inspired or not. It will, on the other side, provide roboticists with the possibility to experiment on their robots with brain models instead of classical controllers.

The platform will provide them with:

- Virtual bodies (robots, bio-inspired)
- Virtual environments to make them live
- Interactive tools to connect these to a brain model
- Interactive tools to design and program their experiment
- Graphical monitors and loggers

Organization of the manual
--------------------------

This manual will help the users to get comfortable with the Experiment Simulation Viewer. It is divided into chapters that cover the main features. The manual does not have to be read linearly.

Software and hardware requirements
--------------------------------------------------------

The software has been tested against OS/browsers that are listed below. Other profiles may function correctly, but they have not been tested. In any case, there is no risk for your computer. Just try and enjoy!

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


Hardware requirements are:

- a fairly recent graphics card
- minimum 4Gb of RAM
- a recent browser supporting OpenGL
- check that hardware acceleration is enabled. In Firefox, type about:support in the URL bar, and check under "Graphics", it should say "GPU Accelerated Linux: 1/1", else check
  in the browser documentation for support. In Chrome, the URL is chrome://gpu


Release Notes
--------------------------
..  toctree::
    :maxdepth: 1

    release_notes.rst
