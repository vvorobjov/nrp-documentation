.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>
  
.. _before-you-start:

===============================================
Before you start
===============================================

.. todo:: Add separation by types of access

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

.. include:: authentication.rst

.. include:: contact-and-support.rst


Release Notes
--------------------------
..  toctree::
    :maxdepth: 1

    release_notes.rst
