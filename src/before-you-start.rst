
================
Before you start
================

Components
==========

The main part of the NRP is the :doc:`NRP-Core <nrp-core/index>`, which represents a stand-alone application, managing the simulation and binding different simulators together. It determines the workflow of the NRP. For a better user experience, there are other components, which bind NRP-Core with a broser-based application, providing a user-friendly access to simulation workflow and simulation files:

    *   **NRP-Core**, the heart of the NRP.
    *   **NRP-Frontend**, the web application.
    *   **NRP-Backend**, the server, managing the simulations.
    *   **MRP-Proxy**, the component, binding frontend. backend and storage.

The main view of the NRP-Frontend application for managing the experiments:

..  thumbnail:: _images/frontend.png
    :align: center
    :title:
    
    The main view of the NRP-Frontend application for managing the experiments.


    


Installation
=============

Currently, you can either use the NRP installed from the source, or running in Docker containers locally on your machine.

.. toctree::

   installation-docker.rst
   installation-source.rst
