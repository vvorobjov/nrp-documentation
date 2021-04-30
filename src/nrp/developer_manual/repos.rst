.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

.. _nrp-repos:

NRP Repositories
================

The HBP Neurorobotics Project consists of numerous repositories, which contain the :abbr:`NRP (Neurorobotics Project)` code base, patches to the third-party libraries and software, 3D models, template experiments, settings and service applications. Mainly these repositories are `stored in Bitbucket <https://bitbucket.org/hbpneurorobotics/workspace/projects/NRP>`__. Some of them are private and are available only for core developers. The others are public and you can contribute to the development of the :term:`NRP`, having cloned them to your workspace and creating the pull request with your changes afterwards. 

Below you can find a short description of these repositories.

BrainSimulation
+++++++++++++++++++++++++++++++

`Python project <https://bitbucket.org/hbpneurorobotics/brainsimulation>`__. 
It implements the module :class:`hbp_nrp_distributed_nest` for the distributed brain interface for the :term:`CLE`. You can refer the :ref:`code API documentation <hbp_nrp_distributed_nest-api>` and :ref:`hbp_nrp_distributed_nest-dev-space`.

brainvisualizer
+++++++++++++++++++++++++++++++

`JavaScript project <https://bitbucket.org/hbpneurorobotics/brainvisualizer>`__. 
This is the :term:`Frontend` brain visualizer Bower component.

bulletphysics
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/bulletphysics>`__. 
Bullet is an alternative physics engine we can use in :term:`Gazebo` (but generally we don't). This is a slightly modified fork of the `official repo <https://github.com/bulletphysics/bullet3>`__.

CLE
+++++++++++++++++++++++++++++++

`Python project <https://bitbucket.org/hbpneurorobotics/cle>`__.
Here the module :class:`hbp_nrp_cle` is implemented. The :term:`Closed Loop Engine` is the middle-ware putting all the pieces of the Neurorobotics Platform together on the server. In particular, it is connecting :term:`Gazebo` and :term:`NEST` through our :term:`Transfer Functions <TF>` mechanism.

For the following reference use :ref:`cle-developer-manual` and dedicated :ref:`tutorials <cle-tutorials>`.

ExDBackend
+++++++++++++++++++++++++++++++

`Python project <https://bitbucket.org/hbpneurorobotics/exdbackend>`__.
This repository is implementing the backend side of the NRP through :class:`hbp_nrp_backend`, :class:`hbp_nrp_cleserver`, :class:`hbp_nrp_commons` and :class:`hbp_nrp_watchdog` modules. REST calls and back office logic are handling user request to set up a new simulation. It is also handling the simulation live flow using states.

For the following reference use 

    * :ref:`backend_dev_space` - :class:`hbp_nrp_backend`
    * :ref:`cleserver_dev_space` - :class:`hbp_nrp_cleserver`
    * :ref:`hbp_nrp_commons_dev_space` - :class:`hbp_nrp_commons`
    * :ref:`hbp_nrp_watchdog_dev_space` - :class:`hbp_nrp_watchdog`

and dedicated :ref:`tutorials <exdbackend-tutorials>`.

ExDFrontend
+++++++++++++++++++++++++++++++

`JS project <https://bitbucket.org/hbpneurorobotics/exdfrontend>`__.
The Frontend application (:term:`Web Cockpit`) of the Neurorobotics Platform and all required Bower components.

ExperimentControl
+++++++++++++++++++++++++++++++

`Python project <https://bitbucket.org/hbpneurorobotics/experimentcontrol>`__.
This repository contains :class:`hbp_nrp_excontrol`, which implements the state machine logic of the `CLE`_ and `ExDBackend`_.

Experiments
+++++++++++++++++++++++++++++++

`This repository <https://bitbucket.org/hbpneurorobotics/experiments>`__ holds the template experiments and the XSD schemas for project files.

frontendStateMachineEditor
+++++++++++++++++++++++++++++++

`JS project <https://bitbucket.org/hbpneurorobotics/frontendstatemachineeditor>`__.
This repository implements the experiment designer's graphical editor.

gazebo
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/gazebo>`__. 
:term:`Gazebo` is our world and robot simulation engine. This is a modified fork of the official repository.

GazeboRosPackages
+++++++++++++++++++++++++++++++

`ROS packages project <https://bitbucket.org/hbpneurorobotics/gazeborospackages>`__. 
The gazebo ROS packages and plugins that we use in the NRP.

gzweb
+++++++++++++++++++++++++++++++

`Mixed project <https://bitbucket.org/hbpneurorobotics/gzweb>`__.
The gzbridge Gazebo-NRP communication layer + frontend assets + frontend :term:`gzweb` code (provided duplicately in `ExDFrontend`_ as a Bower component).

hbpneurorobotics.bitbucket.io
+++++++++++++++++++++++++++++++

.. todo:: Add description of the repository

Models
+++++++++++++++++++++++++++++++

`This repository <https://bitbucket.org/hbpneurorobotics/models>`__ holds all NRP template models: robots, environments, brains, and their respective XSD schemas.


MUSIC
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/music>`__. 
MUSIC is a brain simulator communication library that we might use for brain distribution, but not yet. This is a modified fork of the official repository.

mvapich2
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/mvapich2>`__.
Mvapich2 is the MPI implementation we use for the distributed BrainSimulation. This is a modified fork of the official repository.

nest-simulator
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/nest-simulator>`__.
Nest is our main brain simulator. It is used over ``PyNN`` in the `CLE`_. This is a modified fork of the official repository.

Neurorobotics Platform
+++++++++++++++++++++++++++++++

This is a `meta-repository <https://bitbucket.org/hbpneurorobotics/neurorobotics-platform>`__ holding the installation documentation to the Neurorobotics Platform source repositories and the changelogs. The corresponding installation instructions can be found in the :ref:`current guides <source-installation>` as well.

nrpBackendProxy
+++++++++++++++++++++++++++++++

`TypeScript project <https://bitbucket.org/hbpneurorobotics/nrpbackendproxy>`__.
The NRP Backend Proxy is handling backend server management for the clients, passing initial calls over. It is also acting as a storage server, handling user experiments and models persistence. Look at the :ref:`developers page <nrp-backend-proxy-dev>` for this repository.

nrp-core
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/nrp-core>`__.
The future core NRP multi-simulation synchronization component. 

opensim
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/opensim>`__.
Opensim is used by Gazebo to simulate muscles. This is modified fork from the official opensim repository.

retina
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/retina>`__.
Retina is a library used in experiments with retina-based vision (ICubs for example). It is optional.

sdformat
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/sdformat>`__.
Sdformat is the library handling SDF files for Gazebo. This is a modified fork of the official repository.

server-scripts
+++++++++++++++++++++++++++++++

Start `scripts <https://bitbucket.org/hbpneurorobotics/server-scripts>`__ for the CLE (gzserver, rosbridge, ...).

simbody
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/simbody>`__.
Simbody is used by Gazebo to provide the physics layer for opensim. This is a modified fork from the official repository.

SlurmClusterMonitor
+++++++++++++++++++++++++++++++

SPlisHSPlasH
+++++++++++++++++++++++++++++++

`C++ project <https://bitbucket.org/hbpneurorobotics/SPlisHSPlasH>`__.
This is our fork of the `SPlisHSPlasH <https://github.com/InteractiveComputerGraphics/SPlisHSPlasH>`__ particle physics simulator that we integrate in Gazebo.

user-scripts
+++++++++++++++++++++++++++++++

`This repository <https://bitbucket.org/hbpneurorobotics/user-scripts>`__ contains configuration and install scripts and resources for the NRP.

VirtualCoach
+++++++++++++++++++++++++++++++

`Python project <https://bitbucket.org/hbpneurorobotics/VirtualCoach>`__.
This repository contains :class:`pynrp`, which is a python scripting client for the NRP server. Refer these pages for more information :ref:`developer page, <virtual_coach_dev_space>`, :ref:`tutorials<virtual-coach-tutorials>` and :ref:`user manual<virtual_coach_intro>`
