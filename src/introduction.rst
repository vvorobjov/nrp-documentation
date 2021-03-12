============
Introduction
============

.. todo:: Add author/responsible

General goals
-------------

The Neurorobotics Platform's purpose is to offer neuroscientists a tool to perform in-silico cognitive or lower-level neural experiments on virtual Guinea pigs, be them biologically inspired or not. It will, on the other side, provide roboticists with the possibility to experiment on their robots with brain models instead of classical controllers.

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
|Linux (Ubuntu 14.04, 16.04) |OK             |OK              |n.a.          |n.a.          |
+----------------------------+---------------+----------------+--------------+--------------+
|Mac OS (X)                  |OK             |OK              |NO            |n.a.          |
+----------------------------+---------------+----------------+--------------+--------------+
|Android (4.4)               |NO             |NO              |n.a.          |n.a.          |
+----------------------------+---------------+----------------+--------------+--------------+
|IOS (IPad)                  |NO             |n.a.            |NO            |n.a.          |
+----------------------------+---------------+----------------+--------------+--------------+


Hardware requirements are:

- a fairly recent graphics card
- minimum 4Gb of RAM
- a recent browser supporting OpenGL
- check that hardware acceleration is enabled. In Firefox, type about:support in the URL bar, and check under "Graphics", it should say "GPU Accelerated Linux: 1/1", else check
  in the browser documentation for support. In Chrome, the URL is chrome://gpu


Release 2.0 (March 2018)
--------------------------------------------------------
Release 2.0 enhances usability and enables to create experiments from scratch.

- Yet bigger resources (120+ servers)
- Models graphical libraries
- Sensor noise graphical library
- Structured transfer functions editor
- Interact with the robot
- Robot inspector, ROS terminal, help tips
- Spinnaker free run integration
- Performance improvements

Release 1.3 (October 2017)
--------------------------------------------------------
Release 1.3 improves on the stability of the platform and adds several new features.

The new features include:

- New virtual lab (Holodeck)
- Improved Environment Designer
- Frontend Performance Monitor
- Demo simulations on Amazon cloud servers for basic users
- User installable platform
- Pilot experiments

Release 1.2 (April 2017)
---------------------------------------
Release 1.2 comes with a more stable platform and new template experiments.

The new features include:

- A new brain visualizer, where you can view the neuron populations in various geometrical setups, as well as the spiking neurons in real time
- A new graphical functions editor, where you can view and manipulate the transfer functions while being able to view all the available ROS topics
- Support for bigger brain models
- A new Python API (Virtual Coach) for batch simulations that can be used to launch simulations without a frontend client
- A graphical object scaling capability
- Streaming of robot camera images 
- Environment enhancements
- New experiments available

  + Husky with neuronal red detection : the color detection is done with a neuronal image recognition.

Release 1.1 (October 2016)
---------------------------------------
Release 1.1 offers better stability, higher performance, a new navigation mode, several new experiments and improved graphics.

New features include:

- a new log console to help in debugging transfer functions and state machine scripts.
- a new 3D environment panel where you can apply color filtering, skyboxes, ambient occlusion and more.
- support for high quality 3D materials through PBR (Physically Based Rendering).
- more responsive user interface, thanks to the introduction of a proxy server.
- faster loading time due to optimized 3D models.
- avatar navigation.
- new experiments available:

  + Force based joint control : a simple example of a force based spiking interface for the Schunk SVH robotic hand
  + HoLLiE hand motion with CPG: a basic implementation of CPG driven motion control for the five fingers of a robotic hand
  + Mouse Braitenberg experiment in biology lab: an experiment that loads the soft-skin mouse model in a biology lab environment
  + Empty template Husky experiment : an empty experiment with no brain that can be used as the basis of an experiment
  + Empty template iCub experiment : an empty experiment with no brain that can be used as the basis of an experiment



The First Public Release (version 1.0)
--------------------------------------

The First Public Release builds on top of the "September" Release and offers more user customizable experiments.

This version provides the following features:

- web experiment simulation frontend
- complete integration in the HBP collaboratory portal
- no additional packages to be installed by users
- multi-user capability
- a user can start or join an experiment
- a user can interact with an experiment, pause, and stop it
- a user can edit the transfer functions (behavior of the robot)
- a user can edit the brain code
- a user can edit the state machine script (automatic environment changes)
- a user can change the environment graphically
- the spikes of the neurons can be displayed in the interface
- the dynamic state of the robot (joints) can be displayed
- experiments are customizable from templates
- template experiments available:

  + the Braitenberg vehicle with a Husky robot in the virtual room
  + the Braitenberg vehicle with a Lauron V robot in the virtual room
  + the Braitenberg vehicle with a Husky robot in the Space Bot Cup field
  + the Braitenberg vehicle with a Lauron V robot in the Space Bot Cup field
  + the eye-tracking experiment with an iCub robot in the virtual room
  + the Braitenberg mouse experiment in the virtual Y-maze
- the user can contact the development team
- the user is invited to fill out a survey form

You are very much encouraged to fill out the survey and to send any bug report through the form_ provided on the `home page`_.

.. _form: https://docs.google.com/forms/d/1rKHSpf_yG0FQgKdfn5Vs__BFfWpd3-F2X8mEi9LH_Dc/viewform
.. _home page: https://www.neurorobotics.net


