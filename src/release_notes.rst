..  sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

..  _release-notes:

..  contents:: Release Notes:
    :depth: 1



Release 3.1 (January 2021)
--------------------------------------------------------
Release 3.1 comes with updated stack of third-party software.

- Base OS for :ref:`Docker image <docker-installation>` is Ubuntu 20.04 Focal Fossa
- Full Python 3 support
- 3D robotics simulator Gazebo is updated to version 11
- Robot Operating System (ROS) is updated to release Noetic Ninjemys
- Stand-alone :term:`Virtual Coach`



Release 3.0.5 (December 2020)
---------------------------------------------------------

- **Last version with Python 2 support!**
- New Whiskeye robot experiment
- Multi experiment web GUI
- Web ROS Console reimplementation
- Brain simulator selector for new experiments



Release 3.0 (April 2020)
--------------------------------------------------------

- Fluid simulation
- Plotting tools in the frontend
- Human-in-the-loop improvement in the Unity client
- Graphical experiment designe
- Web robot designer
- Improved SpiNNaker integration
- Support of HBP EBRAINS Sonata brain file format and SP6 Cerebellum model
- New models and experiment templates
- Python 3 ready



Release 2.3 (October 2019)
--------------------------------------------------------

- Share experiments and models
- Improved editor layout
- Import and export experiments
- Spinnaker visualizers
- Support for pure NEST brains (no PyNN)
- Integration with the HBP EBRAINS Knowledge Graph



Release 2.2 (April 2019)
--------------------------------------------------------

- Record and replay simulations
- New experiment creation work flow with robots and brain drag and drop
- New models libraries
- Support multiple robots (full)
- Plotting and visualization tools support for Nengo
- Join simulation shared by others
- Custom workspace management



Release 2.1 (November 2018)
--------------------------------------------------------

- Easy local installation using docker installer
- Support Nengo brains (in 2.1.1)
- Changelog informs user of new features and compatibility breaks
- New frontend design with layouts and new toolbars
- Support multiple robots (in progress)
- Code editors auto-persistance (no save button anymore)



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
- A new Python API (:term:`Virtual Coach`) for batch simulations that can be used to launch simulations without a frontend client
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