..  sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

.. _nrp-glossary:

NRP Glossary
============

..  glossary::
    :sorted:

    :abbr:`NRP (Neurorobotics Platform)` : Neurorobotics Platform
    Platform : Neurorobotics Platform
    Neurorobotics Platform
        The Neurorobotics Platform's purpose is to offer neuroscientists a tool to perform in-silico cognitive or lower-level neural experiments on virtual Guinea pigs, be them biologically inspired or not. It will, on the other side, provide roboticists with the possibility to experiment on their robots with brain models instead of classical controllers.

    Docker
        `Docker <https://docs.docker.com/get-started/overview/>`__ is a set of platform as a service (PaaS) products that use OS-level virtualization to deliver software in packages called containers. Containers are isolated from one another and bundle their own software, libraries and configuration files; they can communicate with each other through well-defined channels. Because all of the containers share the services of a single operating system kernel, they use fewer resources than virtual machines. 

    Gazebo simulator : Gazebo
    `Gazebo <http://gazebosim.org/>`__
        Gazebo is an open-source 3D robotics simulator. 

    SMACH
        is a task-level architecture for rapidly creating complex robot behavior. At its core, SMACH is a ROS-independent Python library to build hierarchical state machines. SMACH is a new library that takes advantage of very old concepts in order to quickly create robust robot behavior with maintainable and modular code.

    BIBI
        Brain and Body Integrator, the configuration of transfer functions through a simple and eventually graphical syntax. See :ref:`cle-bibi-configuration` for details.

    Brain comm. adapter
        The adapter that is used by the TF framework to connect with the neuronal simulation, e.g. to create suitable devices to connect with the parameters of a TF
    
    :abbr:`CLC (Closed Loop Controller)` : Closed Loop Controller
    Closed Loop Controller
        The Closed Loop Controller, for the reference see :ref:`CLE architecture<cle-architecture>`

    :abbr:`CLE (Closed Loop Engine)` : Closed Loop Engine
    Closed Loop Engine
        The Closed Loop Engine is the middleware putting all the pieces of the Neurorobotics Platofmr together on the server. In particular, it is connecting :term:`Gazebo` and :term:`NEST` through our Transfer Functions mechanism. For more information on CLE, use the dedicated :ref:`tutorials <cle-tutorial-tf>` and :ref:`developer pages<cle-developer-manual>`. The architecture of the CLE can be found :ref:`here<cle-architecture>`.

    Communication Object
        Communication object is the generalization of neuronal network devices and robot publishers and subscribers. Thus, it represents objects that are accessed by the TF framework to connect parameters of a TF with a simulation in either way.

    Device
        In neuronal simulation, devices are little programs that are injected into a neuronal network and run with the same clock as the neuronal simulation and can be accessed from outside. A typical example is a leaky integrator that basically returns the voltage of a neuron. The brain adapters of the CLE e.g. to PyNN do inject such devices into the neuronal network. However, within the TF framework, we also refer to the adapter objects that connect these devices with the TF framework devices, so we identify these adapters with the devices that they adapt. Devices may be either spike sinks or spike sources, i.e. either consume spikes of connected neurons or create spikes (or currents) and send them to connected neurons. Examples of spike sinks are leaky integrators that are essentially neurons that do not spike (infinite threshold voltage) but whose voltage is then accessed by the robot. Examples of spike sources are either current generators (AC, NC or DC source) or Poisson based spike generators.

    Device Group
        For brain simulators, it is often infeasible to work with single devices but whole groups. Consider for example an image recognition. If every pixel would be a spike generator device, the TF would need a number of parameters depending on the image resolution. A device group is a group of such devices that groups all these devices that logically belong together.

    NEST 
        NEST is the neuronal simulator that we currently use by default, see http://www.nest-initiative.org/

    PyNN
        An interface for neuronal simulators, see http://neuralensemble.org/PyNN/

    Robot Publisher
        A robot publisher is the equivalent of a spike source device on the robot side, but only for sending data to the robot. As we are currently using ROS, robot publishers are really ROS publishers sending data to some Gazebo topics.

    Robot Subscriber
        A robot subscriber is the equivalent of a spike sink device, i.e. it is a port for the incoming data.

    Robot comm. adapter
        The adapter that is used by the TF framework to connect with the robot simulation, e.g. to create suitable robot subscribers and robot publishers in accordance with the used input.
        
    TF : Transfer Function
    Transfer Function
        A function that interconnects the neuronal simulator with a (currently simulated) robot. This includes the function itself as well as annotation how to connect its parameters to the neuronal simulation or to the robot simulation. Thus, TFs are end to end and cannot be stacked together. However, their functional specification (the body) can be stacked.

    TF node
    TF manager
        An organizational unit for the TFs. The terms TF node and TF manager are used interchangeably. Each TF must be connected to exactly one TF manager that manages its execution. By default, this is the currently active instance.

    WSE
        World Simulation Engine, the generalization of the robot simulation. We currently use Gazebo (see http://gazebosim.org/) through a ROS (see http://www.ros.org/) interface as our World Simulation Engine.

    REST
        Representational State Transfer, design principle for web interfaces

    rosbridge
        Web interface for ROS using WebSockets, see http://wiki.ros.org/rosbridge_suite

    gzweb
        Web interface for Gazebo using WebGL. Consisting of gzbridge (server) and gz3d (client), see https://bitbucket.org/osrf/gzweb

    ExD
        Experiment (Designer) Backend

    Virtual Coach
        The :ref:`Virtual Coach <virtual_coach_intro>` is a Python API that allows you to run and interact with experiments by scripting them instead of having to use the Web Cockpit. Find more information on the Virtual Coach in the dedicated :ref:`developer pages<virtual_coach_dev_space>`, :ref:`tutorials<virtual-coach-tutorials>` and :ref:`code API reference<virtual-coach-api>`.

    HBP : Human Brain Project
    Human Brain Project
        The Human Brain Project (HBP) is one of the three FET (Future and Emerging Technology) Flagship projects. Started in 2013, it is one of the largest research projects in the world . More than 500 scientists and engineers at over than 140 universities, teaching hospitals, and research centres across Europe come together to address one of the most challenging research targets – the human brain. `Read more about HBP <https://www.humanbrainproject.eu/en/about/overview/>`_.

    OIDC
        OpenID Connect is an authentication layer on top of OAuth 2.0, an authorization framework. In the NRP it used to authenticate you through HBP services. About the accessing NRP read :ref:`here <access-nrp>`.

    Frontend : NRP Frontend
    Web Cockpit : NRP Frontend
    NRP Frontend
        The browser-based user interface for interaction with the NRP. Use the :ref:`section in the user manual <web-cockpit-manual>` in order to get familiar with it. Developers might look at ``ExDFrontend`` in :ref:`nrp-repos`, which implements the main functionality of the Frontend.

    simulation
        is an instance of an :term:`experiment`, launched by a particular user, at a certain time, with a predefined.

    experiment
        is a use case, combining a brain, a robot and an environment.

    Husky robot
        Popular robot platform, i.e., you can read `ROS wiki <http://wiki.ros.org/Robots/Husky>`_

    Braitenberg Vehicle
        A Braitenberg vehicle is a concept conceived in a thought experiment by the Italian-Austrian cyberneticist Valentino Braitenberg: `Wikipedia <https://en.wikipedia.org/wiki/Braitenberg_vehicle>`_
        