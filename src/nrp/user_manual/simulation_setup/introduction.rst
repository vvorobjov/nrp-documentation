Introduction
============

The possibility of creating a multitude of experiments is a crucial success factor for any simulation platform. The
Neurorobotics Platform has been designed to support a wide range of possible experiments. Flexibility, however,
usually comes at the price of complexity. The experiment configurations in the NRP are complex artifacts
consisting of multiple settings. We have done our best to make this specification modular to allow
users to reuse existing configurations wherever possible.

This means that you can reuse existing robots from a wide range of robot models available in either the Gazebo
SDF format or ROS URDF format. Existing virtual environments defined in SDF or point-neuron neural network
models specified as PyNN scripts can also be reused and you can share your connection between a robot and a neural network across
multiple experiments and thus evaluate your neural controller in multiple scenarios.

As a consequence, a large part of the experiment description, the robot, the network and the connection between the two of them,
are described in a separate file that is referenced from the experiment description, this is called the BIBI configuration file.
This BIBI configuration then references the robot, the neural network and transfer functions detailing the connection
between both.

In the remainder of this section, we assume that the user operates on an existing robot and neural network.
The robot may originate from the robot designer, the neural network can be edited within the NRP but we encourage users to
use dedicated tools for this task. We will first describe the specification of transfer functions and how the
connections between neural networks and robots are created. Next, we describe the usage of transfer functions together with the online brain editor
as there are a few points to take into consideration when altering the neural network model at runtime.
We end by describing the experiment configuration on top of the BIBI configuration, especially regarding automating a simulation.

