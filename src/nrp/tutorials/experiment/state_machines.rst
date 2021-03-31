.. _state_machines:

Tutorial: Writing a state machine for an experiment
===================================================

.. todo:: Add author/responsible
.. todo:: Consider duplication :doc:`/nrp/modules/ExDBackend/hbp_nrp_backend/tutorials/state_machines`, https://hbpneurorobotics.atlassian.net/l/c/iHd8of31



The NRP is designed to use state machines to control and evaluate experiments. A state machine controlling an experiment can monitor any simulation properties published on ROS topics (e.g. simulation time, sensor output, spiking activity of brain), it can also publish on ROS topics or call ROS services. A state machine evaluating the success of an experiment has the same capabilities for monitoring a simulation and should either publish a success or failure message to the NRP ROS status topic.

The NRP uses SMACH for executing state machines, hence state machines have to be specified in Python.


Controlling an experiment
-------------------------

The following examples show how to specify state machines for certain tasks.

Trigger an action after 30 seconds of simulation time
"""""""""""""""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/time_event.py
   :language: python

This basic example contains a simple state machine which waits for 30s of simulation time to pass and then sets the color of the right screen to red. It demonstrates the purpose of state machines in the platform.

.. note::
    The state machine will be run automatically in the platform. For this to work, the variable holding a reference to
    the state machine must be named *sm* or *state_machine*.

Trigger an action based on user interaction
"""""""""""""""""""""""""""""""""""""""""""

.. note::
    Currently, the ExD back-end directly executes ROS services upon receiving user interactions/events. In the future the name of the user interaction should be published on a ROS topic to allow the control state machine to execute the user interaction/event.


Trigger an action based on a monitored property
"""""""""""""""""""""""""""""""""""""""""""""""

The MonitorState function allows us to monitor any available ROS topic. Thus, monitoring a property can be done in the same way as monitoring simulation time. The generic ROS Monitor state can be used, but we recommend the usage of a specific
state implementation for monitoring spike rates.

.. literalinclude:: sm_examples/spike_event.py
   :language: python

In this example a neuron monitor, set up in the according BIBI configuration file, is used to monitor brain activity and uses it as condition for an event.

Client Logging
--------------

'Client logs' is a logging mechanism that aims to help users develop and debug their state machines and transfer functions.
All triggered client logs will be visible by the user in a specific window from within the running simulation.

Triggering client logs
""""""""""""""""""""""

The ClientLogger allows the user to trigger client logs in order to help the debugging of state machines.

.. literalinclude:: sm_examples/client_logger.py
   :language: python

In this example, we are interested in having a state machine that will change state when the robot is at the right or left limit of the scene.
We know that the RobotPoseMonitorState allows us to have a condition on the robot coordinates (x,y,z), but we don't know exactly what values correspond to positions we want to watch for.
By using the clientLogger in that function watching the robot position, we can log the (x,y,z) values as we drive the robot in the scene to the target positions.
We can then collect the values at the target position and use them in the state condition.


State client logger
"""""""""""""""""""

The ClientLogState is a state that triggers a client log.

.. literalinclude:: sm_examples/state_client_logger.py
   :language: python

In this example, we are using the positions we collected in the section above and we expect the state machine to change state as the robot position alternates from left to right.
To test the validity of our parameters, we've added the ClientLogState("Husky is at the LEFT/RIGHT!") states that will log messages as the state machines transitions from one state to the other.
By moving the robot around and observing the log messages ("Husky is at the LEFT/RIGHT!") appear in the simulation, the user can verify that the transitions occur at the expected robot position.

Trigger an action based on a monitored client log
"""""""""""""""""""""""""""""""""""""""""""""""""

The WaitForClientLogState function allows us to monitor for a specific client log.
The action is triggered when a log message contains the given string. The check is case sensitive.

.. literalinclude:: sm_examples/client_logger_watcher.py
   :language: python

In this example, we want the state machine to change its state when a specific client log containing the keywords 'left_tv_red' or 'left_tv_blue' is observed.

Combined events
---------------

The following examples show how to combine several events into a single state machine.

Multiple time events as sequence
""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/multiple_time_events.py
   :language: python

In this example the first event is triggered after 30 seconds and the second one after 60 seconds. The actions of both events are implemented by `CBState <http://wiki.ros.org/smach/Tutorials/CBState>`_, simply executing a callback.


Multiple conditions for a single event
""""""""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/multiple_conditions_event.py
   :language: python

This example shows how to combine multiple conditions for a single event by using a `Concurrence Container <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`_. Both conditions, simulation time larger than 30 seconds and brain activity above threshold, have to be met. The order the conditions become true doesn't matter.


Multiple independent events in a single state machine
"""""""""""""""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/multiple_events_sm.py
   :language: python

This example shows how to combine multiple events in a single state machine by nesting `State Machine Containers <http://wiki.ros.org/smach/Tutorials/StateMachine%20container>`_ (events) into a `Concurrence Container <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`_. The concurrence container terminates a soon as both nested state machines (events) terminate.


Evaluating experiment result
----------------------------

A state machine for evaluating the success of an experiment does not differ from an experiment control state machine in terms of capabilities. However, it should have exactly two final states: one publishing a 'success' message on the CLE status topic and one publishing a 'failure' message.

.. note::
    Currently, the frontend is not able to visualize success or failure messages.


Special States
--------------

The following examples demonstrate the application of some special state implementations that are designed to make controlling an experiment easier.

Spawn Objects
"""""""""""""

The NRP has an integrated state implementation class **ModelSpawnServiceState**, designed to spawn objects dynamically from a state machine. As the name suggests, it can be used to spawn simple objects.
This state supports the following parameters:

:model_name: The name of the newly spawned object. This can either be a constant string, a function or a SMACH callback. In the latter cases, it must accept two arguments: The user data of the state chart and the last model name. The first last model name is *None*.
:geometry: The geometry of the newly spawned object in an SDF format
:position: The position at which the object should be spawned. This is specified in world coordinates. Alternative, a function or a SMACH callback can be provided, again using two arguments for the state chart user data and the last position.
:rotation: The rotation of the spawned object. This must be constant.
:can_collide: A flag indicating whether the object can collide with other objects
:gravity_factor: This specifies to which degree gravity should affect the newly spawned object. The state also allows True (full gravity=1) or False (no gravity=0). Default is 1.
:kinematic: A factor specifying the kinematics. This can also be True=1 or False=0. By default, kinematics are enabled.
:mass: The mass of the spawned object. By default, this is 1.
:color: A Gazebo color name. By default, the object is spawned in grey (*Gazebo/Grey*)

In addition, the NRP contains several state implementations to make spawning of simple shapes easier. These states do not allow to specify a *geometry* directly, but rather through other arguments.

* **SpawnSphere** spawns a sphere. An additional *radius* parameter determines the radius of the spawned sphere and defaults to 1m. The default position is (0,0,0), same as the default rotation.  Kinematics and collision are disabled by default.
* **SpawnCylinder** spawns a cylinder. The shape of the cylinder is determined by the *length* and *radius* parameters. Same as spheres, cylinders by default have no kinematics and do not collide.
* **SpawnBox** spawns a box. The size of the box is specified by a vector *size* and defaults to (1,1,1).

Destroy Objects
"""""""""""""""

There is also a special state to easily destroy objects in the scene: **ModelDestroyServiceState** or simpler **DestroyModel**. It accepts exactly one parameter *model_name* which can be either a constant string or a callback function specifying
name of the object to be deleted depending on the state chart user data and the last destroyed model name.

Change Model Position and Orientation
"""""""""""""""""""""""""""""""""""""

There is a separate state to change the position, rotation and scale of any given object. This state is called **SetModelServiceState** or simpler **SetPose**. It accepts the following arguments:

:model_name: The name of the object. This can either be a constant string, a function or a SMACH callback. In the latter cases, it must accept two arguments: The user data of the state chart and the last model name. The first last model name is *None*.
:position: The position to which the object should be moved. This is specified in world coordinates. Alternative, a function or a SMACH callback can be provided, again using two arguments for the state chart user data and the last position.
:rotation: The rotation of the object. This can be either a constant vector or a function returning the latter.
:scale: A factor to which the object should be scaled. Alternatively, a function returning this scaling factor depending on the state chart user data and the previous scale factor is allowed.
