Experiment configuration
========================

Apart from the BIBI configuration, the experiment configuration mainly consists of the following artifacts:

- The experiment metadata refers to the experiment name, description and image to show in the experiment list.
  Here, the image of the experiment is simply the image file with the same name as the experiment description file.
- The timeout of the experiment determines the amount of time in seconds after which a simulation of this experiment
  should be terminated.
- The simulation maturity is used to determine the visibility of the experiment. An experiment with maturity
  production is expected to be bug-free, while an experiment in development may still include bugs.
- A reference to the BIBI configuration file
- The simulated environment in which the experiment is simulated as a reference to an SDF description, initial camera
  position and initial robot position
- Possible automation of the experiment through state machines.


To specify a state machine using SMACH, a script has to be written that includes a module-level variable named sm or state_machine, in
this variable the state machine will be stored. The execution of this script is done in a separate process and the logistic of starting and stopping the state
machine according to the simulation state is then managed by the platform.

Though any existing SMACH state machine can be used, we offer a set of default implementations to simplify the specification
of control in a number of predefined usage scenarios. These state implementations are described below.

Monitoring the simulation time
------------------------------

One of the most straight-forward things to use to trigger state transitions in a state machine is time. To make the experiment
reproducible, simulation time should be used rather than physical time.

This can be done using the two states **ClockMonitorState** and **WaitToClockState**. The **ClockMonitorState** simply monitors the simulated time
while it is active and uses a callback to decide whether a transition should be made. If the callback evaluates to False or an error occurred, the state transitions to invalid,
otherwise to valid (usually transitioning the state to itself). The **WaitToClockState** is a default implementation that simply waits until
a given simulation time is reached.

.. code-block:: python

    smach.StateMachine.add('CONDITION',
                           states.WaitToClockState(30),
                           {'valid': 'CONDITION', 'invalid': 'ACTION',
                            'preempted': 'CONDITION_PREEMPTED'})

An example usage of the **WaitToClockState** is shown in the listing above.

Monitoring neural activity
--------------------------

Monitoring the neural activity is very similar to monitoring the simulated time. There are four state implementations
available, **MonitorSpikeRateState**, **MonitorLeakyIntegratorAlphaState**, **MonitorLeakyIntegratorExpState** and **MonitorSpikeRecorderState**.
Each of these state implementations targets a different type of neural monitor. All of these state implementations take as input the name of the
monitor that should be queried and a callback function for when new monitoring messages come in. This callback function accepts three parameters: The user data of the
state machine, the simulation time for which the monitoring message was sent and the current rate or an array of spikes received in the last period.

If the callback function evaluates to True or a monitoring message was received which was unrelated to the monitor under query, the state transitions to valid, otherwise to invalid.

Monitoring the robot position
-----------------------------

Like monitoring the neural activity, we also support states specifically designed to monitor the robots state, in particular its pose or twist.
For this, the state implementations **RobotPoseMonitorState** and **RobotTwistMonitorState** are used. They accept as a parameter a callback function to which
two parameters are passed: The user dictionary of the state machine and the robots pose or twist at the last simulation time.

If the callback function evaluates to True, the state transitions to valid, otherwise to invalid.

Changing the lights of the virtual environment
----------------------------------------------

States are also able to interact with a simulation. One of the simplest interactions is the interaction with lights in the virtual environment. This interaction is
implemented in the **LightServiceState**. This state accepts as parameters

- the name of the light that shall be changed
- the diffuse (the color) as a RGB list or dictionary
- the constant, linear and quadratic attenuation (each as numbers)

One of the good things about the light state is that besides the name, any of these elements can be omitted. In that case, the state will query the current values
for any missing fields before performing a light change request, so a state machine may only change the color, without changing attenuation at all.

Changing material colors of items in the virtual environment
------------------------------------------------------------

Another interaction with the simulated environment directly supported by state machines is changing the colors of items in the virtual environment. This
is supported through the class **SetMaterialColorServiceState** that accepts the following parameters:

- The object whose material should be modified (for example *right_vr_screen*)
- The part of the object that should be modified (for example *body*)
- The surface that should get a new color (for example *screen_glass*)
- The color to set (for example *Gazebo/Red*)

In parentheses, we printed the values to set the screen color to red in a standard experiment in the virtual room or the SpaceBotCup environment.
