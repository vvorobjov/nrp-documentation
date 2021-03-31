===========================================================================================
Tutorial: A simple Braitenberg experiment with a Husky robot connected to a Spinnaker board
===========================================================================================

.. todo:: Add author/responsible

In this tutorial, we connect a Spinnaker board to a simulated Husky robot in a Braitenberg-like experiment.
We do this by altering a nest-based Husky Braitenberg experiment and port that to Spinnaker.

Changing the simulator
----------------------

At first, we need to change the simulation assembly to use the Spinnaker software instead of nest to simulate the
neural network. Whereas synchronous, mono-threaded nest is currently set as the default simulation assembly mode,
Spinnaker needs to be specified explicitly. 

To do this, we need to change the BIBI model to reflect this change. The BIBI model does not allow us directly
to specify the used neural network simulator but instead allows us to chose between different simulation modes.
This is to make sure that the selection of components work together well. 

To change the simulation mode, we need to add the following line in the BIBI file directly after the specification
of the robot that should be simulated:

.. code:: xml

    <mode>SynchronousSpinnakerSimulation</mode>

Here, SynchronousSpinnakerSimulation simply means that Spinnaker and the CLE are synchronized *eventually*.
Currently, this is the only supported simulation mode for Spinnaker. This means we run the Spinnaker simulation in
parallel to the gazebo simulation.

An asynchronous Spinnaker simulation is basically a synchronous simulation with asynchronous communication and a
very long CLE timestep. Therefore, the *SynchronousSpinnakerSimulation* configuration is also used for 
an asynchronous simulation using Spinnaker, just using a longer CLE timestep.

Converting neural membrane potentials to robot control commands
---------------------------------------------------------------

Next, we need to make sure that the communication is asynchronous. We start with the conversion of
neural membrane potentials to robot commands. To realize this conversion, we need to integrate spikes of
a given period, add a leak term and then we can use the result of this computation, scaled and transformed, as
robot control command. The leaky integrator device encapsulates the leaky integration of spikes for us.

Below, we depicted the Transfer Function to realize this conversion in the nest-based Braitenberg experiment:

.. code:: python

    @nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
    @nrp.MapSpikeSink("right_wheel_neuron", nrp.brain.actors[2], nrp.leaky_integrator_alpha)
    @nrp.Neuron2Robot(Topic('/husky/cmd_vel', geometry_msgs.msg.Twist))
    def linear_twist(t, left_wheel_neuron, right_wheel_neuron):
        """
        The transfer function which calculates the linear twist of the husky robot based on the
        voltage of left and right wheel neuron.

        :param t: the current simulation time
        :param left_wheel_neuron: the left wheel neuron device
        :param right_wheel_neuron: the right wheel neuron device
        :return: a geometry_msgs/Twist message setting the linear twist fo the husky robot movement.
        """

        return geometry_msgs.msg.Twist(
            linear=geometry_msgs.msg.Vector3(x=20.0 * min(left_wheel_neuron.voltage, right_wheel_neuron.voltage),
                                             y=0.0,
                                             z=0.0),
            angular=geometry_msgs.msg.Vector3(x=0.0,
                                              y=0.0,
                                              z=100.0 * (right_wheel_neuron.voltage - left_wheel_neuron.voltage)))

At first, we need to take note that

* Spinnaker does not support population views, so *actors[1]* will not work
* Spinnaker currently does not support alpha-shaped post-synaptic currents, so we will use the exponential shaped one

To address the first point, we create a separate population for every functional part of the network that we want to interface using Transfer Functions.
To address the latter, we switch to leaky integrators using an exponential decay (since they are available in Spinnaker).

To turn this into an asynchronous communication, we keep using the leaky integrator device type, but
use it as trigger device for the TF. Therefore, the TF is no longer executed by the CLE but whenever there
is a new value for the leaky integrator available.
The implementation of the leaky integrator for Spinnaker even performs the leaky integration right on the Spinnaker chip, which
saves bandwidth.

By default, leaky integrators send the integrated spikes once every 10 timesteps. With the default setting
that a timestep is 1ms, this gives a control rate of 100hz. In this tutorial, we will use a slower control rate of 10hz,
so we set the timesteps to 100.

However, we need to adjust the weight of the leaky integrator, simply because exciting a neuron with exponential decay is much harder
than exciting a neuron with alpha-shaped decay. In experiments, we found that a weight of 2.0 is acceptable.

Therefore, we arrive at the following:

.. code:: python

    @nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.left, nrp.leaky_integrator_exp, weight=2.0, timesteps=100)
    @nrp.MapSpikeSink("right_wheel_neuron", nrp.brain.right, nrp.leaky_integrator_exp, weight=2.0, timesteps=100)
    @nrp.Neuron2Robot(Topic('/husky/cmd_vel', geometry_msgs.msg.Twist), trigger=["left_wheel_neuron"])
    def linear_twist(t, left_wheel_neuron, right_wheel_neuron):
        """
        The transfer function which calculates the linear twist of the husky robot based on the
        voltage of left and right wheel neuron.

        :param t: the current simulation time
        :param left_wheel_neuron: the left wheel neuron device
        :param right_wheel_neuron: the right wheel neuron device
        :return: a geometry_msgs/Twist message setting the linear twist fo the husky robot movement.
        """

        return geometry_msgs.msg.Twist(
            linear=geometry_msgs.msg.Vector3(x=0.02 * min(left_wheel_neuron.voltage, right_wheel_neuron.voltage),
                                             y=0.0,
                                             z=0.0),
            angular=geometry_msgs.msg.Vector3(x=0.0,
                                              y=0.0,
                                              z=0.07 * (right_wheel_neuron.voltage - left_wheel_neuron.voltage)))

The scaling factors are somewhat arbitrary and have been set through try and error. 

.. note:: An alternative to the small scaling factors would be smaller weights for the leaky integrators.
          Due to the current low precision of the leaky integrators, we do not recommend this, as the voltages may then be closer to zero
          and therefore very imprecise.

Converting images to robot commands
-----------------------------------

In the converse direction, we need to transmit the camera image from the robot to the neural network.
The original nest-based setup uses the Poisson spike generators to feed the image data into the network.
However, at the time of writing this tutorial, we still have some connectivity issues. Therefore, in this tutorial,
we are going to use spike injectors, a device type exclusively available for Spinnaker (at the moment, at least).

Spike injectors are devices that simply inject a predefined amount of spikes into an existing neuron population.
For this, the CLE adds a population of specialized neurons into the network that are able to receive commands from the
host and omit a spike whenever they receive a command message. Further, a projection to a neuron population is created that
the spike injector is connected to.

A closed-loop connection using spike injectors has multiple advantages and disadvantages: On the plus, they allow to transmit
rare and discrete events such as collisions to the neural network. On the contrary, if used to transmit (logically) continuous signals such
as a camera image (that are only discretized for technical reasons), the closed loop massively depends on the frequency of
data exchange: Whereas a Poisson spike generator continuously omits spikes, also between reconfigurations, a spike injector
only omits spikes when it is asked to.

However, we can convert the discrete signal of a spike injector to a continuous stream of spikes easily in the network
through recurrences. In the example, we modify the neural network used for the example to a very simple recurrent network. We will use
two populations of ``IF_curr_exp`` neurons that are connected to themselves. Their task is to repeat the input spike with a leak term.
That is, we connect these neurons to themselves with a weight that is sufficient to keep them spiking. 

Therefore, the network is as follows:

.. code:: python

    from hbp_nrp_cle.brainsim import simulator as sim
    
    left = sim.Population(5, sim.IF_curr_exp(), label="left")
    right = sim.Population(5, sim.IF_curr_exp(), label="right")
    
    self_connect = sim.StaticSynapse(weight=1.5, delay=sim.RandomDistribution('uniform', (1.0, 50.0))
    all_connector = sim.AllToAllConnector()
    sim.Projection(
        presynaptic_population=right, postsynaptic_population=right,
        connector=all_connector, synapse_type=self_connect, receptor_type="excitatory"
    )
    sim.Projection(
        presynaptic_population=left, postsynaptic_population=left,
        connector=all_connector, synapse_type=self_connect, receptor_type="excitatory"
    )

Mapping a parameter to a spike injector works by simply adding a *MapSpikeSource* decorator for the parameter. Spike injectors
allow to set the number of spikes to be injected as a parameter **n**. They also allow to specify weights and delays of the
created synapse.

The following decorator adds 10 spike injectors with the default weight 2.0 and default delay 1.0ms to the population *left*:

.. code:: python
    
    @nrp.MapSpikeSource("left_injector", nrp.brain.left, nrp.injector, n=10)
    
The idea of this very simplistic first tutorial is to perform the data transmission in the Transfer Function and then
stimulate the *left* and *right* population whose integrated spikes are then transferred back to robot commands.

For that, we use a library function that is integrated in the NRP to analyze an image for its redness. This
function is available in the *hbp_nrp_cle.tf_framework.tf_lib* module.

Finally, we arrive at the following Transfer Function:

.. code:: python

    import sensor_msgs.msg
    import hbp_nrp_cle.tf_framework.tf_lib #import detect_red

    @nrp.MapRobotSubscriber("camera", '/husky/camera')
    @nrp.MapSpikeSource("left_injector", nrp.brain.left, nrp.injector, n=10)
    @nrp.MapSpikeSource("right_injector", nrp.brain.right, nrp.injector, n=10)
    @nrp.MapVariable("last", initial_value=(True, True))
    @nrp.Robot2Neuron(triggers="camera")
    def eye_sensor_transmit(t, camera, left_injector, right_injector, last):
        image_results = hbp_nrp_cle.tf_framework.tf_lib.detect_red(image=camera.value)
        found_left = False
        found_right = False
        if image_results.left * 10 > image_results.go_on:
            found_left = True
            if not last.value[0]:
                clientLogger.info("Found red color left")
            right_injector.inject_spikes()
        if image_results.right * 10 > image_results.go_on:
            found_right = True
            if not last.value[1]:
                clientLogger.info("Found red color right")
            left_injector.inject_spikes()
        if not found_left and not found_right:
            if last.value[0] or last.value[1]:
                clientLogger.info("Found no red color")
            right_injector.inject_spikes()
        last.value = (found_left, found_right)

Monitoring Spikes
-----------------

Of course, we would like to monitor the spiking activity of the neural network in the running simulation.
Unfortunately, at the time of writing, the monitoring system of the NRP is still used synchronously. That means,
monitoring information that is used in the NRP, e.g. through the spike train widget, is only sent out at the
rate of the CLE timestep. If you set this timestep to minutes or even hours, then monitoring information is only sent
out at this interval. This is highly problematic as the monitoring tools currently also only use the time resolution of the CLE
timestep. This means, the monitoring tools are currently mostly useless for Spinnaker simulations.

While the monitoring tools are not adapted to Spinnaker, you can use the client logger to monitor spike activity.
In particular, you could just create a Transfer Function that uses a spike recorder as a regular device.
As any other device in Spinnaker, spike recorders are also enabled as trigger devices for Transfer Functions.

However, unlike the leaky integrators where you can specify how often you would like to receive triggers, spike recorders
trigger Transfer Functions whenever they receive a new UDP packet from the Spinnaker board containing new information on
spiking activity. 

To monitor the activity of the right population, you could simply add the following Transfer Function:

.. code:: python

    @nrp.MapSpikeSink("rec", nrp.brain.right, nrp.spike_recorder)
    @nrp.Neuron2Robot(triggers="rec")
    def live_monitor(t, rec):
        clientLogger.info("Spikes recorded: {}".format(rec.times))
    
.. warning:: If used as trigger devices, Spike recorders erase their recorded spiking activity after
             Transfer Functions have been triggered. Otherwise, the recording is cleared after the
             synchronized Transfer Functions have been executed. Spikes that arrive between the time a Transfer 
             Function is executed and the time the spike recorder is reset, are lost.

In the same way, you can monitor any other neurons as well.
