Tutorial: Executing Transfer Functions outside the simulation loop
==================================================================

Transfer Functions by default run synchronized to the neural network and the world simulation due to
the advantages in reproducibility. However, some applications have special demands where Transfer
Functions should be able to be executed independent of the simulation loop, for example when a new
sensor value arrives or when a special event happens.

For these applications, the NRP allows to define custom triggers for Transfer Functions. That is,
you can define that certain devices of the TF should be used to indicate when the TF should be run.

Calling Transfer Functions when a new image arrives
---------------------------------------------------

As an example, consider the *eye_sensor_transmit* TF from the :doc:`robot2neuron`. You might want to specify
that you would like to run this TF not only with the timestep of the CLE (by default every 20ms),
but simply whenever there is a new camera image coming from the robot. To do that, all you need to do is simply
to define the *camera* device as a trigger for this TF.


.. code-block:: python

    @nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))
    @nrp.MapSpikeSource("red_left_eye", nrp.brain.sensors[slice(0, 3, 2)], nrp.poisson)
    @nrp.MapSpikeSource("red_right_eye", nrp.brain.sensors[slice(1, 4, 2)], nrp.poisson)
    @nrp.MapSpikeSource("green_blue_eye", nrp.brain.sensors[4], nrp.poisson)
    @nrp.Robot2Neuron(triggers="camera")
    def eye_sensor_transmit(t, camera, red_left_eye, red_right_eye, green_blue_eye):

This specification is done by simply assigning the parameter name to the triggers of the TF. If you specify
a string that is not the name of a parameter, an exception will be raised.

Multiple triggers
-----------------

Transfer Functions also allow to define multiple triggers. In that case, the TF will be run whenever
*any* of the trigger devices signals that the TF could run. If we extend the *eye_sensor_transmit* TF
with a second camera, the header of that TF would as follows:

.. code-block:: python

    @nrp.MapRobotSubscriber("camera1", Topic('/husky/camera1', sensor_msgs.msg.Image))
    @nrp.MapRobotSubscriber("camera2", Topic('/husky/camera2', sensor_msgs.msg.Image))
    @nrp.MapSpikeSource("red_left_eye", nrp.brain.sensors[slice(0, 3, 2)], nrp.poisson)
    @nrp.MapSpikeSource("red_right_eye", nrp.brain.sensors[slice(1, 4, 2)], nrp.poisson)
    @nrp.MapSpikeSource("green_blue_eye", nrp.brain.sensors[4], nrp.poisson)
    @nrp.Robot2Neuron(triggers=["camera1", "camera2"])
    def eye_sensor_transmit(t, camera1, camera2, red_left_eye, red_right_eye, green_blue_eye):

In other words, you can simply use lists to provide multiple triggers.

.. note::
    If you specify a trigger, this means that the TF does not take part in the normal TF execution
    logic. Thus, if you assign an empty list to the triggers of a TF, that TF will never be run.

Triggers and synchronized execution
-----------------------------------

In case you need both out-of-order execution through triggers *and* normal synchronized execution,
you can achieve such a scenario by simply adding *t* to the trigger list.
