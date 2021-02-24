Tutorial: throttling a Transfer Function
========================================

In some applications, in particular when using multiple triggers as in the :doc:`triggers`,
you need to throttle Transfer Functions so that they are only executed fewer times. For example,
you experience that your camera is very fast sending new images but you actually only need to update much slower.
For that, the NRP offers throttling of TFs. That is, you can specify that a Transfer Function runs at most at a given frequency.

As an example, consider again the *eye_sensor_transmit* TF from the :doc:`triggers`. We basically want that the TF is only run
if at least a certain time has passed since the last execution of that TF. To achieve that, we can specify a throttling to a maximum frequency.
For example, we want the TF to run at the maximum frequency of 25hz. To achieve that, we set the throttling to 25:

.. code-block:: python

    @nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))
    @nrp.MapSpikeSource("red_left_eye", nrp.brain.sensors[slice(0, 3, 2)], nrp.poisson)
    @nrp.MapSpikeSource("red_right_eye", nrp.brain.sensors[slice(1, 4, 2)], nrp.poisson)
    @nrp.MapSpikeSource("green_blue_eye", nrp.brain.sensors[4], nrp.poisson)
    @nrp.Robot2Neuron(triggers="camera", throttling_rate=25.0)
    def eye_sensor_transmit(t, camera, red_left_eye, red_right_eye, green_blue_eye):

The frequency specifications may either be integers or floats. You can define a throttling frequency also for
Transfer Functions that combine triggers with normal synchronized execution or do not use triggers at all.
In that case, the TF will be called synchronized with the CLE, but at most with the given frequency, which may
mean that some cycles are simply skipped for that TF.

For example, the throttling rate of 25hz in the example above specifies that the TF is called at most
every 40ms. Nevertheless, it is only called potentially when one of its triggers tell it to. Consider
for instance that the camera sends a new image every 30ms, then the TF is actually called every 60ms (the
smallest multiple of the trigger rate that is greater or equal to the minimum interval. If no throttling is
specified, the minimum interval is 0 and the TF is always executed.
