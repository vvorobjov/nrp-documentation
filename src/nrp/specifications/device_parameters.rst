Device Parameters
=================

Devices in the CLE can be configured with additional parameters depending on the type of device created. The purpose of this section is to document these parameters and their default values.
With these configurations, a user may override some parameters in the device creation.

For example, suppose you would like to set the default amplitude of a DC source to zero. That is, you may create a device using the following code:

.. code:: Python

    @MapSpikeSource("Parameter", nrp.map_neurons(range(600), lambda i: nrp.brain.sensors[i]), nrp.dc_source)

The following code should do the trick. Note that you just add the parameter "amplitude = 0.0" at the end of the device. 

.. code:: Python

    @MapSpikeSource("Parameter", nrp.map_neurons(range(600), lambda i: nrp.brain.sensors[i]), nrp.dc_source, amplitude = 0.0)

The following lists the possible configuration options for each device type with their default values.

Connectors
----------

A *connector* may be used to specify default values for weights and delays that are then used across 
multiple devices. Further, a connector may have a mode that determines whether the spikes should be 
delivered to all, to one or to a fixed number of connected neurons. If a connector is specified, the 
connector must specify a mode. If no connector is specified, an AllToAll connector is used. 
Connectors are specified as dictionaries with the following entries:

- weight (optional)
- delay (optional)
- mode (OneToOne, AllToAll, Fixed)

A fixed number pre connector must also specify a number (n) that defaults to 1.

AC Generator
------------

- amplitude: 0.0
- offset: 0.0
- frequency: 10.0
- phase: 0.0
- start: 0.0
- stop: infinity

DC Generator
------------

- amplitude: 0.0
- start: 0.0
- stop: infinity

Fixed Spike Generator
---------------------

- initial_rate: 0.0
- cm: 1.0
- tau_m: 1000.0
- tau_refrac: sim.state.dt
- v_thresh: -50.0
- v_reset: -100.0
- v_rest: -100.0
- connector: None
- weight: None
- delay: None
- source: None
- receptor_type: excitatory
- synapse_type: None
- label: None

If no weight is specified, neither in the device nor in a connector, random weights are chosen from
a uniform distribution from [0.0, 0.01] ([-0.01, 0.0] if the connection is inhibitory and the connected neurons are
conductance-based).

By default, a static synapse is created with the specified weight and delay. Alternatively, a Tsodyks-Markram synapse can be created.

Leaky Integrator (Alpha-shaped)
-------------------------------

- v_thresh: infinity
- cm: 1.0
- tau_m: 10.0
- tau_syn_E: 2.
- tau_syn_I: 2.
- v_rest: 0.0
- v_reset: 0.0
- tau_refrac: 0.1
- i_offset: 0.0
- connector: None
- weight: None
- delay: 0.1
- source: None
- receptor_type: excitatory
- synapse_type: None
- label: None
- rng: None

A leaky integrator device is always initialized with the resting potential v_rest.

If no weight is specified, neither in the device nor in a connector, weights are set to 0.01 (-0.01 if the connection is inhibitory).

By default, a static synapse is created with the specified weight and delay. Alternatively, a Tsodyks-Markram synapse can be created.

Leaky Integrator (Exp-Shaped)
-----------------------------

- v_thresh: infinity
- cm: 1.0
- tau_m: 20.0
- tau_syn_E: .5
- tau_syn_I: .5
- v_rest: 0.0
- v_reset: 0.0
- tau_refrac: 0.1
- i_offset: 0.0
- connector: None
- weight: None
- delay: RandomDistribution(uniform, [0.1, 2.0])
- source: None
- receptor_type: excitatory
- synapse_type: None
- label: None
- rng: None

A leaky integrator device is always initialized with the resting potential v_rest.

If no weight is specified, neither in the device nor in a connector, random weights are chosen from
a uniform distribution from [0.0, 0.01] ([-0.01, -0.0] if the connection is inhibitory).

By default, a static synapse is created with the specified weight and delay. Alternatively, a Tsodyks-Markram synapse can be created.

NC Generator
------------

- mean: 0.0
- stdev: 1.0
- dt: sim.state.dt
- start: 0.0
- stop: infinity

Poisson Generator
-----------------

- duration: infinity
- start: 0.0
- rate: 0.0
- connector: None
- weight: 0.00015
- delay: 0.1
- source: None
- receptor_type: excitatory
- synapse_type: None
- label: None
- rng: None
- n: 1

If no weight is specified, neither in the device nor in a connector, random weights are chosen from
a uniform distribution from [0.0, 0.01] ([-0.01, -0.0] if the connection is inhibitory).

By default, a static synapse is created with the specified weight and delay. Alternatively, a Tsodyks-Markram synapse can be created.

The parameter *n* specifies how many Poisson generators should be created in the network that realize
the Poisson generator device.

Population Rate
---------------

- tau_fall: 20.0
- tau_rise: 10.0

Spike Recorder
--------------

- use_ids: True

The spike recorder returns an array with all the spikes of the respective population.
By default, the neurons are identified using their global IDs of the underlying simulator.
However, by setting **use_ids** to *False*, this behavior changes and the recorder returns the 
indices of the neurons within the monitored population. If that population is a view, the index of
the neuron within the view is used for the spike recorder.
