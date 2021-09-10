Changing Simulated Models during a running simulation
=====================================================

.. todo:: Add author/responsible


During a simulation, it is possible to change the models simulated in each of the simulators separately.
This may be done in order to continue to use the state of the other simulator or the internal state
of transfer functions. Besides actually changing the models, such a change may imply a reset of the
communication adapters and a reset of all Transfer Functions connected to adapter components
issued by the communication adapter as well.

- **Changing the Robot**
  For the default robot communication adapter (:class:`hbp_nrp_cle.robotsim.RosCommunicationAdapter`),
  such a hard reset is not necessary as ROS topics are not closely coupled. In particular, it is transparent
  to a subscriber when a publisher vanishes and a new publisher is about to be created. As long as a
  new publisher publishes messages on the same topic, the subscriber does not get noticed at all.

- **Changing the Neuronal Network**
  The situation is very different for the neuronal network simulation where the communication adapters
  may inject artificial neurons into the network. When the network is reset, these neurons cease to exist
  and must be reintroduced. This requires functionality to discard the current devices connected to any
  of the transfer functions and replace them by reconnecting the transfer functions to neurons.


Implementation of Changing a Neuronal Network under the hood
------------------------------------------------------------

The CLE has support for changing the neuronal network at runtime (during
a simulation). For this, the following steps are performed:

#. If the CLE is currently running, stop it.
#. If the brain adapter has already loaded a neuronal network, discard it.
#. Load the new neural network through the brain control adapter.
#. If populations have been renamed, rename also the population references in the transfer functions
#. Perform a hard reset on the brain devices of the transfer functions.
   That is: Discard all devices created so far, then iterate through all the transfer functions and
   recreate all brain devices from scratch.
#. If anything bad happens, for example if neurons are accessed that no longer exist, roll back to the previous
   brain. If necessary, also roll back the renaming of populations in the transfer functions

In particular, the following is not done:

- Reset of the robot publishers and subscribers. In particular, if no new topic messages are sent, any
  topic subscriber will still have its old value.
- Reset of the TF variables to their initial values. This means that any stateful transfer functions will
  not be reset.
- Reset of the position of the robot or other models loaded in the robots environment.

By reusing one of these configurations a reset of the entire simulation may not be needed
after changing the neuronal network. This is in particular helpful, when only some parameters internal
to the neuronal network have been adjusted.

Necessary Considerations when changing a Neuronal Network
---------------------------------------------------------

The neuronal network represents the main control of the robot during a simulation and if this control
is exchanged, the simulation may fail. Thus, when changing a neuronal network during a simulation, the
following points ought to be taken into consideration:

- The simulation time is not reset. This means that the neuronal network will now have a different clock than
  the robotic simulation. They are still being synchronized by the CLE in the sense that they operate on
  the same speed but they may have different values.
- The CLE will try to create new devices for all transfer functions
  currently present in the Transfer Function Manager. This will execute the neuron selectors on the
  changed neuronal network. If this selection fails, if for example the network no longer contains
  a population with this name, then the previous neural network is restored.

The last point means that as soon as you want to change the topology of the network and
not just some connection or global variables, then these actions must be done very thoughtfully
otherwise the simulation transitions to its failed state and then it cannot be resumed.

In particular, you must ensure that all neurons that were previously connected to a transfer function
still exist. This could be implemented by deleting all transfer functions which connect to neurons who
will not be there in the new network, as only transfer functions that are in the transfer function manager
when the network is changed will be reconnected.

Furthermore, all new transfer functions need to be valid at the time they are added to the CLE. In
particular, when a transfer function targets neurons that will be introduced by a new network,
the neural network must be changed before the transfer function can be created.

Overall, this implies the following workflow:

#. Delete all transfer functions which connect to neurons that will not exist when the neural network
   is changed
#. Change the neural network
#. Add transfer functions which connect to neurons newly introduced in the neural network

Every other sequence will lead the simulation to a failed state.

.. note:: Aside from technical restrictions, it can be thought that changing the neural
    network conflicts with biological plausibility, usually an important goal when working with spiking
    neural networks. This is the reason the functionality is mainly focused on simple parameter changes,
    e.g. to compensate for missing learning implementations or to adjust
    parameters that cannot be learnt.

