.. index:: pair: page; Event Loop configuration in experiments
.. _doxid-event_loop_configuration:

Event Loop configuration in experiments
=======================================

The :ref:`EventLoop <doxid-class_event_loop>` can be used in NRP-core experiments in replacement of the :ref:`FTILoop <doxid-class_f_t_i_loop>` as the mechanism for processing and relaying the data between simulators or other types of processes. The sections below explain how to configure an NRP-core experiment to use the :ref:`EventLoop <doxid-class_event_loop>` and how :ref:`Engines <doxid-engines>` can be used to interact with the Event Loop and the :ref:`Computational Graph <doxid-computational_graph>`.



.. _doxid-event_loop_configuration_1event_loop_configuration_parameters:

Experiment Configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Below are commented the relevant configuration parameters in the experiment configuration file (see :ref:`here <doxid-simulation_configuration>` for more details on configuring experiments in NRP-core):

* SimulationLoop: this parameter can be set to two values: "FTILoop", "EventLoop". By default "FTILoop" is used. If set to "EventLoop", at startup time, an :ref:`EventLoop <doxid-class_event_loop>` is created and run at a fixed frequency.

* EventLoopTimestep: this parameter sets the frequency at which the Event Loop is run. By default is set to 10 ms.

* EventLoopTimeout: time in seconds the Event Loop will run before shutting down automatically. If set to 0 no timeout is used.

* :ref:`ComputationalGraph <doxid-class_computational_graph>` : this is an array of strings containing the filenames of the Python scripts defining the Computational Graph that will be loaded and executed by the :ref:`EventLoop <doxid-class_event_loop>`.

* StartROSNode: if this boolean parameter is set to ``true``, a ROS node will be started along with the experiment. This is needed when using :ref:`ROS Nodes <doxid-computational_graph_1ros_nodes>` in the Computational Graph.





.. _doxid-event_loop_configuration_1event_loop_engine_interaction:

Interacting with Engines from the Event Loop
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If ``SimulationLoop`` parameter is set to "EventLoop", an :ref:`FTILoop <doxid-class_f_t_i_loop>` is still started and run asynchronously with the :ref:`EventLoop <doxid-class_event_loop>`. This enables the possibility of interacting with :ref:`Engines <doxid-engines>` asynchronously from a Computational Graph using :ref:`Engine Nodes <doxid-computational_graph_1engine_nodes>`. In this case, the Engines specified in the experiment configuration file are started normally but datapacks requested from and sent to them are not given by TransceiverFunctions, but from the Engine Nodes present in the graph.

To implement the behavior described above, the :ref:`FTILoop <doxid-class_f_t_i_loop>` normal functioning is modified when it is run alongside the :ref:`EventLoop <doxid-class_event_loop>`. Referring to the :ref:`FTILoop <doxid-class_f_t_i_loop>` simulation loop steps as described in this :ref:`page <doxid-simulation_loop>`, these modifications are summarized below:

* step (3) is replaced by: for each :ref:`InputEngineNode <doxid-class_input_engine_node>` in the computational graph, methods ``requestedDataPacks()`` and ``setDataPacks()`` are used to get the set of requested datapacks for each engine and, after they are received from the engine, injecting them into the graph.

* steps (4) and (5) are skipped.

* step (6) is replaced by: datapacks published to each :ref:`OutputEngineNode <doxid-class_output_engine_node>` in the graph are fetched using their ``getDataPacks()`` method and sent to the corresponding Engines.





.. _doxid-event_loop_configuration_1fti_loop_with_computational_graph:

Running a Computational Graph Synchronously in an FTILoop
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There is also the possibility of running a Computational Graph synchronously in an :ref:`FTILoop <doxid-class_f_t_i_loop>` replacing Transceiver Functions. In this case, no :ref:`EventLoop <doxid-class_event_loop>` is instantiated and the Computational Graph is executed directly from the :ref:`FTILoop <doxid-class_f_t_i_loop>`. The :ref:`FTILoop <doxid-class_f_t_i_loop>` runs with the same behavior described in the section above, but in every simulation loop, instead of steps (4) and (5), the Computational Graph is executed.

To use a Computational Graph instead of Transceiver Functions in the :ref:`FTILoop <doxid-class_f_t_i_loop>` set parameter ``SimulationLoop`` to "FTILoop" and ``:ref:`DataPackProcessor <doxid-class_data_pack_processor>``` to "cg" in the experiment configuration file.

