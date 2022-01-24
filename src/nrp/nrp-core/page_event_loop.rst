.. index:: pair: page; Event Loop
.. _doxid-event_loop:

Event Loop
==========

The Event Loop implements asynchronous interaction between simulations in NRP-core. It processes incoming events from external processes and sends out others in response. The events accepted or sent by the Event Loop are always data messages in any of the supported communication protocols (eg. ROS messages). In contrast with the :ref:`FTILoop <doxid-class_f_t_i_loop>`, the Event Loop allows to connect simulations or other processes asynchronously through this event processing mechanism.

In order to process and send out events, the Event Loop uses a graph structure, the Computational Graph, which nodes are computational units which receive, process and send events. The edges in the graph represent connections between these nodes through which events can be exchanged. Edges are directed, meaning that a given connection allows to send events in only one direction.

The sole function of the Event Loop is to run a Computational Graph at a fixed frequency. The nodes in the graph are executed in a specific order which ensures that each of them performs their computations on the latest available data. By instantiating and connecting nodes, users can specify which events the Event Loop should react to, how they should be processed and which events should be sent out in response.

The next pages describe in more detail the elements and function of the Event Loop and the Computational Graph and how they can be used in NRP-core experiments:

* :ref:`Computational Graph <doxid-computational_graph>` : this page offers a description, with implementation details, of all elements and components involved in the Computational Graph.

* :ref:`Instantiating a Computational Graph in Python <doxid-python_graph>` : this page describes how to define Computational Graphs in Python.

* :ref:`Event Loop configuration in experiments <doxid-event_loop_configuration>` : this page describes how to configure NRP-core experiments to use the Event Loop and the Computational Graph.

Several example experiments using the Event Loop and the Computational Graph are available in the ``examples/event_loop_examples`` folder.

.. toctree::
	:hidden:

	page_computational_graph.rst
	page_event_loop_configuration.rst
	page_python_graph.rst

.. rubric:: Related Pages:

|	:doc:`page_computational_graph`
|	:doc:`page_event_loop_configuration`
|	:doc:`page_python_graph`


