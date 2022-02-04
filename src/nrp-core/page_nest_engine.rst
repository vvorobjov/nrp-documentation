.. index:: pair: page; NEST Engine
.. _doxid-nest_engine:

NEST Engine
===========

This engine enables the interfacing of the `NEST <https://www.nest-simulator.org/>`__ spiking neural network simulator to NRP-core. Two implementations are provided:

* :ref:`NEST JSON Engine <doxid-nest_json>` : based on the :ref:`JSON over REST <doxid-engine_comm_1engine_json>` engine

* :ref:`NEST Server Engine <doxid-nest_server>` : this is a client-only engine implementation that enables the use of `nest-server <https://pypi.org/project/nest-server/>`__ with NRP-core.

Both implementations behave and are configured in a very similar way. At initialisation time, they take as input a python script where a NEST network is defined; then, at run time, they handle the communication with the Simulation Loop. The details of their behaviour and configuration are described in the pages linked above.

.. toctree::
	:hidden:

	page_nest_json.rst
	page_nest_server.rst

.. rubric:: Related Pages:

|	:doc:`page_nest_json`
|	:doc:`page_nest_server`


