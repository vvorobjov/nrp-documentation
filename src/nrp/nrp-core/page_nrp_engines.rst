.. index:: pair: page; Engine implementations shipped with NRP-core
.. _doxid-nrp_engines:

Engine implementations shipped with NRP-core
============================================

This page lists :ref:`Engine <doxid-engines>` implementations currently available with NRP-core. These implementations provide out-of-the-box functionality for use with NRP-core, and also represent good examples to study for users interested in implementing their own engines. For each engine implementation, a general description of the provided functionality is offered first, followed by the engine configuration parameters, and finally a list of datapack types supported by the engine.

Additionally, in order to make it easier to implement new engines, we propose a way to leverage code reusability: for new engine implementations, interested users can therefore refer to one of the available :ref:`Engine Communication Protocols <doxid-engine_comm>`. These are Engine templates implementing all aspects of engine client / server communication for a given protocol.

List of available engines:

* :ref:`Gazebo <doxid-gazebo_engine>` : engine implementation for the `Gazebo physics simulator <http://gazebosim.org/>`__

* :ref:`NEST <doxid-nest_engine>` : two different implementations that integrate the `NEST Simulator <https://www.nest-simulator.org/>`__ into NRP-core

* :ref:`Python JSON Engine <doxid-python_json_engine>` : generic Python-based mechanism that imports and executes a simulator inside a user-defined python script. Ideal for simulators with a Python API.

* :ref:`OpenSim <doxid-opensim_engine>` : engine implementation based on the :ref:`Python JSON Engine <doxid-python_json_engine>` and the `OpenSim Python API <https://simtk.org/api_docs/opensim/api_docs#>`__

.. toctree::
	:hidden:

	page_engine_comm.rst
	page_gazebo_engine.rst
	page_nest_engine.rst
	page_opensim_engine.rst
	page_python_json_engine.rst

.. rubric:: Related Pages:

|	:doc:`page_engine_comm`
|	:doc:`page_gazebo_engine`
|		:doc:`page_gazebo_datapacks`
|		:doc:`page_gazebo_plugins`
|	:doc:`page_nest_engine`
|		:doc:`page_nest_json`
|		:doc:`page_nest_server`
|	:doc:`page_opensim_engine`
|	:doc:`page_python_json_engine`


