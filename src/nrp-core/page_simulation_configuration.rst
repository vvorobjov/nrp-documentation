.. index:: pair: page; Simulation Configuration
.. _doxid-simulation_configuration:

Simulation Configuration
========================

This page describes how experiments are configured in NRP-core.

The details of the configuration of any given experiment are stored in a single JSON file, hereafter referred to as the “simulation configuration file”. This file (very originally named "simulation_configuration.json" in most of the examples) contains all the necessary configuration parameters to initialize and run an experiment, including details such as the engines involved in the experiment, active TransceiverFunctions, and engine timesteps. Below is an example taken from *examples/tf_exchange* experiment:

.. ref-code-block:: cpp

	{
	    "SimulationName": "tf_exchange",
	    "SimulationDescription": "Launch two python engines. Engine1 will offer the current time as a datapack. tf_1 will request said datapack and send it to Engine2. Engine2 will receive the time and display it",
	    "SimulationTimeout": 1,
	    "EngineConfigs": 
	    [
	        {
	            "EngineType": "python_json",
	            "EngineName": "python_1",
	            "PythonFileName": "engine_1.py"
	        },
	        {
	            "EngineType": "python_json",
	            "EngineName": "python_2",
	            "PythonFileName": "engine_2.py"
	        }
	    ],
	    "DataPackProcessingFunctions":
	    [
	        {
	            "Name": "tf_1",
	            "FileName": "tf_1.py"
	        }
	    ]
	}

In order to configure entirely all aspects of a given simulation, the NRP users only need to modify the configuration file. Inside it, the sections that should receive most attention from the users are: ``EngineConfigs``, where the configuration of all the :ref:`engines <doxid-engines>` participating in the experiment is specified; and ``DataPackProcessingFunctions``, where :ref:`transceiver functions <doxid-transceiver_function>` are specified.

For the definition of the configuration structure, we rely on `json-schema <https://json-schema.org/>`__.

The sections linked hereafter first detail how JSON-schema is used for managing experiment configuration, then go into the details of the main schemas and parameters that users may have to tinker with. These are (mainly) simulation, engine, and transceiver function schemas. NRP users who do not intend to integrate a new simulation engine may focus their attention on the ``Parameters`` section available in the pages linked below. The rest of the contents made available hereafter is intended either for developers wishing to integrate new engines & modify existing ones, or for users interested in implementation details.

* :ref:`General notes about the use of JSON schema <doxid-json_schema>`

* :ref:`Simulation schema <doxid-simulation_schema>`

* :ref:`Engine schema <doxid-engine_base_schema>`

* :ref:`Transceiver function schema <doxid-transceiver_function_schema>`

.. toctree::
	:hidden:

	page_engine_base_schema.rst
	page_json_schema.rst
	page_simulation_schema.rst
	page_transceiver_function_schema.rst

.. rubric:: Related Pages:

|	:doc:`page_engine_base_schema`
|	:doc:`page_json_schema`
|	:doc:`page_simulation_schema`
|	:doc:`page_transceiver_function_schema`


