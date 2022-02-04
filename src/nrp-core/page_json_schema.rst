.. index:: pair: page; General notes about the use of JSON schema
.. _doxid-json_schema:

General notes about the use of JSON schema
==========================================



.. _doxid-json_schema_1json_schema_in_nrp:

Use of JSON Schema in NRP-core
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

JSON Schema is a highly readable, JSON-based schema language which offers similar capabilities for JSON as the XML Schema does for XML. In NRP-core it is used to define the structure of the experiment configuration as well as constraints and default values of the different parameters. At run-time, it is used to validate the simulation configuration file and to set default values for those parameters which are not present in this configuration file.

JSON Schema allows for composability and inheritance. Taking advantage of this capability, the simulation configuration schema is distributed in different sub-schemas, which are then composed or inherited as needed. **Composition** can be achieved using the `ref <https://json-schema.org/understanding-json-schema/structuring.html#reuse>`__ keyword, which allows to reference other schemas. **Inheritance** is achieved by using `allOf <https://json-schema.org/understanding-json-schema/reference/combining.html#allof>`__, which allows to validate against all the schemas referenced or defined inside of the allOf section. In this way an inherited schema can include a reference to its base schema and its own definition overriding, extending or over-constraining the defined parameters. An example of inheritance can be found in each of the engine schemas.

All JSON schemas in the framework are contained in the folder ``config_schemas`` at the root of the project. Given the readability of json-schema, these schemas can be used directly as a source of documentation to look up each of the parameters in the configuration.

As an example, the schema corresponding to :ref:`simulation <doxid-simulation_schema>`, the top level schema which is used to validate a simulation configuration file is given below.

.. ref-code-block:: cpp

	{
	  "$schema": "http://json-schema.org/draft-07/schema#",
	  "title": "Simulation",
	  "description": "Simulation configuration schema. Specify an experiment using multiple engines and transceiver functions.",
	  "$id": "#Simulation",
	  "type": "object",
	  "properties" : {
	    "SimulationLoop" : {
	      "enum" : ["FTILoop", "EventLoop"],
	      "default": "FTILoop",
	      "description": "Type of simulation loop used in the experiment"
	    },
	    "SimulationTimeout" : {
	      "type" : "integer",
	      "default": 0,
	      "description": "Experiment Timeout (in seconds). It refers to simulation time."
	    },
	    "SimulationTimestep" : {
	      "type" : "number",
	      "default": 0.01,
	      "description": "Time in seconds the simulation advances in each Simulation Loop. It refers to simulation time."
	    },
	    "DataPackProcessor" : {
	      "type" : "string",
	      "enum" :  ["tf", "cg"],
	      "default": "tf",
	      "description": "Framework used to process and rely datapack data between engines. Available options are the TF framework (tf) and Computation Graph (cg)"
	    },
	    "ProcessLauncherType" : {
	      "type" : "string",
	      "default": "Basic",
	      "description": "ProcessLauncher type to be used for launching engine processes"
	    },
	    "EngineConfigs" : {
	      "type" : "array",
	      "items": {"$ref": "https://neurorobotics.net/engines/engine_base.json#EngineBase"},
	      "description": "Engines that will be started in the experiment"
	    },
	    "DataPackProcessingFunctions" : {
	      "type" : "array",
	      "items": {"$ref": "https://neurorobotics.net/transceiver_function.json#TransceiverFunction"},
	      "description": "Transceiver and Preprocessing functions that will be used in the experiment"
	    },
	    "ComputationalGraph" : {
	      "type" : "array",
	      "items": "string",
	      "description": "List of filenames defining the ComputationalGraph that will be used in the experiment"
	    },
	    "StartROSNode" : {
	      "type": "boolean",
	      "default": false,
	      "description": "If true a ROS node is started by NRPCoreSim"
	    },
	    "EventLoopTimeout" : {
	      "type" : "integer",
	      "description": "Event loop timeout (in seconds). 0 means no timeout. If not specified 'SimulationTimeout' is used instead"
	    },
	    "EventLoopTimestep" : {
	      "type" : "number",
	      "description": "Time length (in seconds) of each loop, i.e it is the inverse of the Event Loop frequency. If not specified 'SimulationTimestep' is used instead"
	    }
	  },
	  "required": []
	}

Each JSON schema can have an `id <https://json-schema.org/understanding-json-schema/structuring.html#the-id-property>`__ which enables one to reference them in other schemas. The ``properties`` dictionary contains the definition of all the parameters in the schema. Each of these parameters has a ``type``, a ``description`` (used only for documentation purposes) and optionally a default value.





.. _doxid-json_schema_1schema_reference:

Referencing schemas
~~~~~~~~~~~~~~~~~~~

The ``type`` of a parameter can be a simple type or a reference to another schema. A reference to another schema is formatted as a standard `URI <https://en.wikipedia.org/wiki/Uniform_Resource_Identifier/>`__, with the next parts:

* authority: identifying the institution responsible for the schema. For the schemas developed by the `NRP Team <https://neurorobotics.net/who-we-are.html>`__ this will always be `https://neurorobotics.net <https://neurorobotics.net>`__

* path: this is the path to the file containing the referenced schema from ``config_schemas`` folder

* fragment: the id of the referenced schema

For example ``"https://neurorobotics.net/engines/engine_base.json#EngineBase"`` references a schema with id ``EngineBase`` which is stored in ``config_schemas/engines/engine_base.json`` file and was implemented by the NRP Team. It must be noticed that even though the reference looks like a URL it is actually a URI, meaning that it might not (and usually won't) point to a resource that can be found online.

There is one limitation when referencing a schema by its ``id``. json-schema allows to define multiple schemas in a single file. In this case, the top-level JSON object can be a dictionary storing all the defined schemas. For an example see ``config_schemas/engines/engines_nest.json``. In this case in which the schema is not define at the top-level of the JSON object referencing it by ``id`` doesn't work. The alternative is to reference it by the path to the schema in the JSON object. For example, if the schema below is stored in a file named ``example.json`` :

.. ref-code-block:: cpp

	{"engine_1" : {
	    "$id": "#Engine1",
	  },
	  "engine_2" : {
	    "$id": "#Engine2",
	  }
	}

``Engine1`` could be referenced as:

.. ref-code-block:: cpp

	https://example.net/example.json#/engine_1

For a more in-depth explanation of the json-schema format details, `https://json-schema.org <https://json-schema.org>`__ is full of `guides <https://json-schema.org/learn/getting-started-step-by-step.html>`__ and information.





.. _doxid-json_schema_1default_parameters:

Parameter Default Values
~~~~~~~~~~~~~~~~~~~~~~~~

Default values of schema parameters are used to automatically set a value for the corresponding parameter if it has not been explicitly set in the configuration file. In this way, if a parameter is not present in the configuration file but it has a default value in the schema, it will be automatically added. This step is performed after validating the configuration file.

Those parameters which can't be left unset in the configuration file must be marked as *required* in the schema. For that they must be included in the ``required`` list in the schema JSON object, as it is done in the simulation schema listed above. Usually, required parameters don't have a default value and vice versa, i.e. if a parameter is not required, a default value is defined for it in the schema. There are two exceptions to this rule:

* Due to limitations in the `library <https://github.com/pboettch/json-schema-validator>`__ we use for schema validation, default values set to an empty array (``[]``) are disregarded.

* Sometimes the default value of a parameter cannot be set to a fixed value in the schema. This is for example the case when default values want to be set to an environment variable. Another example could be if the default value for a parameter needs some computations.

To undertake these exceptions, default values can also be set in-code using the convenience method ``json_utils::set_default``. If a default value for a certain parameter is to be set in-code using this method, it should not be set as required in the corresponding schema and a default value should not be defined for it in this schema.

Finally, when using ``allOf`` to define schema hierarchies (e.g. in engine schemas), all the schemas specified within the ``allOf`` section will be traversed in order. An important implication of this behavior is that the first default value found for a parameter will be used. Therefore, in case of wanting to override default values in an inherited schema, the reference to the base schema should be place at the end of the ``allOf`` list.

