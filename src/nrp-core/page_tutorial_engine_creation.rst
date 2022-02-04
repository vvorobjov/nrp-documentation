.. index:: pair: page; Creating a new Engine from scratch
.. _doxid-tutorial_engine_creation:

Creating a new Engine from scratch
==================================

As explained in :ref:`apposite section of the architecture overview <doxid-engines>`, engines are a core aspect of the NRP-core framework. They run the various (and possibly heterogeneous) components/modules of the simulations, with the Simulation Loop and TransceiverFunctions merely being a way to synchronize and exchange data therebetween. In other terms, the NRP-core facilitates communication between differing simulator types in order to integrate them into a single coherent simulation. We aim to achieve predictable behaviour even in cases where simulators with different execution schemes are deployed. This requires a strict engine interface, which will properly synchronize runtime and data exchange.

The NRP has adopted a client-server approach to this problem, together with constraints in terms of synchronous communications. Each simulator runs in its own process, and acts as a server. The Simulation Loop manages synchronization, and accesses each engine as a client. Data exchange is facilitated via :ref:`DataPacks <doxid-datapacks>`. Therefore, a developer wishing to create a new engine **must supply five components** :

* :ref:`Engine Server <doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_server>`

* :ref:`EngineClient <doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_client>`

* :ref:`ProcessLauncher <doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_proc_launcher>`

* :ref:`Engine configuration schema <doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_config>`

* Only if required, custom :ref:`DataPack python bindings <doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_datapack>`

In the next sections we comment how to proceed to implement each of this components. The code samples in these sections are based on a bare bone example engine included in the folder *docs/example_engine*.

Please note that this guide describes the steps needed to create an engine *from scratch*. It will take a considerable amount of development time, and the exact implementation will depend on the communication protocol and data structures of your choice. In order to learn how to base your new engine implementation in one of the provided engine templates see this guide: :ref:`Creating a new Engine from template <doxid-engine_creation_template>` Should you wish to integrate a simulator with a Python interface in NRP-core, we also supply a :ref:`PythonJSONEngine <doxid-python_json_engine>`, which can execute arbitrary Python scripts.



.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_directories:

Directory tree
~~~~~~~~~~~~~~

We propose to structure source files of the new engine in the following way:

.. ref-code-block:: cpp

	example_engine/
	├── cmake
	│   └── ProjectConfig.cmake.in
	├── CMakeLists.txt
	├── example_engine_server_executable
	│   ├── example_engine_server_executable.cpp
	│   ├── example_engine_server_executable.h
	│   └── main.cpp
	└── nrp_example_engine
	    ├── config
	    │   ├── cmake_constants.h.in
	    │   ├── example_config.h
	    │   └── example_config.json
	    ├── engine_server
	    │   ├── example_engine_server.cpp
	    │   └── example_engine_server.h
	    ├── nrp_client
	    │   ├── example_engine_client.cpp
	    │   └── example_engine_client.h
	    └── python
	        ├── example_engine_python.cpp
	        └── __init__.py.in

* root - root directory of the new engine, the right place to put your ``CMakeLists.txt``

* cmake - helper files for cmake

* example_engine_server_executable - source files related to server executable

* config - source files related to engine configuration

* engine_server - source code of the server side of the engine

* nrp_client - source code of the client side of the engine

* python - Python module with Python wrappers for datapack classes





.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_cmake:

Setting up CMake
~~~~~~~~~~~~~~~~

We use CMake to manage project compilation. In this :ref:`page <doxid-tutorial_engine_creation_engine_cmake_example_explanation>`, it is explained the basic structure used in an Engine cmake configuration file in order to create all libraries and executables necessary for the new engine.





.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_config:

Creating an Engine configuration schema
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Engines should be configurable by users. Configuration is based on JSON documents, which are validated using `JSON schemas <https://json-schema.org/>`__. This :ref:`page <doxid-simulation_configuration>` offers more details on configuration management in NRP-core.

Every new engine configuration schema should be based on the provided basic configuration schema:

.. code-block:: cpp

	https://neurorobotics.net/engines/engine_base.json#/EngineBase

The new engine schema can be afterwards placed into a separate JSON file in *config_schemas/engines/* folder, so it can be found at run time.

Here is an example of how this might look like:



.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_config_example:

Example
-------

.. ref-code-block:: cpp

	{
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Example config",
	    "description": "Base Json Engine configuration schema",
	    "$id": "#EngineExample",
	    "allOf": [
	      { "$ref": "https://neurorobotics.net/engines/engine_base.json#EngineBase" },
	      {
	        "properties" : {
	          "ServerAddress": {
	            "type": "string",
	            "default": "localhost:9002",
	            "description": "Address from which the engine server sends/receives data"
	          },
	          "RegistrationServerAddress": {
	            "type": "string",
	            "default": "localhost:9001",
	            "description": "Address to which servers should register to"
	          }
	        }
	      }
	    ]
	}





.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_config_linking:

Linking configuration schema to the engine
------------------------------------------

To use the newly created schema, it has to be linked to the engine client. This is done by passing the schema URI as template argument to the base class of your new engine:

.. ref-code-block:: cpp

	class ExampleEngineClient
	        : public EngineClient<ExampleEngineClient, "https://neurorobotics.net/engines/engine_example.json#/EngineExample">

A further explanation of how schema URIs are structured can be found in the section :ref:`Referencing schemas <doxid-json_schema_1schema_reference>`.







.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_datapack:

DataPack data
~~~~~~~~~~~~~

DataPacks are object that facilitate exchange of data between transceiver functions and simulators. A more detailed description of datapacks can be found in this :ref:`page <doxid-datapacks>`.

They consists of some :ref:`data structure <doxid-datapacks_1datapacks_data>`, used as payload in data exchanges between Engine servers and clients, and :ref:`metadata <doxid-datapacks_1datapacks_id>`, used to uniquely identify the datapack and relate it to a specific engine. There is in principle no restrictions in the type a :ref:`DataPack <doxid-class_data_pack>` can store, as long as Engine client and server are able to exchange them over the wire. But since DataPacks also must be available inside of TransceiverFunctions, python bindings must be available for each data type used in DataPacks. To reduce the complexity of developping new engines, it is strongly recommended to use one of the data types for which NRP-core already provide python bindings. These are: nlohmann::json and protobuf messages. However, if you decide to use another data type you must implement Python bindings for it and make them available to NRP-core as a Python module.





.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_client:

Creating an EngineClient
~~~~~~~~~~~~~~~~~~~~~~~~

An :ref:`EngineClient <doxid-class_engine_client>` is used by the Simulation Loop to interface with a simulator via an Engine Server. A communication protocol is required to facilitate data exchange. We provide a set of predefined protocol implementations here. In most cases, using one of these as a base template suffices and greatly reduces development efforts.

A new engine client must inherit from the :ref:`EngineClient <doxid-class_engine_client>` class. As such, it may look as shown below. A detailed function description can be found in :ref:`EngineClientInterface <doxid-class_engine_client_interface>`.

A set of methods need to be implemented in the new client class. These methods will be called by the Simulation Loop in various points of the loop.



.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_simulation_control_methods:

Simulation control and state methods
------------------------------------

* :ref:`EngineClientInterface::initialize() <doxid-class_engine_client_interface_1ac600fd036f83cc1aa0ae8fa79b176b44>` - should perform all necessary steps (requests to the server) to initialize the simulation. The function is called before the simulation loop starts.

* :ref:`EngineClient::runLoopStepCallback() <doxid-class_engine_client_1ac0325b83cbae4d2eb7c2acea2206afd7>` - callback method called from :ref:`EngineClient::runLoopStepAsync() <doxid-class_engine_client_1a4b95a41aa73bbc8367d7acf0f47c2756>` which should request the server to run a simulation step with specified timestep. The request is performed in a separate thread. This allows all engines to run their steps in parallel. :ref:`EngineClientInterface::runLoopStepAsyncGet() <doxid-class_engine_client_interface_1af7df79c71a0b87597b1638b07384c4e9>` must be used to join the threads again.

* :ref:`EngineClientInterface::shutdown() <doxid-class_engine_client_interface_1a0a15d1d539bc8134f8ed62668c284883>` - should request the server to perform cleanup, before the server process is requested to terminate.

* :ref:`EngineClientInterface::reset() <doxid-class_engine_client_interface_1a65d86ba09fd72e32b0399ca290c38632>` - should request the server to reset the simulation.





.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_data_exchange_methods:

Data exchange methods
---------------------

* :ref:`EngineClientInterface::getDataPacksFromEngine() <doxid-class_engine_client_interface_1aa6a8f352acfa20e6cc3d34e3c69a7468>` - may be used to request results of the latest step from the simulator. Received data should be deserialized into proper datapack types, which will be consumed by transceiver functions. The function will be called before runLoopStepAsync.

* :ref:`EngineClientInterface::sendDataPacksToEngine() <doxid-class_engine_client_interface_1a72110362e024889f75d55c3a80696da3>` - may be used to pass relevant data, like reference values, to the simulator. Input to the functions will be a list of datapacks, (results of transceiver function execution). The datapacks need to be serialized into structures used by the communication protocol between engine client and server. The function will be called after runLoopStepAsync.





.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_simulation_spawning_methods:

Simulation process spawning methods
-----------------------------------

These methods are used by the process launcher.

* :ref:`EngineClientInterface::engineProcStartParams() <doxid-class_engine_client_interface_1a6747137f2b551040adca807e6df38a59>` - should return all startup parameters of the simulation process. Related to :ref:`EngineProcStartParams <doxid-engine_base_schema_1engine_proc_start_params>` config parameter.

* :ref:`EngineClientInterface::engineProcEnvParams() <doxid-class_engine_client_interface_1ac3bf04a627785a082fbe83a5aa004227>` - should return any extra environment variables needed to run the simulation process. Related to :ref:`EngineEnvParams <doxid-engine_base_schema_1engine_env_params>` config parameter.





.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_client_example:

Example
-------

.. ref-code-block:: cpp

	#ifndef EXAMPLE_ENGINE_CLIENT_H
	#define EXAMPLE_ENGINE_CLIENT_H
	
	#include "nrp_example_engine/config/example_config.h"
	#include "nrp_general_library/engine_interfaces/engine_client_interface.h"
	#include "nrp_general_library/plugin_system/plugin.h"
	
	class ExampleEngineClient
	        : public :ref:`EngineClient <doxid-class_engine_client>`<ExampleEngineClient, ExampleConfigConst::EngineSchema>
	{
	    public:
	
	        ExampleEngineClient(:ref:`nlohmann::json <doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` &config, :ref:`ProcessLauncherInterface::unique_ptr <doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>` &&launcher);
	
	        void :ref:`initialize <doxid-class_engine_client_interface_1ac600fd036f83cc1aa0ae8fa79b176b44>`() override;
	
	        void :ref:`reset <doxid-class_engine_client_interface_1a65d86ba09fd72e32b0399ca290c38632>`() override;
	
	        void :ref:`shutdown <doxid-class_engine_client_interface_1a0a15d1d539bc8134f8ed62668c284883>`() override;
	
	        void :ref:`sendDataPacksToEngine <doxid-class_engine_client_interface_1a72110362e024889f75d55c3a80696da3>`(const datapacks_ptr_t &datapacksArray) override;
	        datapacks_set_t :ref:`getDataPacksFromEngine <doxid-class_engine_client_interface_1aa6a8f352acfa20e6cc3d34e3c69a7468>`(const datapack_identifiers_set_t &datapackIdentifiers) override;
	
	        const std::vector<std::string> :ref:`engineProcStartParams <doxid-class_engine_client_interface_1a6747137f2b551040adca807e6df38a59>`() const override;
	
	        const std::vector<std::string> :ref:`engineProcEnvParams <doxid-class_engine_client_interface_1ac3bf04a627785a082fbe83a5aa004227>`() const override;
	
	    protected:
	
	        :ref:`SimulationTime <doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`runLoopStepCallback <doxid-class_engine_client_1ac0325b83cbae4d2eb7c2acea2206afd7>`(:ref:`SimulationTime <doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep) override;
	};
	
	using ExampleEngineLauncher = ExampleEngineClient::EngineLauncher<ExampleConfigConst::EngineType>;
	
	:ref:`CREATE_NRP_ENGINE_LAUNCHER <doxid-plugin_8h_1adcd291c2e449ed4d17ddfb6476b1d246>`(ExampleEngineLauncher);
	
	
	#endif // EXAMPLE_ENGINE_CLIENT_H







.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_python:

Creating a Python module
~~~~~~~~~~~~~~~~~~~~~~~~

The Simulation Loop and engines are written in C++, but transceiver functions are written in Python. We need a way of wrapping C++ code with Python, particularly for datapack data types which will be used in TFs. This is done inside so called Python module. Most of the wrappers are already defined in the base Python module, but wrappers for new datapack types must be added.

.. ref-code-block:: cpp

	namespace python = boost::python;
	
	:ref:`BOOST_PYTHON_MODULE <doxid-nrp__general__library_2nrp__general__library_2python_2python__module_8cpp_1ad9c5acca6f80373c6921132395ceb500>`(PYTHON_MODULE_NAME)
	{
	    // Import the base Python module
	    python::import(PYTHON_MODULE_NAME_STR);
	}





.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_proc_launcher:

Creating a new ProcessLauncher
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The NRP runs multiple simulators. To keep their runtime environment separate, each simulator runs in its own process. At startup, the NRP forks an additional process for each engine. This is the purpose of the :ref:`ProcessLauncher <doxid-class_process_launcher>`. Usually, developers can use the default launcher and won't have to implement their own. However, should the need arise, a developer can define his own :ref:`LaunchCommand <doxid-class_launch_command>`. We recommend using the :ref:`BasicFork <doxid-class_basic_fork>` class as a starting template, and modify it to fit the specific engine's needs.





.. _doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_server:

Creating an Engine Server
~~~~~~~~~~~~~~~~~~~~~~~~~

An Engine Server runs in its own process, executes the simulation, and exchanges data with the Simulation Loop via the :ref:`EngineClient <doxid-class_engine_client>`. To interface with said client, a communication protocol is required. We provide a set of predefined protocol implementations here.

If your simulator provides a dedicated server, you may use it directly, by specifying path to the executable in :ref:`EngineProcCmd <doxid-engine_base_schema_1engine_proc_command>` config parameter. An example of engine using a server provided by the simulator is our nrp_nest_server_engine.

If no dedicated server exists for your simulator, you will need to create it. Generally, the server must be able to handle requests from the following client methods:

* initialize - initialize the simulation with parameters coming from the client

* shutdown - shutdown the simulation

* reset - reset the simulation

* runLoopStepAsync - run step of the simulation with step duration requested by the client

* getDataPacksFromEngine - return data from the last simulation step to the client

* sendDataPacksToEngine - retrieve data for the next simulation step from the client

The Engine Server must also define a :ref:`main() <doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>` function to execute. Path to the executable should be specified in :ref:`EngineProcCmd <doxid-engine_base_schema_1engine_proc_command>` config parameter.

.. toctree::
	:hidden:

	page_tutorial_engine_creation_engine_cmake_example_explanation.rst

.. rubric:: Related Pages:

|	:doc:`page_tutorial_engine_creation_engine_cmake_example_explanation`


