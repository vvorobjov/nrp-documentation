.. index:: pair: page; Creating a new Engine from template
.. _doxid-engine_creation_template:

Creating a new Engine from template
===================================

We provide engine templates, also referred as :ref:`Engine Communication Protocols <doxid-engine_comm>`, which implement most of the functionality needed to run simulations. These engines take care of setting up client and server instances, communication, sending control messages and performing data exchange. Only simulator-related glue code and functionality have to be added.

Currently available template engines:

* :ref:`JSON over REST <doxid-engine_comm_1engine_json>`

* :ref:`protobuf over gRPC <doxid-engine_comm_1engine_grpc>`

A minimal working engine of any of the types mentioned above can be generated using a script that comes with NRP Core. The script is going to create the following components:

* Client class, allowing NRP Core to communicate with the server process.

* Server class, responsible for direct interactions with the simulator.

* Server executable, which allows to start the server as a separate process

* :ref:`DataPack <doxid-class_data_pack>` controller class, responsible for passing data to and from the simulation. The class will be used by the engine server.

* CMakeLists.txt that allows to compile the new code.

* Minimal experiment, that allows to verify if the newly created code works as expected.

In most cases the only entities that will need further modifications are the server class and the datapack controller class, as they are the two classes that are going to directly interact with your simulator.

*Please note that in order to use these engine your simulator has to provide a C/C++ API.*



.. _doxid-engine_creation_template_1engine_creation_template_script:

Creating a new engine from template
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



.. _doxid-engine_creation_template_1engine_creation_template_script_name:

Engine name
-----------

In order to generate a new engine, you will need to choose a name. This name will be used to generate all other names in the engine - classes, directories, modules etc. Ideally it should be a single word, for example the name of your simulator, and start with an uppercase letter.





.. _doxid-engine_creation_template_1engine_creation_template_script_grpc:

Generating and compiling code for gRPC engine
---------------------------------------------

We are going to create a gRPC engine called TestGrpcEngine

To generate the code of the new engine:

.. ref-code-block:: cpp

	cd tools
	./create_new_engine.py --name Test --type grpc

This command should create a new directory called test_grpc_engine. Move it to the top-level directory:

.. ref-code-block:: cpp

	mv test_grpc_engine ../test_grpc_engine

To compile the new engine, add the following line at the end of the top-level CMakeLists.txt and configure the project again:

.. ref-code-block:: cpp

	add_subdirectory("test_grpc_engine")

Compile the code:

.. ref-code-block:: cpp

	cd build
	make -j8 install

In order to verify that the newly created engine works, you can run the example experiment provided with the engine:

.. ref-code-block:: cpp

	cd ../test_grpc_engine/example_experiment
	NRPCoreSim -c simulation_config.json -p "NRPTestGrpcEngine.so"





.. _doxid-engine_creation_template_1engine_creation_template_script_json:

Generating and compiling code for JSON engine
---------------------------------------------

The steps to create the new engine in this case are very similar to those referred above. Just change the type to *json* when invoking the script:

.. ref-code-block:: cpp

	cd tools
	./create_new_engine.py --name Test --type json







.. _doxid-engine_creation_template_1engine_creation_template_client:

Client side
~~~~~~~~~~~



.. _doxid-engine_creation_template_1engine_creation_template_datapack:

DataPacks
---------

In the case of a JSON based Engine, it will always used :ref:`JsonDataPack <doxid-datapacks_1datapacks_json>`, so there is no need to implement new types.

In the case of a gRPC based Engine, by default the new engine created by the script will only accept protobuf messages of the type *EngineTest.TestPayload*. As it names indicate, this is a message definition used for testing purposes. In order to extend the engine to accept other protobuf message definitions:

#. Follow this guide: :ref:`Adding new protobuf message definitions <doxid-tutorial_add_proto_definition>` to make the new definitions available to NRP-core

#. Adapt the engine client class definition (TestEngineGrpcClient) by including in the :ref:`EngineGrpcClient <doxid-class_engine_grpc_client>` template it inherits from the additional protobuf message classes the engine should support (see file *test_grpc_engine/engine_client/test_grpc_client.h* in the *test_grpc_engine* folder)





.. _doxid-engine_creation_template_1engine_creation_template_client_configuration:

Configuration schema
--------------------

By default, the new engines will use the :ref:`grpc engine schema <doxid-engine_comm_1engine_grpc_config_section>` and :ref:`json engine schema <doxid-engine_comm_1engine_json_config_section>` respectively to validate the engine configuration from the experiment configuration file. See :ref:`here <doxid-simulation_configuration>` for more details.

If your engine configuration requires additional parameters, you can create your own schema using the aforementioned schemas as a base. See :ref:`Creating an Engine configuration schema <doxid-tutorial_engine_creation_1tutorial_engine_creation_engine_config>` for more details on how to do this.





.. _doxid-engine_creation_template_1engine_creation_template_client_class:

Client class
------------

The client class doesn't need any modifications in order to work.







.. _doxid-engine_creation_template_1engine_creation_template_server:

Server side
~~~~~~~~~~~



.. _doxid-engine_creation_template_1engine_creation_template_server_class:

Server class
------------

In order to control the simulation, the following methods of the server class must be implemented:

* initialize() - should initialize the simulation. The function will receive full simulation configuration as JSON object.

* runLoopStep() - this function is supposed to advance the simulation by requested time step.

* shutdown() - should gracefully shut down the simulation.

* reset() - optional. Should reset the simulation to its initial state.





.. _doxid-engine_creation_template_1engine_creation_template_server_controllers:

DataPack controllers
--------------------

:ref:`DataPack <doxid-class_data_pack>` controllers (:ref:`DataPackController <doxid-class_data_pack_controller>`) are helper classes which facilitate data exchange between the simulator and the server part of the engine. They can be thought of as adaptors between data arriving from the engine client in form of protobuf objects (gRPC engine) or JSON objects (JSON engine), and internal data from the simulation.

The engines generated using the script provides a single datapack controller class, but more may be added if needed.

The following methods of the controller should be implemented:

* DataPackController::getDataPackInformationCallback() should return a protobuf object (gRPC engine) or a JSON object (JSON engine) with the most recent simulation results. This data will be then passed from the engine server to the client, and from there to the transceiver functions.

* DataPackController::handleDataPackDataCallback() should inject received protobuf or JSON data into the simulator.

