.. index:: pair: class; EngineJSONServer
.. _doxid-class_engine_j_s_o_n_server:

class EngineJSONServer
======================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Manages communication with the NRP. Uses a REST server to send/receive data. Singleton class. :ref:`More...<details-class_engine_j_s_o_n_server>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_json_server.h>
	
	class EngineJSONServer {
	public:
		// typedefs
	
		typedef std::timed_mutex :target:`mutex_t<doxid-class_engine_j_s_o_n_server_1a5df75e9fa8a25592e4e3ad7064362673>`;
		typedef std::unique_lock<:ref:`EngineJSONServer::mutex_t<doxid-class_engine_j_s_o_n_server_1a5df75e9fa8a25592e4e3ad7064362673>`> :target:`lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`;

		// construction
	
		:ref:`EngineJSONServer<doxid-class_engine_j_s_o_n_server_1ac9d6e1071b1d64cdf6585a6a97e66922>`(
			const std::string& engineAddress,
			const std::string& engineName,
			const std::string& clientAddress
		);
	
		:ref:`EngineJSONServer<doxid-class_engine_j_s_o_n_server_1a8c39adf2c74c1001b8f5cae253d172bc>`(const std::string& engineAddress);
		:target:`EngineJSONServer<doxid-class_engine_j_s_o_n_server_1aa71f6fac82012eae03d5ed9ef69987f6>`();
		:target:`EngineJSONServer<doxid-class_engine_j_s_o_n_server_1a08e1f6ef0c1188776e4cebba77f41889>`(const EngineJSONServer&);

		// methods
	
		EngineJSONServer& :target:`operator =<doxid-class_engine_j_s_o_n_server_1acfacf46dfa94ab1b35047f8ef4ff0d64>` (const EngineJSONServer&);
		bool :ref:`isServerRunning<doxid-class_engine_j_s_o_n_server_1a0f65133b9a3a09f6bd2141d135d21e7c>`() const;
		void :ref:`startServerAsync<doxid-class_engine_j_s_o_n_server_1a33acfbda554050d3d47cab00e3d4869e>`();
		void :ref:`startServer<doxid-class_engine_j_s_o_n_server_1a10dbdbcbf4ee934d1b7740c4e3bb11d7>`();
		void :ref:`shutdownServer<doxid-class_engine_j_s_o_n_server_1ad3a67a6b3fb7889725d8474b8631eb10>`();
		uint16_t :ref:`serverPort<doxid-class_engine_j_s_o_n_server_1add529a3d33c10daff532d50790f206e6>`() const;
		std::string :ref:`serverAddress<doxid-class_engine_j_s_o_n_server_1ab98b796faed11c13f43300fc8fdbacb2>`() const;
	
		void :ref:`registerDataPack<doxid-class_engine_j_s_o_n_server_1a20aa263c3e4605edc9b461b270bd6b3b>`(
			const std::string& datapackName,
			:ref:`JsonDataPackController<doxid-class_json_data_pack_controller>`* interface
		);
	
		void :ref:`registerDataPackNoLock<doxid-class_engine_j_s_o_n_server_1a64df4ef44c6a98b8d125cd77bd43bcb6>`(
			const std::string& datapackName,
			:ref:`JsonDataPackController<doxid-class_json_data_pack_controller>`* interface
		);
	
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`runLoopStep<doxid-class_engine_j_s_o_n_server_1a0345dec840e4827786442050a6589787>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep) = 0;
	
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`initialize<doxid-class_engine_j_s_o_n_server_1ac8df612106fa7d2d92663f8fd6cfcfe5>`(
			const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data,
			:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock
		) = 0;
	
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`reset<doxid-class_engine_j_s_o_n_server_1a76b6062e7a83a6b5adefb01c07547de0>`(:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock) = 0;
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`shutdown<doxid-class_engine_j_s_o_n_server_1a83f8f54978e715ce341af674e1813f90>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data) = 0;
	};

	// direct descendants

	class :ref:`NestJSONServer<doxid-class_nest_j_s_o_n_server>`;
	class :ref:`PythonJSONServer<doxid-class_python_j_s_o_n_server>`;
.. _details-class_engine_j_s_o_n_server:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Manages communication with the NRP. Uses a REST server to send/receive data. Singleton class.

Construction
------------

.. index:: pair: function; EngineJSONServer
.. _doxid-class_engine_j_s_o_n_server_1ac9d6e1071b1d64cdf6585a6a97e66922:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	EngineJSONServer(
		const std::string& engineAddress,
		const std::string& engineName,
		const std::string& clientAddress
	)

Constructor. Tries to bind to a port and register itself with clientAddress.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineAddress

		- Server Address. If it contains a port, will try to bind to said port. If that fails, will increment port number and try again. This will continue for at most :ref:`EngineJSONConfigConst::MaxAddrBindTries <doxid-struct_engine_j_s_o_n_config_const_1a13f8dce8a46ede8d689107a8accd25d9>` times

	*
		- engineName

		- Engine Name

	*
		- clientAddress

		- Client Address. The server will try to register itself under this address

.. index:: pair: function; EngineJSONServer
.. _doxid-class_engine_j_s_o_n_server_1a8c39adf2c74c1001b8f5cae253d172bc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	EngineJSONServer(const std::string& engineAddress)

Constructor. Will try to bind to engineAddress.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineAddress

		- Server address

Methods
-------

.. index:: pair: function; isServerRunning
.. _doxid-class_engine_j_s_o_n_server_1a0f65133b9a3a09f6bd2141d135d21e7c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool isServerRunning() const

Is the server running?



.. rubric:: Returns:

Returns true if the server is running

.. index:: pair: function; startServerAsync
.. _doxid-class_engine_j_s_o_n_server_1a33acfbda554050d3d47cab00e3d4869e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void startServerAsync()

Start the server in asynchronous mode.

.. index:: pair: function; startServer
.. _doxid-class_engine_j_s_o_n_server_1a10dbdbcbf4ee934d1b7740c4e3bb11d7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void startServer()

Start the server synchronously.

.. index:: pair: function; shutdownServer
.. _doxid-class_engine_j_s_o_n_server_1ad3a67a6b3fb7889725d8474b8631eb10:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void shutdownServer()

Stop running server.

.. index:: pair: function; serverPort
.. _doxid-class_engine_j_s_o_n_server_1add529a3d33c10daff532d50790f206e6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	uint16_t serverPort() const

Get running server port.



.. rubric:: Returns:

Returns port of running server, 0 if server is not running

.. index:: pair: function; serverAddress
.. _doxid-class_engine_j_s_o_n_server_1ab98b796faed11c13f43300fc8fdbacb2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::string serverAddress() const

Get server address.

.. index:: pair: function; registerDataPack
.. _doxid-class_engine_j_s_o_n_server_1a20aa263c3e4605edc9b461b270bd6b3b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerDataPack(
		const std::string& datapackName,
		:ref:`JsonDataPackController<doxid-class_json_data_pack_controller>`* interface
	)

Registers a datapack.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackName

		- Name of datapack

	*
		- interface

		- Pointer to interface

.. index:: pair: function; registerDataPackNoLock
.. _doxid-class_engine_j_s_o_n_server_1a64df4ef44c6a98b8d125cd77bd43bcb6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerDataPackNoLock(
		const std::string& datapackName,
		:ref:`JsonDataPackController<doxid-class_json_data_pack_controller>`* interface
	)

Registers a datapack. Skips locking the mutex. Should only be used if thread-safe access to _datapacksControllers can be guaranteed.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackName

		- Name of datapack

	*
		- interface

		- Pointer to interface

.. index:: pair: function; runLoopStep
.. _doxid-class_engine_j_s_o_n_server_1a0345dec840e4827786442050a6589787:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runLoopStep(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep) = 0

Run a single loop step.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timeStep

		- Step to take



.. rubric:: Returns:

Returns the time registered by this engine at the end of the loop

.. index:: pair: function; initialize
.. _doxid-class_engine_j_s_o_n_server_1ac8df612106fa7d2d92663f8fd6cfcfe5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` initialize(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data,
		:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock
	) = 0

Engine Initialization routine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- data

		- Initialization data

	*
		- datapackLock

		- :ref:`DataPack <doxid-class_data_pack>` Lock. Prevents access to _datapacksControllers



.. rubric:: Returns:

Returns data about initialization status

.. index:: pair: function; reset
.. _doxid-class_engine_j_s_o_n_server_1a76b6062e7a83a6b5adefb01c07547de0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` reset(:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock) = 0

Engine reset routine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackLock

		- :ref:`DataPack <doxid-class_data_pack>` Lock. Prevents access to _datapacksControllers



.. rubric:: Returns:

Returns data about initialization status

.. index:: pair: function; shutdown
.. _doxid-class_engine_j_s_o_n_server_1a83f8f54978e715ce341af674e1813f90:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` shutdown(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data) = 0

Engine Shutdown routine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- data

		- Shutdown data



.. rubric:: Returns:

Returns data about shutdown status

