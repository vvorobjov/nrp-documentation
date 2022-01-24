.. index:: pair: class; EngineJSONRegistrationServer
.. _doxid-class_engine_j_s_o_n_registration_server:

class EngineJSONRegistrationServer
==================================

.. toctree::
	:hidden:

	struct_EngineJSONRegistrationServer_RequestHandler.rst

Overview
~~~~~~~~

Singleton. Creates an HTTP REST server to register newly created EngineJSONServers and store their addresses. :ref:`More...<details-class_engine_j_s_o_n_registration_server>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_json_registration_server.h>
	
	class EngineJSONRegistrationServer {
	public:
		// structs
	
		struct :ref:`RequestHandler<doxid-struct_engine_j_s_o_n_registration_server_1_1_request_handler>`;

		// fields
	
		static constexpr std::string_view :ref:`JSONEngineName<doxid-class_engine_j_s_o_n_registration_server_1a35616f199c4824ab392ba46419f20d4e>` = "engine_name";
		static constexpr std::string_view :ref:`JSONAddress<doxid-class_engine_j_s_o_n_registration_server_1ac9350fd41611da35b3d027706aded2f0>` = "address";

		// construction
	
		:target:`EngineJSONRegistrationServer<doxid-class_engine_j_s_o_n_registration_server_1aa2495fa9dc05b3d7d2941ed8da4828c4>`(const EngineJSONRegistrationServer&);
		:target:`EngineJSONRegistrationServer<doxid-class_engine_j_s_o_n_registration_server_1a020b5ee43afe2ec551e7fa2666172bc6>`(EngineJSONRegistrationServer&&);

		// methods
	
		static EngineJSONRegistrationServer* :ref:`getInstance<doxid-class_engine_j_s_o_n_registration_server_1a01804bc675e83fbe9a13beb6c3ad5e0a>`();
		static EngineJSONRegistrationServer* :ref:`resetInstance<doxid-class_engine_j_s_o_n_registration_server_1ae9547c569a289f60d1cbe63f14a38534>`(const std::string& serverAddress);
		static void :ref:`clearInstance<doxid-class_engine_j_s_o_n_registration_server_1ae359e0c3b9e18ac4389aaa63e59c26c7>`();
	
		static bool :ref:`sendClientEngineRequest<doxid-class_engine_j_s_o_n_registration_server_1a6885323dcb8530362e6e6944f44d0b99>`(
			const std::string& address,
			const engine_name_t& engineName,
			const std::string& engineAddress,
			const unsigned int numTries = 1,
			const unsigned int waitTime = 0
		);
	
		EngineJSONRegistrationServer& :target:`operator =<doxid-class_engine_j_s_o_n_registration_server_1a43eec2caef264530b7280585fdd3191b>` (const EngineJSONRegistrationServer&);
		EngineJSONRegistrationServer& :target:`operator =<doxid-class_engine_j_s_o_n_registration_server_1a9cb38a5bf588160b415544118c97622f>` (EngineJSONRegistrationServer&&);
		void :ref:`startServerAsync<doxid-class_engine_j_s_o_n_registration_server_1a016279cbccdd31b35e3ae9963f8f9a9b>`();
		void :ref:`shutdownServer<doxid-class_engine_j_s_o_n_registration_server_1aaad1ad1cf0417e9df8cee0002b2472a7>`();
		const std::string :ref:`serverAddress<doxid-class_engine_j_s_o_n_registration_server_1a2811eb1e246bae09305537325a64ea82>`() const;
		bool :ref:`isRunning<doxid-class_engine_j_s_o_n_registration_server_1aa76af550e1b1f51f0fc2af978cbe8543>`() const;
		size_t :ref:`getNumWaitingEngines<doxid-class_engine_j_s_o_n_registration_server_1a7ca7f180ee20ad5f9c15396dafb97f45>`();
		std::string :ref:`retrieveEngineAddress<doxid-class_engine_j_s_o_n_registration_server_1adb9a0ea5659ef28f09c229ff671489dc>`(const engine_name_t& engineName);
		std::string :ref:`requestEngine<doxid-class_engine_j_s_o_n_registration_server_1a7c1c8ee2ecc0417e1f21dd637678df5c>`(const engine_name_t& engineName);
	
		void :ref:`registerEngineAddress<doxid-class_engine_j_s_o_n_registration_server_1aee4adaba680e0aceeee432fc12bb3679>`(
			const engine_name_t& engineName,
			const std::string& address
		);
	};
.. _details-class_engine_j_s_o_n_registration_server:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Singleton. Creates an HTTP REST server to register newly created EngineJSONServers and store their addresses.

Fields
------

.. index:: pair: variable; JSONEngineName
.. _doxid-class_engine_j_s_o_n_registration_server_1a35616f199c4824ab392ba46419f20d4e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view JSONEngineName = "engine_name"

JSON Engine Name locator used during registration.

.. index:: pair: variable; JSONAddress
.. _doxid-class_engine_j_s_o_n_registration_server_1ac9350fd41611da35b3d027706aded2f0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view JSONAddress = "address"

JSON Engine Address locator used during registration.

Methods
-------

.. index:: pair: function; getInstance
.. _doxid-class_engine_j_s_o_n_registration_server_1a01804bc675e83fbe9a13beb6c3ad5e0a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static EngineJSONRegistrationServer* getInstance()

Get Instance of :ref:`EngineJSONRegistrationServer <doxid-class_engine_j_s_o_n_registration_server>`.



.. rubric:: Returns:

Returns ptr to :ref:`EngineJSONRegistrationServer <doxid-class_engine_j_s_o_n_registration_server>` if it exists, nullptr otherwise

.. index:: pair: function; resetInstance
.. _doxid-class_engine_j_s_o_n_registration_server_1ae9547c569a289f60d1cbe63f14a38534:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static EngineJSONRegistrationServer* resetInstance(const std::string& serverAddress)

Reset :ref:`EngineJSONRegistrationServer <doxid-class_engine_j_s_o_n_registration_server>` with the given address.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- serverAddress

		- Server Address to bind to



.. rubric:: Returns:

Returns pointer to created instance

.. index:: pair: function; clearInstance
.. _doxid-class_engine_j_s_o_n_registration_server_1ae359e0c3b9e18ac4389aaa63e59c26c7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static void clearInstance()

Delete Instance.

.. index:: pair: function; sendClientEngineRequest
.. _doxid-class_engine_j_s_o_n_registration_server_1a6885323dcb8530362e6e6944f44d0b99:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static bool sendClientEngineRequest(
		const std::string& address,
		const engine_name_t& engineName,
		const std::string& engineAddress,
		const unsigned int numTries = 1,
		const unsigned int waitTime = 0
	)

Send Engine Name and address to specified address.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- address

		- Address to send data to

	*
		- engineName

		- Name of engine

	*
		- engineAddress

		- Address of engine

	*
		- numTries

		- Number of times to try and contact the registration server

	*
		- waitTime

		- Time (in seconds) to wait between contact attempts



.. rubric:: Returns:

Returns true on success, false otherwise

.. index:: pair: function; startServerAsync
.. _doxid-class_engine_j_s_o_n_registration_server_1a016279cbccdd31b35e3ae9963f8f9a9b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void startServerAsync()

Start the server if it's not already running.

.. index:: pair: function; shutdownServer
.. _doxid-class_engine_j_s_o_n_registration_server_1aaad1ad1cf0417e9df8cee0002b2472a7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void shutdownServer()

Stop the Server.

.. index:: pair: function; serverAddress
.. _doxid-class_engine_j_s_o_n_registration_server_1a2811eb1e246bae09305537325a64ea82:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const std::string serverAddress() const

Get server address.

.. index:: pair: function; isRunning
.. _doxid-class_engine_j_s_o_n_registration_server_1aa76af550e1b1f51f0fc2af978cbe8543:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool isRunning() const

Returns true when server is running, false otherwise.

.. index:: pair: function; getNumWaitingEngines
.. _doxid-class_engine_j_s_o_n_registration_server_1a7ca7f180ee20ad5f9c15396dafb97f45:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	size_t getNumWaitingEngines()

Get the number of engines that are still waiting for registration.

.. index:: pair: function; retrieveEngineAddress
.. _doxid-class_engine_j_s_o_n_registration_server_1adb9a0ea5659ef28f09c229ff671489dc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::string retrieveEngineAddress(const engine_name_t& engineName)

Retrieve a registered engine address. If available and non-empty, erase it from _registeredAddresses.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineName

		- Engine Name for which to find the address



.. rubric:: Returns:

If address available, return it. Otherwise return empty string

.. index:: pair: function; requestEngine
.. _doxid-class_engine_j_s_o_n_registration_server_1a7c1c8ee2ecc0417e1f21dd637678df5c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::string requestEngine(const engine_name_t& engineName)

Request an engine's address. If available, erases entry from _registeredAddresses.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineName

		- Name of engine to wait for



.. rubric:: Returns:

If available, returns name of engine. Else, returns empty string

.. index:: pair: function; registerEngineAddress
.. _doxid-class_engine_j_s_o_n_registration_server_1aee4adaba680e0aceeee432fc12bb3679:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerEngineAddress(
		const engine_name_t& engineName,
		const std::string& address
	)

Register an engine's address.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineName

		- Name of engine

	*
		- address

		- Address of engine

