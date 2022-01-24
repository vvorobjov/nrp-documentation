.. index:: pair: class; EngineGrpcServer
.. _doxid-class_engine_grpc_server:

template class EngineGrpcServer
===============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Abstract class for Engine server with gRPC support. :ref:`More...<details-class_engine_grpc_server>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_grpc_server.h>
	
	template <class ... MSG_TYPES>
	class EngineGrpcServer: public Service {
	public:
		// typedefs
	
		typedef std::timed_mutex :target:`mutex_t<doxid-class_engine_grpc_server_1a8c31ad3bdbbbdd8b18a19c43a20ce1c1>`;
		typedef std::unique_lock<:ref:`EngineGrpcServer::mutex_t<doxid-class_engine_grpc_server_1a8c31ad3bdbbbdd8b18a19c43a20ce1c1>`> :target:`lock_t<doxid-class_engine_grpc_server_1a649df914ed68119cfa914a9bf980dcf9>`;

		// construction
	
		:target:`EngineGrpcServer<doxid-class_engine_grpc_server_1acc7f484e281b9a0ecfa663037f0abeda>`(const std::string& address);
	
		:ref:`EngineGrpcServer<doxid-class_engine_grpc_server_1a9d1ce9177dfedb7714a4e780a30f134a>`(
			const std::string serverAddress,
			const std::string& engineName,
			const std::string&
		);

		// methods
	
		void :ref:`startServer<doxid-class_engine_grpc_server_1a1298b6f1e7447038a138ae69dbbfdd1e>`();
		void :ref:`startServerAsync<doxid-class_engine_grpc_server_1a933a300fa47c9817e1b9ec3125e11879>`();
		void :ref:`shutdownServer<doxid-class_engine_grpc_server_1a7e1356da8d00515328d178cc72ea9a7d>`();
		bool :ref:`isServerRunning<doxid-class_engine_grpc_server_1a30b5e327538546a6deeae17049b257e9>`() const;
		const std::string :ref:`serverAddress<doxid-class_engine_grpc_server_1abf20d0c8cb7a0e5d61d8c60131fbc389>`() const;
	
		void :ref:`registerDataPack<doxid-class_engine_grpc_server_1a69859d163d1aff3cd9ec3947f6ba1cc6>`(
			const std::string& datapackName,
			:ref:`ProtoDataPackController<doxid-engine__grpc__server_8h_1a8b6f823dadc78cb7cb8e59f426810363>`* interface
		);
	
		void :target:`registerDataPackNoLock<doxid-class_engine_grpc_server_1a66dca0d25b7db065ea2a8fef951a19da>`(
			const std::string& datapackName,
			:ref:`ProtoDataPackController<doxid-engine__grpc__server_8h_1a8b6f823dadc78cb7cb8e59f426810363>`* interface
		);
	
		unsigned :target:`getNumRegisteredDataPacks<doxid-class_engine_grpc_server_1aa1d06fc74495f845da2241ad42c2a534>`();
	};

	// direct descendants

	class :ref:`NRPCommunicationController<doxid-class_n_r_p_communication_controller>`;
.. _details-class_engine_grpc_server:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Abstract class for Engine server with gRPC support.

The class provides a base for implementing an Engine server with gRPC as middleware. All RPC services are implemented. Derived classes are responsible for implementing simulation initialization, shutdown and run step methods.

Construction
------------

.. index:: pair: function; EngineGrpcServer
.. _doxid-class_engine_grpc_server_1a9d1ce9177dfedb7714a4e780a30f134a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	EngineGrpcServer(
		const std::string serverAddress,
		const std::string& engineName,
		const std::string&
	)

Constructor.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- serverAddress

		- Address of the gRPC server

	*
		- engineName

		- Name of the simulation engine

	*
		- registrationAddress

		- Should be removed

Methods
-------

.. index:: pair: function; startServer
.. _doxid-class_engine_grpc_server_1a1298b6f1e7447038a138ae69dbbfdd1e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void startServer()

Starts the gRPC server in synchronous mode.

.. index:: pair: function; startServerAsync
.. _doxid-class_engine_grpc_server_1a933a300fa47c9817e1b9ec3125e11879:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void startServerAsync()

Starts the gRPC server in asynchronous mode.

.. index:: pair: function; shutdownServer
.. _doxid-class_engine_grpc_server_1a7e1356da8d00515328d178cc72ea9a7d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void shutdownServer()

Shutdowns the gRPC server.

.. index:: pair: function; isServerRunning
.. _doxid-class_engine_grpc_server_1a30b5e327538546a6deeae17049b257e9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool isServerRunning() const

Indicates whether the gRPC server is currently running.

.. index:: pair: function; serverAddress
.. _doxid-class_engine_grpc_server_1abf20d0c8cb7a0e5d61d8c60131fbc389:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const std::string serverAddress() const

Returns address of the gRPC server.

.. index:: pair: function; registerDataPack
.. _doxid-class_engine_grpc_server_1a69859d163d1aff3cd9ec3947f6ba1cc6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerDataPack(
		const std::string& datapackName,
		:ref:`ProtoDataPackController<doxid-engine__grpc__server_8h_1a8b6f823dadc78cb7cb8e59f426810363>`* interface
	)

Registers a datapack controller with the given name in the engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackName

		- Name of the datapack to be registered

	*
		- datapackController

		- Pointer to the datapack controller object that's supposed to be registered in the engine

