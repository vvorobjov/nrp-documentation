.. index:: pair: class; EngineGrpcServer
.. _doxid-class_engine_grpc_server:

class EngineGrpcServer
======================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Abstract class for Engine server with gRPC support. :ref:`More...<details-class_engine_grpc_server>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_grpc_server.h>
	
	class EngineGrpcServer: public Service {
	public:
		// typedefs
	
		typedef std::timed_mutex :target:`mutex_t<doxid-class_engine_grpc_server_1ab40542dacc855a02ffcc1ef23157ec90>`;
		typedef std::unique_lock<:ref:`EngineGrpcServer::mutex_t<doxid-class_engine_grpc_server_1ab40542dacc855a02ffcc1ef23157ec90>`> :target:`lock_t<doxid-class_engine_grpc_server_1a07a6378e03bd4eacbb3f5255c225744e>`;

		// construction
	
		:target:`EngineGrpcServer<doxid-class_engine_grpc_server_1a4ae425be5ee3b8cc9a07f33eb8aba14c>`(const std::string& address);
	
		:ref:`EngineGrpcServer<doxid-class_engine_grpc_server_1ac868ec53ce1d37e823f2b13bfc593b0a>`(
			const std::string serverAddress,
			const std::string& engineName,
			const std::string&
		);

		// methods
	
		void :ref:`startServer<doxid-class_engine_grpc_server_1a578b6d98acd6e1b102a131a6b09ce1bd>`();
		void :ref:`startServerAsync<doxid-class_engine_grpc_server_1a873debfc6573e9b5d343bf9fff970d0b>`();
		void :ref:`shutdownServer<doxid-class_engine_grpc_server_1ae5bef34e951ed424c5ad52d10babfcc0>`();
		bool :ref:`isServerRunning<doxid-class_engine_grpc_server_1a3016dbda087eaaef054472de914093e1>`() const;
		const std::string :ref:`serverAddress<doxid-class_engine_grpc_server_1acd608554e1b2f520c75b7d77652e1c58>`() const;
	
		void :ref:`registerDataPack<doxid-class_engine_grpc_server_1a50502d2f65705c5d8bdeb3de6f29d8fc>`(
			const std::string& datapackName,
			:ref:`ProtoDataPackController<doxid-engine__grpc__server_8h_1a8b6f823dadc78cb7cb8e59f426810363>`* interface
		);
	
		void :target:`registerDataPackNoLock<doxid-class_engine_grpc_server_1ae1966279b174aad6b985f25890ed5f1e>`(
			const std::string& datapackName,
			:ref:`ProtoDataPackController<doxid-engine__grpc__server_8h_1a8b6f823dadc78cb7cb8e59f426810363>`* interface
		);
	
		unsigned :target:`getNumRegisteredDataPacks<doxid-class_engine_grpc_server_1a0fe86763e3a6e67c64829a207f68f7c4>`();
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
.. _doxid-class_engine_grpc_server_1ac868ec53ce1d37e823f2b13bfc593b0a:

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
.. _doxid-class_engine_grpc_server_1a578b6d98acd6e1b102a131a6b09ce1bd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void startServer()

Starts the gRPC server in synchronous mode.

.. index:: pair: function; startServerAsync
.. _doxid-class_engine_grpc_server_1a873debfc6573e9b5d343bf9fff970d0b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void startServerAsync()

Starts the gRPC server in asynchronous mode.

.. index:: pair: function; shutdownServer
.. _doxid-class_engine_grpc_server_1ae5bef34e951ed424c5ad52d10babfcc0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void shutdownServer()

Shutdowns the gRPC server.

.. index:: pair: function; isServerRunning
.. _doxid-class_engine_grpc_server_1a3016dbda087eaaef054472de914093e1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool isServerRunning() const

Indicates whether the gRPC server is currently running.

.. index:: pair: function; serverAddress
.. _doxid-class_engine_grpc_server_1acd608554e1b2f520c75b7d77652e1c58:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const std::string serverAddress() const

Returns address of the gRPC server.

.. index:: pair: function; registerDataPack
.. _doxid-class_engine_grpc_server_1a50502d2f65705c5d8bdeb3de6f29d8fc:

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

