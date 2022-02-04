.. index:: pair: class; NrpCoreServer
.. _doxid-class_nrp_core_server:

class NrpCoreServer
===================

.. toctree::
	:hidden:

	enum_NrpCoreServer_RequestType.rst
	struct_NrpCoreServer_RequestStatus.rst

Overview
~~~~~~~~

NRP Server class, responsible for handling simulation control requests coming from the client application or script. :ref:`More...<details-class_nrp_core_server>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nrp_core_server.h>
	
	class NrpCoreServer: public Service {
	public:
		// enums
	
		enum :ref:`RequestType<doxid-class_nrp_core_server_1a5c39bb778ee26f83e26b2fb6a5d9794b>`;

		// structs
	
		struct :ref:`RequestStatus<doxid-struct_nrp_core_server_1_1_request_status>`;

		// construction
	
		:ref:`NrpCoreServer<doxid-class_nrp_core_server_1a370f24272a9d1c1c5bd491757d4353bc>`(const std::string& address);

		// methods
	
		:ref:`RequestType<doxid-class_nrp_core_server_1a5c39bb778ee26f83e26b2fb6a5d9794b>` :ref:`getRequestType<doxid-class_nrp_core_server_1a15853f1da23de05ed97df189db8c9b6e>`() const;
		unsigned :ref:`getNumIterations<doxid-class_nrp_core_server_1a969e6f33637866aeee3619ed4a75141c>`() const;
		void :ref:`waitForRequest<doxid-class_nrp_core_server_1af5d22eb7b393f335ba1ea3026389c0e5>`();
		void :ref:`markRequestAsProcessed<doxid-class_nrp_core_server_1a71b3318e52960f9b07414c22a5d17001>`();
		void :ref:`markRequestAsFailed<doxid-class_nrp_core_server_1addbe4268e67f934b6ce1fc313f2d2352>`(const std::string& errorMessage);
	};
.. _details-class_nrp_core_server:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

NRP Server class, responsible for handling simulation control requests coming from the client application or script.

The class has two roles:

* receive simulation control requests from the client application

* relay received requests to the main thread of NRP Core and wait for response

The class is an instance of gRPC Service, which means that it will have a pool of worker threads. Every request coming from the client will be handled by a separate thread from the pool. The natural approach to request handling would be to call proper methods of the :ref:`SimulationManager <doxid-class_simulation_manager>` directly from the callbacks (for example, :ref:`SimulationManager::initFTILoop() <doxid-class_simulation_manager_1a34e9ae849be6d01c1df8a39d2e61bbbf>` from NrpCoreServer::init()). This proved to cause problems with certain python libraries, like OpenCV, that may be used in the transceiver functions. The problem seems to arise from the fact, that the python code was executed in a worker thread, and not in the thread that spawned the python interpreter (the main thread of NRP Core). The solution is to create a producer-consumer relation between the NRP Server threads and the main thread. The NRP Server relays (produces) the requests to the main thread, which consumes them and returns a status back to the NRP Server.

Construction
------------

.. index:: pair: function; NrpCoreServer
.. _doxid-class_nrp_core_server_1a370f24272a9d1c1c5bd491757d4353bc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	NrpCoreServer(const std::string& address)

Constructor. Spawns an instance of gRPC server with given address.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- address

		- Address of the gRPC server

Methods
-------

.. index:: pair: function; getRequestType
.. _doxid-class_nrp_core_server_1a15853f1da23de05ed97df189db8c9b6e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`RequestType<doxid-class_nrp_core_server_1a5c39bb778ee26f83e26b2fb6a5d9794b>` getRequestType() const

Returns the type of request coming from the client.

.. index:: pair: function; getNumIterations
.. _doxid-class_nrp_core_server_1a969e6f33637866aeee3619ed4a75141c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	unsigned getNumIterations() const

Returns number of iterations of runLoop requested by the client.

.. index:: pair: function; waitForRequest
.. _doxid-class_nrp_core_server_1af5d22eb7b393f335ba1ea3026389c0e5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void waitForRequest()

Puts the current thread to sleep until a request is available.

The function is supposed to be called by the consumer thread. It should be called in combination with markRequestAsProcessed function.

.. index:: pair: function; markRequestAsProcessed
.. _doxid-class_nrp_core_server_1a71b3318e52960f9b07414c22a5d17001:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void markRequestAsProcessed()

Signals the server that the request has been processed.

The function is supposed to be called by the consumer thread, after the pending request has been consumed. If the function isn't called, the producer thread will not wake up! It must be called after waitForRequest function.

.. index:: pair: function; markRequestAsFailed
.. _doxid-class_nrp_core_server_1addbe4268e67f934b6ce1fc313f2d2352:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void markRequestAsFailed(const std::string& errorMessage)

Signals the server, that there was an error during request handling.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- errorMessage

		- Message describing what went wrong

