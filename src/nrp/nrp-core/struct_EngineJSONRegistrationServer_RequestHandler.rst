.. index:: pair: struct; EngineJSONRegistrationServer::RequestHandler
.. _doxid-struct_engine_j_s_o_n_registration_server_1_1_request_handler:

struct EngineJSONRegistrationServer::RequestHandler
===================================================

.. toctree::
	:hidden:

Struct to handle REST calls.


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	struct RequestHandler: public Handler {
		// construction
	
		:target:`RequestHandler<doxid-struct_engine_j_s_o_n_registration_server_1_1_request_handler_1ad553201a274079f520c2931c3623ceaa>`(:ref:`EngineJSONRegistrationServer<doxid-class_engine_j_s_o_n_registration_server>`* pServer);

		// methods
	
		:target:`HTTP_PROTOTYPE<doxid-struct_engine_j_s_o_n_registration_server_1_1_request_handler_1a3096d40636d97ceee1f463ed5cea220a>`(RequestHandler);
	
		void :target:`onRequest<doxid-struct_engine_j_s_o_n_registration_server_1_1_request_handler_1a367cc0e22a524e787dae7aa9f9d5c66f>`(
			const Pistache::Http::Request& req,
			Pistache::Http::ResponseWriter response
		);
	};
