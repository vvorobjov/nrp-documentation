.. index:: pair: class; EngineClient
.. _doxid-class_engine_client:

template class EngineClient
===========================

.. toctree::
	:hidden:

	class_EngineClient_EngineLauncher.rst

Overview
~~~~~~~~

Base class for all Engines. :ref:`More...<details-class_engine_client>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_client_interface.h>
	
	template <class ENGINE, const char* SCHEMA>
	class EngineClient: public :ref:`EngineClientInterface<doxid-class_engine_client_interface>` {
	public:
		// typedefs
	
		typedef ENGINE :target:`engine_t<doxid-class_engine_client_1ae642237dab2cde85069f302ed91a6f73>`;

		// classes
	
		template <const char* ENGINE_TYPE>
		class :ref:`EngineLauncher<doxid-class_engine_client_1_1_engine_launcher>`;

		// construction
	
		:ref:`EngineClient<doxid-class_engine_client_1a30252c2688ef6eadf51fdf08669a749e>`(
			:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
			:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`&& launcher
		);

		// methods
	
		virtual const std::string :ref:`engineName<doxid-class_engine_client_1a04f80dd3fbb46b0056d825addb2231d7>`() const;
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`getEngineTimestep<doxid-class_engine_client_1a1089097a855a6ab0b4b27605529e1231>`() const;
		virtual const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& :ref:`engineConfig<doxid-class_engine_client_1a5495185601ad8529d112df7591e18b69>`() const;
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& :ref:`engineConfig<doxid-class_engine_client_1a645831aa5aae4233085c3739c0669992>`();
		virtual const std::string :ref:`engineSchema<doxid-class_engine_client_1a356309d9f16c2d0c38002f7978400370>`() const;
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`getEngineTime<doxid-class_engine_client_1a9b05083f880d664d4a4eaaba5e461584>`() const;
		virtual void :ref:`runLoopStepAsync<doxid-class_engine_client_1a4b95a41aa73bbc8367d7acf0f47c2756>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep);
		virtual void :ref:`runLoopStepAsyncGet<doxid-class_engine_client_1ae94c9afd2b99f20dff28c3138e3eb5b1>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeOut);
	};

	// direct descendants

	template <class ENGINE, const char* SCHEMA, class ... MSG_TYPES>
	class :ref:`EngineGrpcClient<doxid-class_engine_grpc_client>`;

	template <class ENGINE, const char* SCHEMA>
	class :ref:`EngineJSONNRPClient<doxid-class_engine_j_s_o_n_n_r_p_client>`;

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// typedefs
	
		typedef std::shared_ptr<T> :ref:`shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`;
		typedef std::shared_ptr<const T> :ref:`const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>`;
		typedef std::unique_ptr<T> :ref:`unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`;
		typedef std::unique_ptr<const T> :ref:`const_unique_ptr<doxid-class_ptr_templates_1aef0eb44f9c386dbf0de54d0f5afac667>`;
		typedef std::set<:ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`> :ref:`datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`;
		typedef std::vector<:ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>`> :ref:`datapacks_t<doxid-class_engine_client_interface_1a2562bba1cd7ecc32dc6bfd54691c669c>`;
		typedef std::set<:ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>`, CompareDevInt> :ref:`datapacks_set_t<doxid-class_engine_client_interface_1a37935f71194e675f096932e0e0afc4b5>`;
		typedef std::vector<:ref:`DataPackInterface<doxid-class_data_pack_interface>`*> :ref:`datapacks_ptr_t<doxid-class_engine_client_interface_1a54f3c8965ef0b6c1ef52dbc0d9d918e8>`;

		// structs
	
		struct :ref:`CompareDevInt<doxid-struct_engine_client_interface_1_1_compare_dev_int>`;

		// methods
	
		virtual const std::string :ref:`engineName<doxid-class_engine_client_interface_1abb12cc28100d5fb2baddbad92d0293f6>`() const = 0;
		virtual const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& :ref:`engineConfig<doxid-class_engine_client_interface_1a176fcd0c5de87a149574999c53461ac2>`() const = 0;
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& :ref:`engineConfig<doxid-class_engine_client_interface_1a450994aebcb9dda0252d08d14fbde89a>`() = 0;
		virtual const std::vector<std::string> :ref:`engineProcStartParams<doxid-class_engine_client_interface_1a6747137f2b551040adca807e6df38a59>`() const = 0;
		virtual const std::vector<std::string> :ref:`engineProcEnvParams<doxid-class_engine_client_interface_1ac3bf04a627785a082fbe83a5aa004227>`() const = 0;
		virtual pid_t :ref:`launchEngine<doxid-class_engine_client_interface_1a42dd02dc80abcc1f48dccf9da0ce2f0c>`();
		virtual void :ref:`initialize<doxid-class_engine_client_interface_1ac600fd036f83cc1aa0ae8fa79b176b44>`() = 0;
		virtual void :ref:`reset<doxid-class_engine_client_interface_1a65d86ba09fd72e32b0399ca290c38632>`() = 0;
		virtual void :ref:`shutdown<doxid-class_engine_client_interface_1a0a15d1d539bc8134f8ed62668c284883>`() = 0;
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`getEngineTimestep<doxid-class_engine_client_interface_1a292b03422ca28976dc3d70f90e11d4e4>`() const = 0;
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`getEngineTime<doxid-class_engine_client_interface_1ab3080a4a253a676d9be0205a3abe9224>`() const = 0;
		virtual const std::string :ref:`engineSchema<doxid-class_engine_client_interface_1a93fc42d5d932c15f856e3db395dac5d5>`() const = 0;
		virtual void :ref:`runLoopStepAsync<doxid-class_engine_client_interface_1aabe6d06f4b2272422b782aef2ba7df4b>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep) = 0;
		virtual void :ref:`runLoopStepAsyncGet<doxid-class_engine_client_interface_1af7df79c71a0b87597b1638b07384c4e9>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeOut) = 0;
		const :ref:`datapacks_t<doxid-class_engine_client_interface_1a2562bba1cd7ecc32dc6bfd54691c669c>`& :ref:`updateDataPacksFromEngine<doxid-class_engine_client_interface_1afeb9a1af5db6f59d09c93f97afc11895>`(const :ref:`datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`& datapackIdentifiers);
		constexpr const :ref:`datapacks_t<doxid-class_engine_client_interface_1a2562bba1cd7ecc32dc6bfd54691c669c>`& :ref:`getCachedDataPacks<doxid-class_engine_client_interface_1a9b0a665916043faaf250df4b06fdece4>`() const;
		virtual void :ref:`sendDataPacksToEngine<doxid-class_engine_client_interface_1a72110362e024889f75d55c3a80696da3>`(const :ref:`datapacks_ptr_t<doxid-class_engine_client_interface_1a54f3c8965ef0b6c1ef52dbc0d9d918e8>`& datapacksArray) = 0;
		void :ref:`updateCachedDataPacks<doxid-class_engine_client_interface_1a5b95f18bb550700360ca408bc0e51583>`(:ref:`datapacks_set_t<doxid-class_engine_client_interface_1a37935f71194e675f096932e0e0afc4b5>`&& devs);
		virtual :ref:`datapacks_set_t<doxid-class_engine_client_interface_1a37935f71194e675f096932e0e0afc4b5>` :ref:`getDataPacksFromEngine<doxid-class_engine_client_interface_1aa6a8f352acfa20e6cc3d34e3c69a7468>`(const :ref:`datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`& datapackIdentifiers) = 0;

.. _details-class_engine_client:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Base class for all Engines.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- ENGINE

		- Final derived engine class

Construction
------------

.. index:: pair: function; EngineClient
.. _doxid-class_engine_client_1a30252c2688ef6eadf51fdf08669a749e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	EngineClient(
		:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
		:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`&& launcher
	)

Constructor.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineConfig

		- Engine Configuration

	*
		- launcher

		- Process Forker

Methods
-------

.. index:: pair: function; engineName
.. _doxid-class_engine_client_1a04f80dd3fbb46b0056d825addb2231d7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::string engineName() const

Get Engine Name.



.. rubric:: Returns:

Returns engine name

.. index:: pair: function; getEngineTimestep
.. _doxid-class_engine_client_1a1089097a855a6ab0b4b27605529e1231:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` getEngineTimestep() const

Get engine timestep.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- Throws

		- on error

.. index:: pair: function; engineConfig
.. _doxid-class_engine_client_1a5495185601ad8529d112df7591e18b69:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig() const

Get Engine Configuration.

.. index:: pair: function; engineConfig
.. _doxid-class_engine_client_1a645831aa5aae4233085c3739c0669992:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig()

Get Engine Configuration.

.. index:: pair: function; engineSchema
.. _doxid-class_engine_client_1a356309d9f16c2d0c38002f7978400370:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::string engineSchema() const

Get json schema for this engine type.

.. index:: pair: function; getEngineTime
.. _doxid-class_engine_client_1a9b05083f880d664d4a4eaaba5e461584:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` getEngineTime() const

Returns current engine (simulation) time.

The time is updated by :ref:`EngineClient::runLoopStepAsyncGet() <doxid-class_engine_client_1ae94c9afd2b99f20dff28c3138e3eb5b1>` method.



.. rubric:: Returns:

Current engine (simulation) time

.. index:: pair: function; runLoopStepAsync
.. _doxid-class_engine_client_1a4b95a41aa73bbc8367d7acf0f47c2756:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void runLoopStepAsync(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep)

Concrete implementation of :ref:`EngineClientInterface::runLoopStepAsync() <doxid-class_engine_client_interface_1aabe6d06f4b2272422b782aef2ba7df4b>`

The function starts :ref:`EngineClient::runLoopStepCallback() <doxid-class_engine_client_1ac0325b83cbae4d2eb7c2acea2206afd7>` asynchronously using std::async. The callback function should be provided by concrete engine implementation. The result of the callback is going to be retrieved using an std::future object in :ref:`EngineClient::runLoopStepAsyncGet() <doxid-class_engine_client_1ae94c9afd2b99f20dff28c3138e3eb5b1>`.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timeStep

		- Requested duration of the simulation loop step.

	*
		- :ref:`NRPException <doxid-class_n_r_p_exception>`

		- If the future object is still valid (:ref:`EngineClient::runLoopStepAsyncGet() <doxid-class_engine_client_1ae94c9afd2b99f20dff28c3138e3eb5b1>` was not called)

.. index:: pair: function; runLoopStepAsyncGet
.. _doxid-class_engine_client_1ae94c9afd2b99f20dff28c3138e3eb5b1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void runLoopStepAsyncGet(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeOut)

Concrete implementation of :ref:`EngineClientInterface::runLoopStepAsyncGet() <doxid-class_engine_client_interface_1af7df79c71a0b87597b1638b07384c4e9>`

The function should be called after :ref:`EngineClient::runLoopStepAsync() <doxid-class_engine_client_1a4b95a41aa73bbc8367d7acf0f47c2756>`. It will wait for the worker thread to finish and retrieve the results from the future object. The value returned by the future should be the simulation (engine) time after running the loop step. It will be saved in the engine object, and can be accessed with :ref:`EngineClient::getEngineTime() <doxid-class_engine_client_1a9b05083f880d664d4a4eaaba5e461584>`.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timeOut

		- Timeout of the loop step. If it's less or equal to 0, the function will wait indefinitely.

	*
		- :ref:`NRPException <doxid-class_n_r_p_exception>`

		- On timeout

