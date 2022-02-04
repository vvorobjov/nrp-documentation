.. index:: pair: class; NestEngineServerNRPClient
.. _doxid-class_nest_engine_server_n_r_p_client:

class NestEngineServerNRPClient
===============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

NRP - Nest Communicator on the NRP side. Converts :ref:`DataPackInterface <doxid-class_data_pack_interface>` classes from/to JSON objects. :ref:`More...<details-class_nest_engine_server_n_r_p_client>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nest_engine_server_nrp_client.h>
	
	class NestEngineServerNRPClient: public :ref:`EngineClient<doxid-class_engine_client>` {
	public:
		// typedefs
	
		typedef std::map<std::string, std::string> :target:`population_mapping_t<doxid-class_nest_engine_server_n_r_p_client_1a5869038444c6ebd917e2e43a834f618b>`;

		// construction
	
		:target:`NestEngineServerNRPClient<doxid-class_nest_engine_server_n_r_p_client_1a926516c5081af3015ef3f74796e6cc82>`(
			:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& config,
			:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`&& launcher
		);

		// methods
	
		virtual void :ref:`initialize<doxid-class_nest_engine_server_n_r_p_client_1ad4a5f32236ce7ebdf2d88b40c6b6a4e8>`();
		virtual void :ref:`reset<doxid-class_nest_engine_server_n_r_p_client_1a7b472b61b3f65eab6dcefcbe611b44d4>`();
		virtual void :ref:`shutdown<doxid-class_nest_engine_server_n_r_p_client_1adb96ff86f814df40b0aa42bccf2918de>`();
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`runLoopStepCallback<doxid-class_nest_engine_server_n_r_p_client_1aed809c2821cde420a0ee7532ca9db860>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep);
		virtual void :ref:`sendDataPacksToEngine<doxid-class_nest_engine_server_n_r_p_client_1a6d0d52e416f0b06496b4b5372a7f6152>`(const :ref:`datapacks_ptr_t<doxid-class_engine_client_interface_1a54f3c8965ef0b6c1ef52dbc0d9d918e8>`& datapacksArray);
		virtual const std::vector<std::string> :ref:`engineProcStartParams<doxid-class_nest_engine_server_n_r_p_client_1ad28fe64c421c23e46379bcad822f7d3b>`() const;
		virtual const std::vector<std::string> :ref:`engineProcEnvParams<doxid-class_nest_engine_server_n_r_p_client_1a7f3d71d25144a1a96ac2a05fd99ccb69>`() const;
		virtual :ref:`datapacks_set_t<doxid-class_engine_client_interface_1a37935f71194e675f096932e0e0afc4b5>` :ref:`getDataPacksFromEngine<doxid-class_nest_engine_server_n_r_p_client_1a95847acd1c828c3bb35fe2ca1d03459f>`(const :ref:`datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`& datapackIdentifiers);
	};

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
		typedef ENGINE :ref:`engine_t<doxid-class_engine_client_1ae642237dab2cde85069f302ed91a6f73>`;

		// structs
	
		struct :ref:`CompareDevInt<doxid-struct_engine_client_interface_1_1_compare_dev_int>`;

		// classes
	
		template <const char* ENGINE_TYPE>
		class :ref:`EngineLauncher<doxid-class_engine_client_1_1_engine_launcher>`;

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
		virtual const std::string :ref:`engineName<doxid-class_engine_client_1a04f80dd3fbb46b0056d825addb2231d7>`() const;
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`getEngineTimestep<doxid-class_engine_client_1a1089097a855a6ab0b4b27605529e1231>`() const;
		virtual const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& :ref:`engineConfig<doxid-class_engine_client_1a5495185601ad8529d112df7591e18b69>`() const;
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& :ref:`engineConfig<doxid-class_engine_client_1a645831aa5aae4233085c3739c0669992>`();
		virtual const std::string :ref:`engineSchema<doxid-class_engine_client_1a356309d9f16c2d0c38002f7978400370>`() const;
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`getEngineTime<doxid-class_engine_client_1a9b05083f880d664d4a4eaaba5e461584>`() const;
		virtual void :ref:`runLoopStepAsync<doxid-class_engine_client_1a4b95a41aa73bbc8367d7acf0f47c2756>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep);
		virtual void :ref:`runLoopStepAsyncGet<doxid-class_engine_client_1ae94c9afd2b99f20dff28c3138e3eb5b1>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeOut);

.. _details-class_nest_engine_server_n_r_p_client:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

NRP - Nest Communicator on the NRP side. Converts :ref:`DataPackInterface <doxid-class_data_pack_interface>` classes from/to JSON objects.

Methods
-------

.. index:: pair: function; initialize
.. _doxid-class_nest_engine_server_n_r_p_client_1ad4a5f32236ce7ebdf2d88b40c6b6a4e8:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void initialize()

Initialize engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- Throws

		- on error



.. rubric:: Returns:

Returns SUCCESS if no error was encountered

.. index:: pair: function; reset
.. _doxid-class_nest_engine_server_n_r_p_client_1a7b472b61b3f65eab6dcefcbe611b44d4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void reset()

Reset engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- Throws

		- on error



.. rubric:: Returns:

Returns SUCCESS if no error was encountered

.. index:: pair: function; shutdown
.. _doxid-class_nest_engine_server_n_r_p_client_1adb96ff86f814df40b0aa42bccf2918de:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void shutdown()

Shutdown engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- Throws

		- on error



.. rubric:: Returns:

Return SUCCESS if no error was encountered

.. index:: pair: function; runLoopStepCallback
.. _doxid-class_nest_engine_server_n_r_p_client_1aed809c2821cde420a0ee7532ca9db860:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runLoopStepCallback(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep)

Executes a single loop step.

This function is going to be called by runLoopStep using std::async. It will be executed by a worker thread, which allows for runLoopStepFunction from multiple engines to run simultaneously.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timeStep

		- A time step by which the simulation should be advanced



.. rubric:: Returns:

Engine time after loop step execution

.. index:: pair: function; sendDataPacksToEngine
.. _doxid-class_nest_engine_server_n_r_p_client_1a6d0d52e416f0b06496b4b5372a7f6152:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void sendDataPacksToEngine(const :ref:`datapacks_ptr_t<doxid-class_engine_client_interface_1a54f3c8965ef0b6c1ef52dbc0d9d918e8>`& datapacksArray)

Sends datapacks to engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapacksArray

		- Array of datapacks that will be send to the engine

	*
		- Throws

		- on error



.. rubric:: Returns:

Returns SUCCESS if all datapacks could be handles, ERROR otherwise

.. index:: pair: function; engineProcStartParams
.. _doxid-class_nest_engine_server_n_r_p_client_1ad28fe64c421c23e46379bcad822f7d3b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::vector<std::string> engineProcStartParams() const

Get all Engine Process Startup parameters.

.. index:: pair: function; engineProcEnvParams
.. _doxid-class_nest_engine_server_n_r_p_client_1a7f3d71d25144a1a96ac2a05fd99ccb69:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::vector<std::string> engineProcEnvParams() const

Get all Engine Process Environment variables.

.. index:: pair: function; getDataPacksFromEngine
.. _doxid-class_nest_engine_server_n_r_p_client_1a95847acd1c828c3bb35fe2ca1d03459f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`datapacks_set_t<doxid-class_engine_client_interface_1a37935f71194e675f096932e0e0afc4b5>` getDataPacksFromEngine(const :ref:`datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`& datapackIdentifiers)

Gets requested datapacks from engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackNames

		- All requested datapack ids

	*
		- Throws

		- on error



.. rubric:: Returns:

Returns all requested datapacks

