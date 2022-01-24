.. index:: pair: class; EngineClientInterface
.. _doxid-class_engine_client_interface:

class EngineClientInterface
===========================

.. toctree::
	:hidden:

	struct_EngineClientInterface_CompareDevInt.rst

Overview
~~~~~~~~

Interface to engines. :ref:`More...<details-class_engine_client_interface>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_client_interface.h>
	
	class EngineClientInterface: public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// typedefs
	
		typedef std::set<:ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`> :target:`datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`;
		typedef std::vector<:ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>`> :target:`datapacks_t<doxid-class_engine_client_interface_1a2562bba1cd7ecc32dc6bfd54691c669c>`;
		typedef std::set<:ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>`, CompareDevInt> :target:`datapacks_set_t<doxid-class_engine_client_interface_1a37935f71194e675f096932e0e0afc4b5>`;
		typedef std::vector<:ref:`DataPackInterface<doxid-class_data_pack_interface>`*> :target:`datapacks_ptr_t<doxid-class_engine_client_interface_1a54f3c8965ef0b6c1ef52dbc0d9d918e8>`;

		// structs
	
		struct :ref:`CompareDevInt<doxid-struct_engine_client_interface_1_1_compare_dev_int>`;

		// construction
	
		:target:`EngineClientInterface<doxid-class_engine_client_interface_1a7f2ba41d3d497ecc6c72bf5dcad8c296>`(:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`&& launcher);

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
	};

	// direct descendants

	template <class ENGINE, const char* SCHEMA>
	class :ref:`EngineClient<doxid-class_engine_client>`;

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

.. _details-class_engine_client_interface:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Interface to engines.

Methods
-------

.. index:: pair: function; engineName
.. _doxid-class_engine_client_interface_1abb12cc28100d5fb2baddbad92d0293f6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::string engineName() const = 0

Get Engine Name.



.. rubric:: Returns:

Returns engine name

.. index:: pair: function; engineConfig
.. _doxid-class_engine_client_interface_1a176fcd0c5de87a149574999c53461ac2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig() const = 0

Get engine config data.

.. index:: pair: function; engineConfig
.. _doxid-class_engine_client_interface_1a450994aebcb9dda0252d08d14fbde89a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig() = 0

Get engine config data.

.. index:: pair: function; engineProcStartParams
.. _doxid-class_engine_client_interface_1a6747137f2b551040adca807e6df38a59:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::vector<std::string> engineProcStartParams() const = 0

Get all Engine Process Startup parameters.

.. index:: pair: function; engineProcEnvParams
.. _doxid-class_engine_client_interface_1ac3bf04a627785a082fbe83a5aa004227:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::vector<std::string> engineProcEnvParams() const = 0

Get all Engine Process Environment variables.

.. index:: pair: function; launchEngine
.. _doxid-class_engine_client_interface_1a42dd02dc80abcc1f48dccf9da0ce2f0c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual pid_t launchEngine()

Launch the engine.



.. rubric:: Returns:

Returns engine process ID on success, throws on failure

.. index:: pair: function; initialize
.. _doxid-class_engine_client_interface_1ac600fd036f83cc1aa0ae8fa79b176b44:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void initialize() = 0

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
.. _doxid-class_engine_client_interface_1a65d86ba09fd72e32b0399ca290c38632:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void reset() = 0

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
.. _doxid-class_engine_client_interface_1a0a15d1d539bc8134f8ed62668c284883:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void shutdown() = 0

Shutdown engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- Throws

		- on error



.. rubric:: Returns:

Return SUCCESS if no error was encountered

.. index:: pair: function; getEngineTimestep
.. _doxid-class_engine_client_interface_1a292b03422ca28976dc3d70f90e11d4e4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` getEngineTimestep() const = 0

Get engine timestep.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- Throws

		- on error

.. index:: pair: function; getEngineTime
.. _doxid-class_engine_client_interface_1ab3080a4a253a676d9be0205a3abe9224:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` getEngineTime() const = 0

Get current engine time.



.. rubric:: Returns:

Returns engine time

.. index:: pair: function; engineSchema
.. _doxid-class_engine_client_interface_1a93fc42d5d932c15f856e3db395dac5d5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::string engineSchema() const = 0

Get json schema for this specific engine type.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- Throws

		- on error



.. rubric:: Returns:

Returns URI of engine schema

.. index:: pair: function; runLoopStepAsync
.. _doxid-class_engine_client_interface_1aabe6d06f4b2272422b782aef2ba7df4b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void runLoopStepAsync(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep) = 0

Starts a single loop step in a separate thread.

The function should be called in tandem with :ref:`EngineClientInterface::runLoopStepAsyncGet() <doxid-class_engine_client_interface_1af7df79c71a0b87597b1638b07384c4e9>`, which will join the worker thread and retrieve the results of the loop step.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timeStep

		- Requested duration of the simulation loop step.

.. index:: pair: function; runLoopStepAsyncGet
.. _doxid-class_engine_client_interface_1af7df79c71a0b87597b1638b07384c4e9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void runLoopStepAsyncGet(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeOut) = 0

Waits and gets the results of the loop step started by :ref:`EngineClientInterface::runLoopStepAsync() <doxid-class_engine_client_interface_1aabe6d06f4b2272422b782aef2ba7df4b>`

The function should be called after calling :ref:`EngineClientInterface::runLoopStepAsync() <doxid-class_engine_client_interface_1aabe6d06f4b2272422b782aef2ba7df4b>`. It should join the worker thread and retrieve the results of the loop step.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timeOut

		- Timeout of the loop step. If it's less or equal to 0, the function will wait indefinitely.

.. index:: pair: function; updateDataPacksFromEngine
.. _doxid-class_engine_client_interface_1afeb9a1af5db6f59d09c93f97afc11895:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const :ref:`datapacks_t<doxid-class_engine_client_interface_1a2562bba1cd7ecc32dc6bfd54691c669c>`& updateDataPacksFromEngine(const :ref:`datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`& datapackIdentifiers)

Gets requested datapacks from engine and updates _datapackCache with the results Uses getDataPacksFromEngine override for the actual communication.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackNames

		- All requested names. NOTE: can also include IDs of other engines. A check must be added that only the corresponding IDs are retrieved



.. rubric:: Returns:

Returns all datapacks returned by the engine

.. index:: pair: function; getCachedDataPacks
.. _doxid-class_engine_client_interface_1a9b0a665916043faaf250df4b06fdece4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	constexpr const :ref:`datapacks_t<doxid-class_engine_client_interface_1a2562bba1cd7ecc32dc6bfd54691c669c>`& getCachedDataPacks() const

get cached engine datapacks

.. index:: pair: function; sendDataPacksToEngine
.. _doxid-class_engine_client_interface_1a72110362e024889f75d55c3a80696da3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void sendDataPacksToEngine(const :ref:`datapacks_ptr_t<doxid-class_engine_client_interface_1a54f3c8965ef0b6c1ef52dbc0d9d918e8>`& datapacksArray) = 0

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

.. index:: pair: function; updateCachedDataPacks
.. _doxid-class_engine_client_interface_1a5b95f18bb550700360ca408bc0e51583:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void updateCachedDataPacks(:ref:`datapacks_set_t<doxid-class_engine_client_interface_1a37935f71194e675f096932e0e0afc4b5>`&& devs)

Update _datapackCache from datapacks.

If the datapack with a particular name is already in the cache, the function will replace it. If the datapack isn't in the cache, the function will insert it.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- devs

		- DataPacks to insert

.. index:: pair: function; getDataPacksFromEngine
.. _doxid-class_engine_client_interface_1aa6a8f352acfa20e6cc3d34e3c69a7468:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`datapacks_set_t<doxid-class_engine_client_interface_1a37935f71194e675f096932e0e0afc4b5>` getDataPacksFromEngine(const :ref:`datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`& datapackIdentifiers) = 0

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

