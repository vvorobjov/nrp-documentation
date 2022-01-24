.. index:: pair: class; GazeboEngineGrpcNRPClient
.. _doxid-class_gazebo_engine_grpc_n_r_p_client:

class GazeboEngineGrpcNRPClient
===============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

NRP - Gazebo Communicator on the NRP side. Converts :ref:`DataPackInterface <doxid-class_data_pack_interface>` classes from/to JSON objects. :ref:`More...<details-class_gazebo_engine_grpc_n_r_p_client>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <gazebo_engine_grpc_nrp_client.h>
	
	class GazeboEngineGrpcNRPClient: public :ref:`EngineGrpcClient<doxid-class_engine_grpc_client>` {
	public:
		// construction
	
		:target:`GazeboEngineGrpcNRPClient<doxid-class_gazebo_engine_grpc_n_r_p_client_1ae87a24d3d2b81f7587fdcfcc0c920228>`(
			:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& config,
			:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`&& launcher
		);

		// methods
	
		virtual void :ref:`initialize<doxid-class_gazebo_engine_grpc_n_r_p_client_1a3314685b24d101b0142e43958dc6be77>`();
		virtual void :ref:`reset<doxid-class_gazebo_engine_grpc_n_r_p_client_1afdc7d3b1a233528efdbe24b26498c245>`();
		virtual void :ref:`shutdown<doxid-class_gazebo_engine_grpc_n_r_p_client_1a4eb33c1629cdb965a97f2b48e4c281de>`();
		virtual const std::vector<std::string> :ref:`engineProcStartParams<doxid-class_gazebo_engine_grpc_n_r_p_client_1a54671280b7e86dbaed213058d27bfbce>`() const;
		virtual const std::vector<std::string> :ref:`engineProcEnvParams<doxid-class_gazebo_engine_grpc_n_r_p_client_1a28422b2c8ad5d2ff5a81afebe6119c41>`() const;
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
		grpc_connectivity_state :ref:`getChannelStatus<doxid-class_engine_grpc_client_1af84dbb285cca036df686daa4eb5d86b2>`();
		grpc_connectivity_state :ref:`connect<doxid-class_engine_grpc_client_1a2ab65b40e05325f6772ec9a9d09d5b45>`();
		void :ref:`sendInitCommand<doxid-class_engine_grpc_client_1aafaa7e67bd53026dcebea328425575fe>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data);
		void :ref:`sendResetCommand<doxid-class_engine_grpc_client_1a820e44c9f6f5bd84680e5301f570a18e>`();
		void :ref:`sendShutdownCommand<doxid-class_engine_grpc_client_1a3e4b74abc5e29130769669d5009ebf40>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data);
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`runLoopStepCallback<doxid-class_engine_grpc_client_1a05682b9efbc1d7c657ae5eac4839a900>`(const :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep);
		virtual void :ref:`sendDataPacksToEngine<doxid-class_engine_grpc_client_1ae64e2a30981d0abbd3ef27011b593c4e>`(const typename :ref:`EngineClientInterface::datapacks_ptr_t<doxid-class_engine_client_interface_1a54f3c8965ef0b6c1ef52dbc0d9d918e8>`& datapacksArray);
		virtual const std::vector<std::string> :ref:`engineProcStartParams<doxid-class_engine_grpc_client_1afa65a143d777a1d5372ee29f8d2d785c>`() const;
		virtual const std::vector<std::string> :ref:`engineProcEnvParams<doxid-class_engine_grpc_client_1ad8fa30f99d36f6573bc902a13e14f7d9>`() const;
	
		template <class MSG_TYPE, class ... REMAINING_MSG_TYPES>
		:ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>` :ref:`getDataPackInterfaceFromProto<doxid-class_engine_grpc_client_1a7355c9e27dbe06328df7b6e47448d2f8>`(Engine::DataPackMessage& datapackData) const;
	
		template <class MSG_TYPE, class ... REMAINING_MSG_TYPES>
		void :ref:`setProtoFromDataPackInterface<doxid-class_engine_grpc_client_1a707ac452bcecad8dd161b7a338eafe33>`(
			Engine::DataPackMessage* datapackData,
			:ref:`DataPackInterface<doxid-class_data_pack_interface>`* datapack
		);
	
		virtual :ref:`EngineClientInterface::datapacks_set_t<doxid-class_engine_client_interface_1a37935f71194e675f096932e0e0afc4b5>` :ref:`getDataPacksFromEngine<doxid-class_engine_grpc_client_1a69888082185369c8c6f5d8f8a0e8e646>`(const typename :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`& datapackIdentifiers);

.. _details-class_gazebo_engine_grpc_n_r_p_client:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

NRP - Gazebo Communicator on the NRP side. Converts :ref:`DataPackInterface <doxid-class_data_pack_interface>` classes from/to JSON objects.

Methods
-------

.. index:: pair: function; initialize
.. _doxid-class_gazebo_engine_grpc_n_r_p_client_1a3314685b24d101b0142e43958dc6be77:

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
.. _doxid-class_gazebo_engine_grpc_n_r_p_client_1afdc7d3b1a233528efdbe24b26498c245:

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
.. _doxid-class_gazebo_engine_grpc_n_r_p_client_1a4eb33c1629cdb965a97f2b48e4c281de:

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

.. index:: pair: function; engineProcStartParams
.. _doxid-class_gazebo_engine_grpc_n_r_p_client_1a54671280b7e86dbaed213058d27bfbce:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::vector<std::string> engineProcStartParams() const

Get all Engine Process Startup parameters.

.. index:: pair: function; engineProcEnvParams
.. _doxid-class_gazebo_engine_grpc_n_r_p_client_1a28422b2c8ad5d2ff5a81afebe6119c41:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::vector<std::string> engineProcEnvParams() const

Get all Engine Process Environment variables.

