.. index:: pair: class; NRPCommunicationController
.. _doxid-class_n_r_p_communication_controller:

class NRPCommunicationController
================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Manages communication with the NRP. Uses a REST server to send/receive data. Singleton class. :ref:`More...<details-class_n_r_p_communication_controller>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nrp_communication_controller.h>
	
	class NRPCommunicationController: public :ref:`EngineGrpcServer<doxid-class_engine_grpc_server>` {
	public:
		// construction
	
		:ref:`NRPCommunicationController<doxid-class_n_r_p_communication_controller_1a04b6b5d1dc709d4cc9b7ce0c1ded1907>`(const NRPCommunicationController& other);
		:ref:`NRPCommunicationController<doxid-class_n_r_p_communication_controller_1a2a768ff4414bb7dc56d97a3c859d5ee4>`(NRPCommunicationController&& other);

		// methods
	
		NRPCommunicationController& :ref:`operator =<doxid-class_n_r_p_communication_controller_1a069bf71a1176a91de5160422142c3699>` (const NRPCommunicationController& other);
		NRPCommunicationController&& :ref:`operator =<doxid-class_n_r_p_communication_controller_1a0048ff9de9f249943c981fd871a36f48>` (NRPCommunicationController&& other);
		void :ref:`registerStepController<doxid-class_n_r_p_communication_controller_1a8f9ad3d2be51cdd0f67c13f4892bb219>`(:ref:`GazeboStepController<doxid-class_gazebo_step_controller>`* stepController);
		void :ref:`registerSensorPlugin<doxid-class_n_r_p_communication_controller_1a89cd0926c02e8468bf2b755f9a64e4dd>`(gazebo::SensorPlugin* sensorPlugin);
		void :ref:`registerModelPlugin<doxid-class_n_r_p_communication_controller_1af44f76f87f77aa7cc561c373648d66e4>`(gazebo::ModelPlugin* modelPlugin);
		static NRPCommunicationController& :ref:`getInstance<doxid-class_n_r_p_communication_controller_1af9c8dd8896c0ed844eb3d1414792e31f>`();
		static NRPCommunicationController& :ref:`resetInstance<doxid-class_n_r_p_communication_controller_1a1ee9ec94baf2c28b9ccf438b6a8e4d73>`(const std::string& serverURL);
	
		static NRPCommunicationController& :ref:`resetInstance<doxid-class_n_r_p_communication_controller_1ab1e03835627880215cacf40536c73cd2>`(
			const std::string& serverURL,
			const std::string& engineName,
			const std::string& registrationURL
		);
	
		template <class T>
		static std::string :ref:`createDataPackName<doxid-class_n_r_p_communication_controller_1a55db766e3ad53f28b767875f0188ce5c>`(
			const gazebo::PluginT<T>& plugin,
			const std::string& objectName
		);
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// typedefs
	
		typedef std::timed_mutex :ref:`mutex_t<doxid-class_engine_grpc_server_1ab40542dacc855a02ffcc1ef23157ec90>`;
		typedef std::unique_lock<:ref:`EngineGrpcServer::mutex_t<doxid-class_engine_grpc_server_1ab40542dacc855a02ffcc1ef23157ec90>`> :ref:`lock_t<doxid-class_engine_grpc_server_1a07a6378e03bd4eacbb3f5255c225744e>`;

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
	
		void :ref:`registerDataPackNoLock<doxid-class_engine_grpc_server_1ae1966279b174aad6b985f25890ed5f1e>`(
			const std::string& datapackName,
			:ref:`ProtoDataPackController<doxid-engine__grpc__server_8h_1a8b6f823dadc78cb7cb8e59f426810363>`* interface
		);
	
		unsigned :ref:`getNumRegisteredDataPacks<doxid-class_engine_grpc_server_1a0fe86763e3a6e67c64829a207f68f7c4>`();

.. _details-class_n_r_p_communication_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Manages communication with the NRP. Uses a REST server to send/receive data. Singleton class.

Construction
------------

.. index:: pair: function; NRPCommunicationController
.. _doxid-class_n_r_p_communication_controller_1a04b6b5d1dc709d4cc9b7ce0c1ded1907:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	NRPCommunicationController(const NRPCommunicationController& other)

Delete for singleton.

.. index:: pair: function; NRPCommunicationController
.. _doxid-class_n_r_p_communication_controller_1a2a768ff4414bb7dc56d97a3c859d5ee4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	NRPCommunicationController(NRPCommunicationController&& other)

Delete for singleton.

Methods
-------

.. index:: pair: function; operator=
.. _doxid-class_n_r_p_communication_controller_1a069bf71a1176a91de5160422142c3699:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	NRPCommunicationController& operator = (const NRPCommunicationController& other)

Delete for singleton.

.. index:: pair: function; operator=
.. _doxid-class_n_r_p_communication_controller_1a0048ff9de9f249943c981fd871a36f48:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	NRPCommunicationController&& operator = (NRPCommunicationController&& other)

Delete for singleton.

.. index:: pair: function; registerStepController
.. _doxid-class_n_r_p_communication_controller_1a8f9ad3d2be51cdd0f67c13f4892bb219:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerStepController(:ref:`GazeboStepController<doxid-class_gazebo_step_controller>`* stepController)

Register a step controller.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- stepController

		- Pointer to step controller

.. index:: pair: function; registerSensorPlugin
.. _doxid-class_n_r_p_communication_controller_1a89cd0926c02e8468bf2b755f9a64e4dd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerSensorPlugin(gazebo::SensorPlugin* sensorPlugin)

Register a sensor plugin.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- sensorPlugin

		- Pointer to sensor plugin

.. index:: pair: function; registerModelPlugin
.. _doxid-class_n_r_p_communication_controller_1af44f76f87f77aa7cc561c373648d66e4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerModelPlugin(gazebo::ModelPlugin* modelPlugin)

Register a model plugin.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- sensorPlugin

		- Pointer to model plugin

.. index:: pair: function; getInstance
.. _doxid-class_n_r_p_communication_controller_1af9c8dd8896c0ed844eb3d1414792e31f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static NRPCommunicationController& getInstance()

Get singleton instance.



.. rubric:: Returns:

Gets instance of :ref:`NRPCommunicationController <doxid-class_n_r_p_communication_controller>`

.. index:: pair: function; resetInstance
.. _doxid-class_n_r_p_communication_controller_1a1ee9ec94baf2c28b9ccf438b6a8e4d73:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static NRPCommunicationController& resetInstance(const std::string& serverURL)

Reset server with the given server URL.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- serverURL

		- URL used by server



.. rubric:: Returns:

Returns reference to server instance

.. index:: pair: function; resetInstance
.. _doxid-class_n_r_p_communication_controller_1ab1e03835627880215cacf40536c73cd2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static NRPCommunicationController& resetInstance(
		const std::string& serverURL,
		const std::string& engineName,
		const std::string& registrationURL
	)

Reset server with the given server URL.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- serverURL

		- URL used by server

	*
		- engineName

		- Name of this engine

	*
		- registrationURL

		- URL used to register this engine server's URL



.. rubric:: Returns:

Returns reference to server instance

.. index:: pair: function; createDataPackName
.. _doxid-class_n_r_p_communication_controller_1a55db766e3ad53f28b767875f0188ce5c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class T>
	static std::string createDataPackName(
		const gazebo::PluginT<T>& plugin,
		const std::string& objectName
	)

Create datapack name from the given plugin and sensor/joint/link.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- T

		- Plugin Type

	*
		- plugin

		- Controller Plugin

	*
		- objectName

		- Name of the controlled object (sensor, joint, link, ...)



.. rubric:: Returns:

Returns datapack name

