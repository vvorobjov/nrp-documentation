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
	
		typedef std::timed_mutex :ref:`mutex_t<doxid-class_engine_grpc_server_1a8c31ad3bdbbbdd8b18a19c43a20ce1c1>`;
		typedef std::unique_lock<:ref:`EngineGrpcServer::mutex_t<doxid-class_engine_grpc_server_1a8c31ad3bdbbbdd8b18a19c43a20ce1c1>`> :ref:`lock_t<doxid-class_engine_grpc_server_1a649df914ed68119cfa914a9bf980dcf9>`;

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
	
		void :ref:`registerDataPackNoLock<doxid-class_engine_grpc_server_1a66dca0d25b7db065ea2a8fef951a19da>`(
			const std::string& datapackName,
			:ref:`ProtoDataPackController<doxid-engine__grpc__server_8h_1a8b6f823dadc78cb7cb8e59f426810363>`* interface
		);
	
		unsigned :ref:`getNumRegisteredDataPacks<doxid-class_engine_grpc_server_1aa1d06fc74495f845da2241ad42c2a534>`();

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

