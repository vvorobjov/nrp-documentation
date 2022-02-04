.. index:: pair: class; EngineClient::EngineLauncher
.. _doxid-class_engine_client_1_1_engine_launcher:

template class EngineClient::EngineLauncher
===========================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Class for launching engine. :ref:`More...<details-class_engine_client_1_1_engine_launcher>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_client_interface.h>
	
	template <const char* ENGINE_TYPE>
	class EngineLauncher: public :ref:`EngineLauncherInterface<doxid-class_engine_launcher_interface>` {
	public:
		// construction
	
		:target:`EngineLauncher<doxid-class_engine_client_1_1_engine_launcher_1ad7de6a49330c9e7d4b3d8eaa0f425580>`();
		:target:`EngineLauncher<doxid-class_engine_client_1_1_engine_launcher_1ac5ab207585ce91a5c26ffc998f8ec2fe>`(const :ref:`engine_type_t<doxid-class_engine_launcher_interface_1af687314e7b9a2f37664b78b6f673e13f>`& engineType);

		// methods
	
		virtual :ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>` :ref:`launchEngine<doxid-class_engine_client_1_1_engine_launcher_1a3c2050ae4d2d3b825adf6678e51851b2>`(
			:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
			:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`&& launcher
		);
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
		typedef decltype(:ref:`DataPackIdentifier::Type<doxid-struct_data_pack_identifier_1a39e482341dca27cee33a6d7d78f99605>`) :ref:`engine_type_t<doxid-class_engine_launcher_interface_1af687314e7b9a2f37664b78b6f673e13f>`;

		// methods
	
		const :ref:`engine_type_t<doxid-class_engine_launcher_interface_1af687314e7b9a2f37664b78b6f673e13f>`& :ref:`engineType<doxid-class_engine_launcher_interface_1a866599c5f65eaff264b3c6f89ae75a48>`() const;
	
		virtual :ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>` :ref:`launchEngine<doxid-class_engine_launcher_interface_1adcc3aed895245d6fed9c59970b1d5b0b>`(
			:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
			:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`&& launcher
		) = 0;

.. _details-class_engine_client_1_1_engine_launcher:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Class for launching engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- ENGINE_TYPE

		- Default engine type

Methods
-------

.. index:: pair: function; launchEngine
.. _doxid-class_engine_client_1_1_engine_launcher_1a3c2050ae4d2d3b825adf6678e51851b2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>` launchEngine(
		:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
		:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`&& launcher
	)

Launches an engine. Configures config and forks a new child process for the engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineConfig

		- Engine Configuration

	*
		- launcher

		- Process Forker



.. rubric:: Returns:

Returns pointer to :ref:`EngineClientInterface <doxid-class_engine_client_interface>`

