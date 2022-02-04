.. index:: pair: class; EngineLauncherManager
.. _doxid-class_engine_launcher_manager:

class EngineLauncherManager
===========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Engine Launcher Manager. Used to register, and find engine launchers. :ref:`More...<details-class_engine_launcher_manager>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_launcher_manager.h>
	
	class EngineLauncherManager: public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// methods
	
		void :ref:`registerLauncher<doxid-class_engine_launcher_manager_1a5cc94530bee8d9d894521d117eb30ecc>`(const :ref:`EngineLauncherInterfaceSharedPtr<doxid-engine__client__interface_8h_1ae3f4035d1af7dc2ad1f8e415922cecbc>`& launcher);
		:ref:`EngineLauncherInterfaceSharedPtr<doxid-engine__client__interface_8h_1ae3f4035d1af7dc2ad1f8e415922cecbc>` :ref:`findLauncher<doxid-class_engine_launcher_manager_1a8b6fe3dc47067a9a414fc746bd8cb553>`(const :ref:`EngineLauncherInterface::engine_type_t<doxid-class_engine_launcher_interface_1af687314e7b9a2f37664b78b6f673e13f>`& name) const;
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

.. _details-class_engine_launcher_manager:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Engine Launcher Manager. Used to register, and find engine launchers.

Methods
-------

.. index:: pair: function; registerLauncher
.. _doxid-class_engine_launcher_manager_1a5cc94530bee8d9d894521d117eb30ecc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerLauncher(const :ref:`EngineLauncherInterfaceSharedPtr<doxid-engine__client__interface_8h_1ae3f4035d1af7dc2ad1f8e415922cecbc>`& launcher)

Register launcher.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- launcher

		- Launcher to register

.. index:: pair: function; findLauncher
.. _doxid-class_engine_launcher_manager_1a8b6fe3dc47067a9a414fc746bd8cb553:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`EngineLauncherInterfaceSharedPtr<doxid-engine__client__interface_8h_1ae3f4035d1af7dc2ad1f8e415922cecbc>` findLauncher(const :ref:`EngineLauncherInterface::engine_type_t<doxid-class_engine_launcher_interface_1af687314e7b9a2f37664b78b6f673e13f>`& name) const

Finds a Launcher via the given name.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- name

		- Name of Launcher



.. rubric:: Returns:

Returns pointer to the Launcher if available, nullptr otherwise

