.. index:: pair: class; ProcessLauncherManager
.. _doxid-class_process_launcher_manager:

template class ProcessLauncherManager
=====================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Class to manage process managers. :ref:`More...<details-class_process_launcher_manager>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <process_launcher_manager.h>
	
	template <class ... PROCESS_LAUNCHERS>
	class ProcessLauncherManager: public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// construction
	
		:ref:`ProcessLauncherManager<doxid-class_process_launcher_manager_1aee3f28ad899d62da10de3944aa356015>`();
		:target:`ProcessLauncherManager<doxid-class_process_launcher_manager_1ae49f2df085e3e6abd5826dce1a0483a0>`(const ProcessLauncherManager&);
		:target:`ProcessLauncherManager<doxid-class_process_launcher_manager_1aa8ca644396c1f4d636102f5ee8755d0f>`(ProcessLauncherManager&&);

		// methods
	
		ProcessLauncherManager& :target:`operator =<doxid-class_process_launcher_manager_1a4416b0bfe415e7fc343993ecb44f53c1>` (ProcessLauncherManager&&);
		ProcessLauncherManager& :target:`operator =<doxid-class_process_launcher_manager_1a496106e24d5ab0275364c16b32585fce>` (const ProcessLauncherManager&);
		:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>` :ref:`createProcessLauncher<doxid-class_process_launcher_manager_1a18c3d4adb604d37cc42ab630432cf440>`(const std::string& launcherType) const;
		void :target:`registerProcessLauncher<doxid-class_process_launcher_manager_1ac4335b7a9b18acd1b669916c87c28040>`(:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`&& launcher);
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

.. _details-class_process_launcher_manager:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Class to manage process managers.

Construction
------------

.. index:: pair: function; ProcessLauncherManager
.. _doxid-class_process_launcher_manager_1aee3f28ad899d62da10de3944aa356015:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ProcessLauncherManager()

Constructor. Registers all Process Launchers for further use.

Methods
-------

.. index:: pair: function; createProcessLauncher
.. _doxid-class_process_launcher_manager_1a18c3d4adb604d37cc42ab630432cf440:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>` createProcessLauncher(const std::string& launcherType) const

Create a new process launcher.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- launcherType

		- Name of launcher



.. rubric:: Returns:

Returns ptr to launcher

