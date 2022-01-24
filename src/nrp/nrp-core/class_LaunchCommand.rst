.. index:: pair: class; LaunchCommand
.. _doxid-class_launch_command:

template class LaunchCommand
============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Class for launch commands. Must be specialized further. :ref:`More...<details-class_launch_command>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <launch_command.h>
	
	template <const char* LAUNCH_COMMAND>
	class LaunchCommand: public :ref:`LaunchCommandInterface<doxid-class_launch_command_interface>` {
	public:
		// fields
	
		static constexpr auto :target:`LaunchType<doxid-class_launch_command_1a685e087bb1092d0af2ff194c82f8f258>` = :ref:`LAUNCH_COMMAND<doxid-basic__fork_8h_1aa53475dce0eeddd9471c0bf490572cdf>`;

		// methods
	
		virtual std::string_view :ref:`launchType<doxid-class_launch_command_1add06b48328f56f21f9e51760bc89331a>`() const;
	};

	// direct descendants

	class :ref:`BasicFork<doxid-class_basic_fork>`;

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

		// enums
	
		enum :ref:`ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>`;

		// methods
	
		virtual pid_t :ref:`launchEngineProcess<doxid-class_launch_command_interface_1a29b04be0f3b930a0121b532392561d21>`(
			const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
			const std::vector<std::string>& envParams,
			const std::vector<std::string>& startParams,
			bool appendParentEnv = true
		) = 0;
	
		virtual pid_t :ref:`stopEngineProcess<doxid-class_launch_command_interface_1a401550deee9cc8bedeff819716f8d4e3>`(unsigned int killWait) = 0;
		virtual :ref:`ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>` :ref:`getProcessStatus<doxid-class_launch_command_interface_1a67559c48de23dcd6f2b75f3098d555bc>`();
		virtual std::string_view :ref:`launchType<doxid-class_launch_command_interface_1af7464a527670dca6eee742f649d356b3>`() const = 0;

.. _details-class_launch_command:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Class for launch commands. Must be specialized further.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- LAUNCH_COMMAND

		- Name of launch command

Methods
-------

.. index:: pair: function; launchType
.. _doxid-class_launch_command_1add06b48328f56f21f9e51760bc89331a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual std::string_view launchType() const

Get launch command type.

