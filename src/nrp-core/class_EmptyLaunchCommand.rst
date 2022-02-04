.. index:: pair: class; EmptyLaunchCommand
.. _doxid-class_empty_launch_command:

class EmptyLaunchCommand
========================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <empty_launch_command.h>
	
	class EmptyLaunchCommand: public :ref:`LaunchCommand<doxid-class_launch_command>` {
	public:
		// methods
	
		virtual pid_t :ref:`launchEngineProcess<doxid-class_empty_launch_command_1a33f1ecf5e7c83f51ea587013c282271a>`(
			const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`&,
			const std::vector<std::string>&,
			const std::vector<std::string>&,
			bool
		);
	
		virtual pid_t :ref:`stopEngineProcess<doxid-class_empty_launch_command_1a7f9bb45eea2242da4ce185fe51acdb08>`(unsigned int);
		virtual :ref:`ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>` :ref:`getProcessStatus<doxid-class_empty_launch_command_1a241b3b8208310597f7316eba8133cd34>`();
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

		// enums
	
		enum :ref:`ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>`;

		// fields
	
		static constexpr auto :ref:`LaunchType<doxid-class_launch_command_1a685e087bb1092d0af2ff194c82f8f258>` = :ref:`LAUNCH_COMMAND<doxid-basic__fork_8h_1aa53475dce0eeddd9471c0bf490572cdf>`;

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
		virtual std::string_view :ref:`launchType<doxid-class_launch_command_1add06b48328f56f21f9e51760bc89331a>`() const;

.. _details-class_empty_launch_command:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; launchEngineProcess
.. _doxid-class_empty_launch_command_1a33f1ecf5e7c83f51ea587013c282271a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual pid_t launchEngineProcess(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`&,
		const std::vector<std::string>&,
		const std::vector<std::string>&,
		bool
	)

launchEngineProcess always returns -1



.. rubric:: Returns:

-1

.. index:: pair: function; stopEngineProcess
.. _doxid-class_empty_launch_command_1a7f9bb45eea2242da4ce185fe51acdb08:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual pid_t stopEngineProcess(unsigned int)

stopEngineProcess always returns 0



.. rubric:: Returns:

0

.. index:: pair: function; getProcessStatus
.. _doxid-class_empty_launch_command_1a241b3b8208310597f7316eba8133cd34:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>` getProcessStatus()

getProcessStatus always returns ENGINE_RUNNING_STATUS::UNKNOWN



.. rubric:: Returns:

ENGINE_RUNNING_STATUS::UNKNOWN

