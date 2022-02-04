.. index:: pair: class; ProcessLauncherInterface
.. _doxid-class_process_launcher_interface:

class ProcessLauncherInterface
==============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Functions for all process launchers. :ref:`More...<details-class_process_launcher_interface>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <process_launcher.h>
	
	class ProcessLauncherInterface: public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// typedefs
	
		typedef :ref:`LaunchCommandInterface::ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>` :target:`ENGINE_RUNNING_STATUS<doxid-class_process_launcher_interface_1ad958854e32d7a961c7bb07bd23e088b5>`;

		// fields
	
		static constexpr auto :target:`UNKNOWN<doxid-class_process_launcher_interface_1a8ce968ad8b7e9a8fd23b20032133cd11>` = LaunchCommandInterface::ENGINE_RUNNING_STATUS::UNKNOWN;
		static constexpr auto :target:`RUNNING<doxid-class_process_launcher_interface_1a29b24978e039d71a362509c857124e1a>` = LaunchCommandInterface::ENGINE_RUNNING_STATUS::RUNNING;
		static constexpr auto :target:`STOPPED<doxid-class_process_launcher_interface_1a57eb1ac4597c3c830bf9c876447e819e>` = LaunchCommandInterface::ENGINE_RUNNING_STATUS::STOPPED;

		// methods
	
		virtual std::string :ref:`launcherName<doxid-class_process_launcher_interface_1a525cd3685e3eff9a4d1ef17bf4ba06ab>`() const = 0;
		virtual :ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>` :ref:`createLauncher<doxid-class_process_launcher_interface_1a677aa140f965174b25ba89afa7556983>`() = 0;
	
		virtual pid_t :ref:`launchEngineProcess<doxid-class_process_launcher_interface_1ad39354640a56be631da38f6db4f301b2>`(
			const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
			const std::vector<std::string>& envParams,
			const std::vector<std::string>& startParams,
			bool appendParentEnv = true
		) = 0;
	
		virtual pid_t :ref:`stopEngineProcess<doxid-class_process_launcher_interface_1a8c350cf5794790b991d3a42c943e9677>`(unsigned int killWait) = 0;
		virtual :ref:`ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>` :ref:`getProcessStatus<doxid-class_process_launcher_interface_1afecd8b121e9b5df59a47584a93809b94>`();
		:ref:`LaunchCommandInterface<doxid-class_launch_command_interface>`* :ref:`launchCommand<doxid-class_process_launcher_interface_1adde0e1c47dd675a2bf4e0d64cd8fea8f>`() const;
	};

	// direct descendants

	template <
		class PROCESS_LAUNCHER,
		const char* LAUNCHER_TYPE,
		class ... LAUNCHER_COMMANDS
	>
	class :ref:`ProcessLauncher<doxid-class_process_launcher>`;

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

.. _details-class_process_launcher_interface:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Functions for all process launchers.

Methods
-------

.. index:: pair: function; launcherName
.. _doxid-class_process_launcher_interface_1a525cd3685e3eff9a4d1ef17bf4ba06ab:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual std::string launcherName() const = 0

Get name of launcher.

.. index:: pair: function; createLauncher
.. _doxid-class_process_launcher_interface_1a677aa140f965174b25ba89afa7556983:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>` createLauncher() = 0

Create a new proces launcher.

.. index:: pair: function; launchEngineProcess
.. _doxid-class_process_launcher_interface_1ad39354640a56be631da38f6db4f301b2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual pid_t launchEngineProcess(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
		const std::vector<std::string>& envParams,
		const std::vector<std::string>& startParams,
		bool appendParentEnv = true
	) = 0

Fork a new process for the given engine. Will read environment variables and start params from engineConfig.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineConfig

		- Engine Configuration. Env variables and start params take precedence over envParams and startParams

	*
		- envParams

		- Additional Environment Variables for child process. Will take precedence over default env params if appendParentEnv is true

	*
		- startParams

		- Additional Start parameters

	*
		- appendParentEnv

		- Should parent env variables be appended to child process



.. rubric:: Returns:

Returns Process ID of child process on success

.. index:: pair: function; stopEngineProcess
.. _doxid-class_process_launcher_interface_1a8c350cf5794790b991d3a42c943e9677:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual pid_t stopEngineProcess(unsigned int killWait) = 0

Stop a running engine process.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- killWait

		- Time (in seconds) to wait for process to quit by itself before force killing it. 0 means it will wait indefinitely



.. rubric:: Returns:

Returns child PID on success, negative value on error

.. index:: pair: function; getProcessStatus
.. _doxid-class_process_launcher_interface_1afecd8b121e9b5df59a47584a93809b94:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>` getProcessStatus()

Get the current engine process status. If status cannot be retrieved, return ENGINE_RUNNING_STATUS::UNKNOWN.



.. rubric:: Returns:

Returns status as enum :ref:`ProcessLauncherInterface::ENGINE_RUNNING_STATUS <doxid-class_process_launcher_interface_1ad958854e32d7a961c7bb07bd23e088b5>`

.. index:: pair: function; launchCommand
.. _doxid-class_process_launcher_interface_1adde0e1c47dd675a2bf4e0d64cd8fea8f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`LaunchCommandInterface<doxid-class_launch_command_interface>`* launchCommand() const

Get Launch Command. If launchEngineProcess has not yet been called, return nullptr.

