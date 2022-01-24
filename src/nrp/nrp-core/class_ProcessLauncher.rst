.. index:: pair: class; ProcessLauncher
.. _doxid-class_process_launcher:

template class ProcessLauncher
==============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Base class for all process launchers. :ref:`More...<details-class_process_launcher>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <process_launcher.h>
	
	template <
		class PROCESS_LAUNCHER,
		const char* LAUNCHER_TYPE,
		class ... LAUNCHER_COMMANDS
	>
	class ProcessLauncher: public :ref:`ProcessLauncherInterface<doxid-class_process_launcher_interface>` {
	public:
		// fields
	
		static constexpr auto :target:`LauncherType<doxid-class_process_launcher_1a706fe2bfbcd16a9566bfeb37a0f7f1b2>` = LAUNCHER_TYPE;

		// methods
	
		virtual :ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>` :ref:`createLauncher<doxid-class_process_launcher_1a9438e1a2e43d9aa7db49cca73c769965>`();
		virtual std::string :ref:`launcherName<doxid-class_process_launcher_1aac9a1d7097eae647725e2ea9dedc72c9>`() const;
	
		virtual pid_t :ref:`launchEngineProcess<doxid-class_process_launcher_1a44fcbb3ec4202d8d5d491b1ded319a84>`(
			const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
			const std::vector<std::string>& envParams,
			const std::vector<std::string>& startParams,
			bool appendParentEnv = true
		);
	
		virtual pid_t :ref:`stopEngineProcess<doxid-class_process_launcher_1a65c14f21b53a4c92716d7dd995605c78>`(unsigned int killWait);
	};

	// direct descendants

	class :ref:`ProcessLauncherBasic<doxid-class_process_launcher_basic>`;

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
		typedef :ref:`LaunchCommandInterface::ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>` :ref:`ENGINE_RUNNING_STATUS<doxid-class_process_launcher_interface_1ad958854e32d7a961c7bb07bd23e088b5>`;

		// fields
	
		static constexpr auto :ref:`UNKNOWN<doxid-class_process_launcher_interface_1a8ce968ad8b7e9a8fd23b20032133cd11>` = LaunchCommandInterface::ENGINE_RUNNING_STATUS::UNKNOWN;
		static constexpr auto :ref:`RUNNING<doxid-class_process_launcher_interface_1a29b24978e039d71a362509c857124e1a>` = LaunchCommandInterface::ENGINE_RUNNING_STATUS::RUNNING;
		static constexpr auto :ref:`STOPPED<doxid-class_process_launcher_interface_1a57eb1ac4597c3c830bf9c876447e819e>` = LaunchCommandInterface::ENGINE_RUNNING_STATUS::STOPPED;

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

.. _details-class_process_launcher:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Base class for all process launchers.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- PROCESS_LAUNCHER

		- Final class derived from :ref:`ProcessLauncher <doxid-class_process_launcher>`

	*
		- LAUNCHER_TYPE

		- Launcher Type as string

Methods
-------

.. index:: pair: function; createLauncher
.. _doxid-class_process_launcher_1a9438e1a2e43d9aa7db49cca73c769965:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`ProcessLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>` createLauncher()

Create a new proces launcher.

.. index:: pair: function; launcherName
.. _doxid-class_process_launcher_1aac9a1d7097eae647725e2ea9dedc72c9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual std::string launcherName() const

Get name of launcher.

.. index:: pair: function; launchEngineProcess
.. _doxid-class_process_launcher_1a44fcbb3ec4202d8d5d491b1ded319a84:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual pid_t launchEngineProcess(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
		const std::vector<std::string>& envParams,
		const std::vector<std::string>& startParams,
		bool appendParentEnv = true
	)

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
.. _doxid-class_process_launcher_1a65c14f21b53a4c92716d7dd995605c78:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual pid_t stopEngineProcess(unsigned int killWait)

Stop a running engine process.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- killWait

		- Time (in seconds) to wait for process to quit by itself before force killing it. 0 means it will wait indefinitely



.. rubric:: Returns:

Returns child PID on success, negative value on error

