.. index:: pair: class; BasicFork
.. _doxid-class_basic_fork:

class BasicFork
===============

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <basic_fork.h>
	
	class BasicFork: public :ref:`LaunchCommand<doxid-class_launch_command>` {
	public:
		// methods
	
		virtual pid_t :ref:`launchEngineProcess<doxid-class_basic_fork_1a84abadb409228ce01e5a2d2575e7b6e0>`(
			const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
			const std::vector<std::string>& envParams,
			const std::vector<std::string>& startParams,
			bool appendParentEnv = true
		);
	
		virtual pid_t :ref:`stopEngineProcess<doxid-class_basic_fork_1aa4530a049191dbc6905204723de3e337>`(unsigned int killWait);
		virtual :ref:`ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>` :ref:`getProcessStatus<doxid-class_basic_fork_1a32b88a5458bad8e180972f2f664b6838>`();
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

.. _details-class_basic_fork:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; launchEngineProcess
.. _doxid-class_basic_fork_1a84abadb409228ce01e5a2d2575e7b6e0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual pid_t launchEngineProcess(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& engineConfig,
		const std::vector<std::string>& envParams,
		const std::vector<std::string>& startParams,
		bool appendParentEnv = true
	)

Fork a new process for the given engine. Will read environment variables and start params from engineConfig The function should take the environment parameters and start parameters defined in engineConfig, and append any additional strings defined in envParams and startParams before starting the Engine specified in engineConfig. If appendParentEnv is set to true, use the parent environment in the forked child. If set to false, scrub the environment before continuing.



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

Returns PID of child process on success

.. index:: pair: function; stopEngineProcess
.. _doxid-class_basic_fork_1aa4530a049191dbc6905204723de3e337:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual pid_t stopEngineProcess(unsigned int killWait)

Stop a running engine process.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- killWait

		- Time (in seconds) to wait for process to quit by itself before force killing it. 0 means it will wait indefinetly



.. rubric:: Returns:

Returns 0 on success, negative value on error

.. index:: pair: function; getProcessStatus
.. _doxid-class_basic_fork_1a32b88a5458bad8e180972f2f664b6838:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`ENGINE_RUNNING_STATUS<doxid-class_launch_command_interface_1a8f892914289fc45824ba408070b03056>` getProcessStatus()

Get the current engine process status. If status cannot be retrieved, return ENGINE_RUNNING_STATUS::UNKNOWN.



.. rubric:: Returns:

Returns status as enum :ref:`ProcessLauncherInterface::ENGINE_RUNNING_STATUS <doxid-class_process_launcher_interface_1ad958854e32d7a961c7bb07bd23e088b5>`

