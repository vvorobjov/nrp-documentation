.. index:: pair: page; Process Launcher
.. _doxid-process_launcher:

Process Launcher
================

The :ref:`ProcessLauncher <doxid-class_process_launcher>` is in control of spawning Engines in individual computational environments. Depending on the desired behavior, engines can be created in separate processes, containers, or machines.

Default settings specify forking the NRPCoreSim process and launching the engine in the newly spawned child process. This setup suffices for local simulation execution, however a more complex launching process may be desired for online simulation.

Developers can create new process launchers by creating a new class derived from the :ref:`ProcessLauncher <doxid-class_process_launcher>` template. This new class must define the different :ref:`launch commands <doxid-process_launcher_1launch_command>` available to this type of machine, as well as the name of the configuration. Also, the new class must be made visible to the NRPCoreSim executable by adding it to the MainProcessLauncherManager definition in ``nrp_general_library/process_launchers/process_launcher_manager.h``.

Once this is done, the user can set the NRPCoreSim to utilize the new behavior by changing the ``ProcessLauncherType`` parameter in the simulation configuration file. The default value of this parameter is ``Basic``, which will instruct NRPCoreSim to use :ref:`ProcessLauncherBasic <doxid-class_process_launcher_basic>`.

An example of a new process launcher is shown here. Note the ``LAUNCH_CMD#``, which will be discussed later.

.. ref-code-block:: cpp

	// Create a new ProcessLauncher. This launcher can be selected by changing the ProcessLauncherType parameter in the simulation configuration file to "NewBehavior"
	class NewProcessLauncher
	    : public :ref:`ProcessLauncher <doxid-class_process_launcher>`<NewProcessLauncher, "NewBehavior", LAUNCH_CMD1, LAUNCH_CMD2, ...>
	{}

Add it to the MainProcessLauncherManager along with the already existing basic :ref:`ProcessLauncher <doxid-class_process_launcher>` :

.. ref-code-block:: cpp

	...
	// Include header with defined class
	#include "new_process_launcher.h"
	...
	using :ref:`MainProcessLauncherManager <doxid-class_process_launcher_manager>` = :ref:`ProcessLauncherManager\<ProcessLauncherBasic, NewProcessLauncher> <doxid-class_process_launcher_manager>`;

All ProcessLaunchers must contain at least one Launch Command.



.. _doxid-process_launcher_1launch_command:

Launch Commands
~~~~~~~~~~~~~~~

The above example mentions a series of ``LAUNCH_CMD#`` s. In these classes, the actual launching process is declared. Thus, a single :ref:`ProcessLauncher <doxid-class_process_launcher>` behavior can actually define multiple types of Engine launch commands. All ProcessLaunchers must define at least one :ref:`LaunchCommand <doxid-class_launch_command>`. Should an Engine require non-default startup methods, these can be added as ``LAUNCH_CMD`` s to the relevant ProcessLaunchers.

In the simulation configuration file, it can be specified the launch command that will be used to launch each of the engines in the experiment. This is done by setting the parameter ``EngineLaunchCommand``, which default value is ``:ref:`BasicFork <doxid-class_basic_fork>```.

To define LaunchCommands, developers can create a new class, derived from :ref:`LaunchCommand <doxid-class_launch_command>`. They must declare the virtual function described below. For an additional example, look at :ref:`BasicFork <doxid-class_basic_fork>`.

.. ref-code-block:: cpp

	// Define a new command. Engines can use it by setting the EngineLaunchCommand parameter in their configuration file to "NewCommand"
	class NewLaunchCommand
	        : public :ref:`LaunchCommand <doxid-class_launch_command>`<"NewCommand">
	{
	    public:
	        // Launch a new engine. It takes the engine configuration, environment parameters and process start parameters and launch the 
	        // engine according to its implemented behavior. If appendParentEnv is set to true, use the parent environment in the engine process. If set to false, scrub it before continuing.
	        pid_t :ref:`launchEngineProcess <doxid-class_launch_command_interface_1a29b04be0f3b930a0121b532392561d21>`(const :ref:`nlohmann::json <doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` &engineConfig, const std::vector<std::string> &envParams,
	                                  const std::vector<std::string> &startParams, bool appendParentEnv) override;
	
	        // Stop the engine. Note that this command should be run after the Engine's shutdown routines have already been called.
	        // It should try to gracefully quit the engine process. Should the Engine process not have shut down after killWait seconds,
	        // forcefully shut down the process, e.g. by sending a SIGKILL command
	        pid_t :ref:`stopEngineProcess <doxid-class_launch_command_interface_1a401550deee9cc8bedeff819716f8d4e3>`(unsigned int killWait) override;
	
	        // Get the current engine process status. If status cannot be retrieved by this process launcher command,
	        // return ENGINE_RUNNING_STATUS::UNKNOWN
	        virtual ENGINE_RUNNING_STATUS :ref:`getProcessStatus <doxid-class_launch_command_interface_1a67559c48de23dcd6f2b75f3098d555bc>`()
	};

