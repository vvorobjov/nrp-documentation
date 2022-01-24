.. index:: pair: page; Engine Plugin System
.. _doxid-plugin_system:

Engine Plugin System
====================

The :ref:`NRPCoreSim <doxid-nrp_simulation>` executable can load new engine types on startup. The names of additional Engine libraries can be supplied via a "-p" parameter, followed by a comma separated list of engines to load. This gives users the ability to add their own engines on startup and create new simulation configurations with them.

The NRPCoreSim loads one instance of :ref:`PluginManager <doxid-class_plugin_manager>` on startup, and fills it with a set of predefined search directories. Should a user request an additional engine library be loaded, it will iterate over these locations until if finds a library file matching the requested string. The user can request both a relative as well as an absolute path, both will be searched.

CMake is configured to load a list of engine libraries by default. These are defines in the root CMakeLists.txt file, in the NRP_SIMULATION_DEFAULT_ENGINE_LAUNCHERS variable, and will not have to be addded via the "-p" start parameter. By default this variable is set to "NRPPythonJSONEngine.so", and thus :ref:`Python JSON Engine <doxid-python_json_engine>` is always loaded. Any other additional engine plugin required to execute an experiment must be passed to NRPCoreSim using the "-p" parameter.

Creating C plugins is relatively straightforward. We have provided a macro to help developers create new engine libraries. The macro is defined in ``nrp_general_library/plugin_system/plugin.h`` as :ref:`CREATE_NRP_ENGINE_LAUNCHER(...) <doxid-plugin_8h_1adcd291c2e449ed4d17ddfb6476b1d246>`. It takes the EngineLauncher of the loaded Engine as parameter. One library can only include one engine. For example:

.. ref-code-block:: cpp

	:ref:`CREATE_NRP_ENGINE_LAUNCHER <doxid-plugin_8h_1adcd291c2e449ed4d17ddfb6476b1d246>`(GazeboEngineJSONNRPClient::EngineLauncher<"gazebo_json">)

A more detailed description of EngineLaunchers can be found :ref:`here <doxid-engines_1engine_launchers>`.

Once a plugin has been loaded, the NRPCoreSim will add the newly loaded launcher to the running :ref:`EngineLauncherManager <doxid-class_engine_launcher_manager>`, making it available for Engine creation.

