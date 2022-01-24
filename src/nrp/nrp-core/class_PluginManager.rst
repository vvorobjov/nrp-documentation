.. index:: pair: class; PluginManager
.. _doxid-class_plugin_manager:

class PluginManager
===================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Loads libraries and extracts engine launchers. :ref:`More...<details-class_plugin_manager>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <plugin_manager.h>
	
	class PluginManager {
	public:
		// methods
	
		:ref:`EngineLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>` :ref:`loadPlugin<doxid-class_plugin_manager_1a94abd39547ba36fec39bfdbbd1f17483>`(const std::string& pluginLibFile);
		void :ref:`addPluginPath<doxid-class_plugin_manager_1aae4c9a8c7f8c1f74fac06307b88acf22>`(const std::string& pluginPath);
	};
.. _details-class_plugin_manager:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Loads libraries and extracts engine launchers.

Methods
-------

.. index:: pair: function; loadPlugin
.. _doxid-class_plugin_manager_1a94abd39547ba36fec39bfdbbd1f17483:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`EngineLauncherInterface::unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>` loadPlugin(const std::string& pluginLibFile)

Load a Plugin from a given library.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- pluginLibFile

		- Plugin library file (.so)



.. rubric:: Returns:

Returns ptr to loaded EngineLauncher if found, nullptr otherwise

.. index:: pair: function; addPluginPath
.. _doxid-class_plugin_manager_1aae4c9a8c7f8c1f74fac06307b88acf22:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void addPluginPath(const std::string& pluginPath)

Adds search path under which to look for plugins.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- pluginPath

		- Path to plugins

