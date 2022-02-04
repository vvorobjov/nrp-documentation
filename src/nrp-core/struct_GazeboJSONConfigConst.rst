.. index:: pair: struct; GazeboJSONConfigConst
.. _doxid-struct_gazebo_j_s_o_n_config_const:

struct GazeboJSONConfigConst
============================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <gazebo_json_config.h>
	
	struct GazeboJSONConfigConst {
		// fields
	
		static constexpr char :target:`EngineType<doxid-struct_gazebo_j_s_o_n_config_const_1a8712a9c3c63ec96a486092b89e2a6a9a>`[] = "gazebo_json";
		static constexpr char :target:`EngineSchema<doxid-struct_gazebo_j_s_o_n_config_const_1aebf5499573d3a92aabec1a669741843c>`[] = "https://neurorobotics.net/engines/:ref:`engines_gazebo.json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`#/engine_gazebo_json";
		static constexpr std::string_view :ref:`GazeboPluginArg<doxid-struct_gazebo_j_s_o_n_config_const_1abdcff1882e2db6aef6b11ad3424c6de0>` = "-s";
		static constexpr std::string_view :ref:`GazeboRNGSeedArg<doxid-struct_gazebo_j_s_o_n_config_const_1a84e800c1318b4de50c2e4a18b67ec89a>` = "--seed";
	};
.. _details-struct_gazebo_j_s_o_n_config_const:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. index:: pair: variable; GazeboPluginArg
.. _doxid-struct_gazebo_j_s_o_n_config_const_1abdcff1882e2db6aef6b11ad3424c6de0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view GazeboPluginArg = "-s"

Gazebo Start Parameters Argument for plugins.

.. index:: pair: variable; GazeboRNGSeedArg
.. _doxid-struct_gazebo_j_s_o_n_config_const_1a84e800c1318b4de50c2e4a18b67ec89a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view GazeboRNGSeedArg = "--seed"

Gazebo Start Parameters Argument for random seed value.

