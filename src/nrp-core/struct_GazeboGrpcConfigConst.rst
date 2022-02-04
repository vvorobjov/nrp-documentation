.. index:: pair: struct; GazeboGrpcConfigConst
.. _doxid-struct_gazebo_grpc_config_const:

struct GazeboGrpcConfigConst
============================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <gazebo_grpc_config.h>
	
	struct GazeboGrpcConfigConst {
		// fields
	
		static constexpr char :target:`EngineType<doxid-struct_gazebo_grpc_config_const_1af5a59d7148915d27cc1e60819bcd637a>`[] = "gazebo_grpc";
		static constexpr char :target:`EngineSchema<doxid-struct_gazebo_grpc_config_const_1a1cea9800658b41270c9cdf4ffe2cb2a8>`[] = "https://neurorobotics.net/engines/:ref:`engines_gazebo.json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`#/engine_gazebo_grpc";
		static constexpr std::string_view :ref:`GazeboPluginArg<doxid-struct_gazebo_grpc_config_const_1a257775fe90de010e1782ec047e8ca229>` = "-s";
		static constexpr std::string_view :ref:`GazeboRNGSeedArg<doxid-struct_gazebo_grpc_config_const_1a66e9a2c8f65b74856cd536320d87eb6e>` = "--seed";
	};
.. _details-struct_gazebo_grpc_config_const:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. index:: pair: variable; GazeboPluginArg
.. _doxid-struct_gazebo_grpc_config_const_1a257775fe90de010e1782ec047e8ca229:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view GazeboPluginArg = "-s"

Gazebo Start Parameters Argument for plugins.

.. index:: pair: variable; GazeboRNGSeedArg
.. _doxid-struct_gazebo_grpc_config_const_1a66e9a2c8f65b74856cd536320d87eb6e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view GazeboRNGSeedArg = "--seed"

Gazebo Start Parameters Argument for random seed value.

